/*
 * MPU6050.c - 六轴传感器驱动 + 体感跟随
 *
 * 上电时把当前姿势记为 90 度基准，之后 roll/pitch/yaw 映射到关节：
 *   roll  -> 底座 + 腕旋转    pitch -> 大臂/小臂/腕俯仰    yaw -> 腕旋转
 *   快速摇晃 -> 夹爪开合（SHAKE_ENABLE 控制，默认关）
 * 链路：死区 -> 增益 -> 关节限位 -> 速度限幅。
 */
#include "MPU6050.h"
#include "RobotArm.h"
#include "NetLink.h"
#include "IAP.h"
#include "i2c.h"
#include <math.h>

/* ===================== 可调参数 ===================== */
#define ACC_LSB_PER_G 16384.0f /* ±2g  量程 */
#define GYRO_LSB_PER_DPS 16.4f /* ±2000dps 量程 */
#define RAD_TO_DEG 57.2957795f

#define FUSION_ALPHA 0.95f    /* 互补滤波陀螺权重，越大越平滑 */
#define GESTURE_PERIOD_MS 20  /* 50Hz 控制周期 */
#define GESTURE_DEADBAND 3.0f /* 死区（度），抑制手抖 */

/* ---- 姿态 → 关节 映射增益：正负号代表方向，若某个关节转向相反，把它取负即可 ---- */
#define BASE_GAIN 1.20f         /* 左右倾斜 → 底座旋转 */
#define SHOULDER_GAIN 1.20f     /* 前后俯仰 → 大臂抬落（抬头=抬臂） */
#define ELBOW_GAIN -0.40f       /* 小臂跟随，负值 = 与大臂呈折叠关系 */
#define WRIST_PITCH_GAIN -0.70f /* 腕俯仰补偿，让夹爪尽量保持水平 */
#define WRIST_ROLL_GAIN -1.00f  /* 手腕扭转 → 腕部旋转 */

/* ---- 关节安全限位（度），必须落在舵机实际可达范围内，防止顶死堵转 ----
 * 注意：这几个值要和 ESP 端 arm_ik.h 里的 ARM_AXIS 保持一致，
 *       否则 ESP 算出来的角度会被 STM32 二次裁剪，末端就到不了目标点。
 * 坐标（IK）模式需要更宽的活动范围，所以给到 10~170；如果你的机械臂
 * 有结构限位，按实际收紧。 */
#define BASE_MIN_DEG 10
#define BASE_MAX_DEG 170
#define SHOULDER_MIN_DEG 10
#define SHOULDER_MAX_DEG 170
#define ELBOW_MIN_DEG 10
#define ELBOW_MAX_DEG 170
#define WRIST_P_MIN_DEG 0
#define WRIST_P_MAX_DEG 180
#define WRIST_R_MIN_DEG 10
#define WRIST_R_MAX_DEG 170
#define JOINT_SPEED_DPS 180  /* 关节最大速度 180°/s（MG996R 约 350°/s，留余量） */
#define GRIP_SPEED_DPS 400   /* 夹爪开合速度：行程短，放开跑才跟手 */
#define JOINT_ACCEL_DPS2 900 /* 关节加速度：越大起步越跟手，太大会抖 */
#define GRIP_ACCEL_DPS2 4000 /* 夹爪加速度：负载轻，可以给大 */

/* ---- 摇一摇识别 ----
 * SHAKE_ENABLE = 0：默认关闭。机械臂自身运动/舵机抖动的加速度很容易超过门槛，
 *                 会变成“夹爪自己一直开合”的死循环（动一下 → 触发 → 再动 → 再触发）。
 * 需要这个功能时改成 1，并且要满足“整机静止 ≥ SHAKE_IDLE_MS”才开始计数。 */
#define SHAKE_ENABLE 0
#define SHAKE_PEAK_G 0.80f    /* 判定为“抖一下”的加速度门槛(g) */
#define SHAKE_PEAK_COUNT 3    /* 窗口内抖够 3 下才切换 */
#define SHAKE_WINDOW_MS 700   /* 识别窗口 */
#define SHAKE_LOCKOUT_MS 1500 /* 触发后冷却，防止连发 */
#define SHAKE_IDLE_MS 1200    /* 六轴静止这么久之后才允许识别 */

/* ---- 航向角 ---- */
#define YAW_LEAK_PER_S 0.10f /* 每秒向 0 回拉的比例，抵消陀螺漂移 */
#define YAW_LIMIT_DEG 90.0f

/* ===================== 内部状态 ===================== */
static int16_t s_gyro_bias[3] = {0, 0, 0};
static uint8_t s_att_inited = 0;
static float s_att_pitch = 0.0f; /* 互补滤波状态必须常驻，不能放在调用者的临时变量里 */
static float s_att_roll = 0.0f;
static float s_ref_pitch = 0.0f; /* “中位”基准 */
static float s_ref_roll = 0.0f;
static float s_yaw = 0.0f;
static uint8_t s_grip_target = GRIP_CLOSE;

/* ===================== 硬件 I2C2（PB10 = SCL, PB11 = SDA） =====================
 * 底层用 CubeMX 生成的 I2C2：main() 里的 MX_I2C2_Init() 已经把 PB10/PB11
 * 配成复用开漏并初始化好外设（默认 100kHz），驱动里不再自己操作引脚。
 */
#define MPU_I2C_TIMEOUT_MS 20

/** 写一个寄存器，返回 0 表示成功 */
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1U,
                           MPU_I2C_TIMEOUT_MS);
    return (st == HAL_OK) ? 0U : 1U;
}

/** 连续读多个寄存器，返回 0 表示成功 */
static uint8_t I2C_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)len,
                          MPU_I2C_TIMEOUT_MS);
    return (st == HAL_OK) ? 0U : 1U;
}

/* ===================== 传感器配置 ===================== */
static void MPU_SetGyroFsr(uint8_t fsr) {
    /* fsr: 0=±250, 1=±500, 2=±1000, 3=±2000 dps */
    I2C_WriteReg(MPU_GYRO_CFG_REG, (uint8_t)(fsr << 3));
}

static void MPU_SetAccelFsr(uint8_t fsr) {
    /* fsr: 0=±2, 1=±4, 2=±8, 3=±16 g */
    I2C_WriteReg(MPU_ACCEL_CFG_REG, (uint8_t)(fsr << 3));
}

static void MPU_SetLPF(uint16_t lpf_hz) {
    uint8_t cfg;

    if (lpf_hz >= 188U)
        cfg = 1;
    else if (lpf_hz >= 98U)
        cfg = 2;
    else if (lpf_hz >= 42U)
        cfg = 3;
    else if (lpf_hz >= 20U)
        cfg = 4;
    else if (lpf_hz >= 10U)
        cfg = 5;
    else
        cfg = 6;
    I2C_WriteReg(MPU_CFG_REG, cfg);
}

static void MPU_SetRate(uint16_t rate_hz) {
    uint8_t div;

    if (rate_hz > 1000U)
        rate_hz = 1000U;
    if (rate_hz < 4U)
        rate_hz = 4U;
    div = (uint8_t)(1000U / rate_hz - 1U);
    I2C_WriteReg(MPU_SAMPLE_RATE_REG, div);
    MPU_SetLPF((uint16_t)(rate_hz / 2U));
}

uint8_t MPU6050_ReadID(void) {
    uint8_t id = 0;

    if (I2C_ReadRegs(MPU_DEVICE_ID_REG, &id, 1) != 0)
        return 0;
    return id;
}

/** 总线卡死恢复：把 PB10/PB11 当普通 GPIO 打 9 个时钟 + STOP，再重建 I2C2 */
static void I2C_BusRecover(void) {
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    HAL_I2C_DeInit(&hi2c2); /* 先释放外设，才能手动接管引脚 */

    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); /* 释放 SDA */
    for (i = 0; i < 9; i++) {                            /* 打 9 个时钟，让从机把半截传输吐完 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
    }
    /* 补一个 STOP：SCL 高电平期间 SDA 由低到高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1);

    MX_I2C2_Init(); /* 外设和 PB10/PB11 复用重新配好 */
}

uint8_t MPU6050_Init(void) {
    uint8_t i;

    /* 1. I2C2 已由 main() 里的 MX_I2C2_Init() 初始化好（PB10/PB11，100kHz），
     *    这里不用再配引脚，直接按 MPU6050 的时序复位唤醒即可 */

    /* 2. 复位并唤醒 */
    I2C_WriteReg(MPU_PWR_MGMT1_REG, 0x80); /* DEVICE_RESET */
    HAL_Delay(100);
    I2C_WriteReg(MPU_PWR_MGMT1_REG, 0x00); /* 解除睡眠，内部 8MHz 时钟 */
    HAL_Delay(10);

    /* 3. 检查器件 */
    for (i = 0; i < 3; i++) {
        if (MPU6050_ReadID() == 0x68)
            break;
        I2C_BusRecover();
        HAL_Delay(5);
    }
    if (i >= 3)
        return 1;

    /* 4. 基本配置：PLL 时钟、±2000dps、±2g、100Hz 采样 */
    I2C_WriteReg(MPU_PWR_MGMT1_REG, 0x01); /* 时钟源 = X 轴陀螺 PLL，最稳 */
    I2C_WriteReg(MPU_PWR_MGMT2_REG, 0x00); /* 六轴全部打开 */
    MPU_SetGyroFsr(3);
    MPU_SetAccelFsr(0);
    MPU_SetRate(100);
    I2C_WriteReg(MPU_INT_EN_REG, 0x00);
    I2C_WriteReg(MPU_USER_CTRL_REG, 0x00);
    I2C_WriteReg(MPU_FIFO_EN_REG, 0x00);
    I2C_WriteReg(MPU_INTBP_CFG_REG, 0x80); /* INT 高电平有效、推挽 */
    HAL_Delay(10);

    /* 5. 陀螺零偏校准（此时传感器应静止） */
    MPU6050_CalibrateGyro(200);

    s_att_inited = 0;
    s_yaw = 0.0f;
    return 0;
}

void MPU6050_CalibrateGyro(uint16_t samples) {
    int32_t sum[3] = {0, 0, 0};
    uint8_t buf[6];
    uint16_t i;

    if (samples == 0U)
        samples = 1U;

    for (i = 0; i < samples; i++) {
        if (I2C_ReadRegs(MPU_GYRO_XOUTH_REG, buf, 6) == 0) {
            sum[0] += (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
            sum[1] += (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
            sum[2] += (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
        }
        HAL_Delay(2);
    }

    s_gyro_bias[0] = (int16_t)(sum[0] / (int32_t)samples);
    s_gyro_bias[1] = (int16_t)(sum[1] / (int32_t)samples);
    s_gyro_bias[2] = (int16_t)(sum[2] / (int32_t)samples);
}

/* ===================== 数据读取 ===================== */
uint8_t MPU6050_ReadRaw(MPU6050_t *data) {
    static uint8_t s_fail = 0;
    uint8_t buf[14];

    /* 0x3B 起连续 14 字节：ax ay az temp gx gy gz，一次读完保证时戳一致 */
    if (I2C_ReadRegs(MPU_ACCEL_XOUTH_REG, buf, 14) != 0) {
        data->ok = 0;
        /* 连续 3 次读不到（拔线/干扰/总线被从机拉死）就重建一次 I2C2，避免一直卡死 */
        if (++s_fail >= 3U) {
            s_fail = 0;
            I2C_BusRecover();
        }
        return 0;
    }
    s_fail = 0;

    data->ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    data->ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    data->az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    data->temp_c = ((float)(int16_t)(((uint16_t)buf[6] << 8) | buf[7])) / 340.0f + 36.53f;
    data->gx = (int16_t)((int16_t)(((uint16_t)buf[8] << 8) | buf[9]) - s_gyro_bias[0]);
    data->gy = (int16_t)((int16_t)(((uint16_t)buf[10] << 8) | buf[11]) - s_gyro_bias[1]);
    data->gz = (int16_t)((int16_t)(((uint16_t)buf[12] << 8) | buf[13]) - s_gyro_bias[2]);
    data->ok = 1;

    return 1;
}

/* ===================== 姿态解算（互补滤波） ===================== */
void MPU6050_UpdateAttitude(MPU6050_t *data, float dt) {
    float ax = (float)data->ax;
    float ay = (float)data->ay;
    float az = (float)data->az;
    float gx, gy, gz;
    float roll_acc, pitch_acc;

    if (dt <= 0.0f)
        dt = 0.02f;

    /* 1. 加速度计解算：静止时以重力方向为绝对参考 */
    roll_acc = atan2f(ay, az) * RAD_TO_DEG;
    pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /* 2. 陀螺仪角速度（°/s） */
    gx = (float)data->gx / GYRO_LSB_PER_DPS;
    gy = (float)data->gy / GYRO_LSB_PER_DPS;
    gz = (float)data->gz / GYRO_LSB_PER_DPS;

    if (!s_att_inited) {
        s_att_roll = roll_acc;
        s_att_pitch = pitch_acc;
        s_att_inited = 1;
    } else {
        /* 3. 互补滤波：高频信陀螺，低频信加速度
         *    roll 绕 X 轴，用 gx 积分；pitch 绕 Y 轴，用 gy 积分 */
        s_att_roll = FUSION_ALPHA * (s_att_roll + gx * dt) + (1.0f - FUSION_ALPHA) * roll_acc;
        s_att_pitch = FUSION_ALPHA * (s_att_pitch + gy * dt) + (1.0f - FUSION_ALPHA) * pitch_acc;
    }

    data->roll = s_att_roll;
    data->pitch = s_att_pitch;

    /* 4. 航向角只能靠陀螺积分，加一个缓慢回零的“漏”抑制漂移 */
    s_yaw += gz * dt;
    s_yaw -= s_yaw * YAW_LEAK_PER_S * dt;
    if (s_yaw > YAW_LIMIT_DEG)
        s_yaw = YAW_LIMIT_DEG;
    if (s_yaw < -YAW_LIMIT_DEG)
        s_yaw = -YAW_LIMIT_DEG;
    data->yaw = s_yaw;

    /* 5. 动态加速度（去掉 1g 重力），给摇一摇识别用 */
    data->dyn_g = sqrtf(ax * ax + ay * ay + az * az) / ACC_LSB_PER_G - 1.0f;
}

void MPU6050_GetAttitude(float *pitch, float *roll, float *yaw) {
    MPU6050_t d = {0};

    if (MPU6050_ReadRaw(&d)) {
        MPU6050_UpdateAttitude(&d, 0.02f);
    }
    if (pitch)
        *pitch = d.pitch;
    if (roll)
        *roll = d.roll;
    if (yaw)
        *yaw = d.yaw;
}

/** 只读取缓存姿态，不碰 I2C（给 OLED 等其它任务用） GestureStep 每次算完都会更新
 * s_att_pitch/s_att_roll/s_yaw */
void MPU6050_GetCachedAttitude(float *pitch, float *roll, float *yaw) {
    if (pitch)
        *pitch = s_att_pitch - s_ref_pitch;
    if (roll)
        *roll = s_att_roll - s_ref_roll;
    if (yaw)
        *yaw = s_yaw;
}

void MPU6050_GetRelativeAttitude(float *pitch, float *roll, float *yaw) {
    MPU6050_t d = {0};

    if (MPU6050_ReadRaw(&d)) {
        MPU6050_UpdateAttitude(&d, 0.02f);
    }
    if (pitch)
        *pitch = d.pitch - s_ref_pitch;
    if (roll)
        *roll = d.roll - s_ref_roll;
    if (yaw)
        *yaw = s_yaw;
}

void MPU6050_CaptureNeutral(void) {
    MPU6050_t d = {0};
    float sum_p = 0.0f;
    float sum_r = 0.0f;
    uint16_t n = 0;
    uint16_t i;

    /* 取 40 次平均，避免单次抖动把基准带偏（约 0.2s，期间保持不动） */
    for (i = 0; i < 40; i++) {
        if (MPU6050_ReadRaw(&d)) {
            MPU6050_UpdateAttitude(&d, 0.02f);
            sum_p += d.pitch;
            sum_r += d.roll;
            n++;
        }
        HAL_Delay(5);
    }

    if (n > 0U) {
        s_ref_pitch = sum_p / (float)n;
        s_ref_roll = sum_r / (float)n;
    }
    s_yaw = 0.0f;
}

void MPU6050_ResetYaw(void) {
    s_yaw = 0.0f;
}

/* ===================== 摇一摇手势识别 ===================== */
#if SHAKE_ENABLE
static uint8_t Shake_Detect(float dyn_g) {
    static uint8_t peak_cnt = 0;
    static uint8_t armed = 1;
    static uint32_t first_ms = 0;
    static uint32_t lock_ms = 0;

    uint32_t now = HAL_GetTick();
    float mag = fabsf(dyn_g);

    if (now < lock_ms)
        return 0;                             /* 冷却期内不识别 */
    if ((now - first_ms) > SHAKE_WINDOW_MS) { /* 窗口过期，重新计数 */
        peak_cnt = 0;
    }

    if (armed && (mag > SHAKE_PEAK_G)) {
        if (peak_cnt == 0)
            first_ms = now;
        peak_cnt++;
        armed = 0; /* 等回落到低位再数下一次 */
        if (peak_cnt >= SHAKE_PEAK_COUNT) {
            peak_cnt = 0;
            lock_ms = now + SHAKE_LOCKOUT_MS;
            return 1;
        }
    } else if (!armed && (mag < (SHAKE_PEAK_G * 0.4f))) {
        armed = 1;
    }

    return 0;
}
#endif /* SHAKE_ENABLE */

/* ===================== 姿态 → 关节角度 ===================== */
static uint8_t Map_Joint(float delta_deg, float gain, int16_t min_deg, int16_t max_deg) {
    float angle = (float)ROBOTARM_NEUTRAL + delta_deg * gain;

    if (angle < (float)min_deg)
        angle = (float)min_deg;
    if (angle > (float)max_deg)
        angle = (float)max_deg;
    return (uint8_t)(angle + 0.5f);
}

/* ===================== 体感控制主流程 ===================== */
uint8_t MPU6050_GestureInit(void) {
    uint8_t i;

    /* 1. 舵机：启动 PWM，回中，设置限位与速度 */
    RobotArm_Init();

    RobotArm_SetLimit(ARM_BASE, BASE_MIN_DEG, BASE_MAX_DEG);
    RobotArm_SetLimit(ARM_SHOULDER, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
    RobotArm_SetLimit(ARM_ELBOW, ELBOW_MIN_DEG, ELBOW_MAX_DEG);
    RobotArm_SetLimit(ARM_WRIST_PITCH, WRIST_P_MIN_DEG, WRIST_P_MAX_DEG);
    RobotArm_SetLimit(ARM_WRIST_ROLL, WRIST_R_MIN_DEG, WRIST_R_MAX_DEG);
    RobotArm_SetLimit(ARM_GRIPPER, 0, 180);

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        RobotArm_SetSpeed(i, JOINT_SPEED_DPS);
        RobotArm_SetAccel(i, JOINT_ACCEL_DPS2);
    }
    RobotArm_SetSpeed(ARM_GRIPPER, GRIP_SPEED_DPS);
    RobotArm_SetAccel(ARM_GRIPPER, GRIP_ACCEL_DPS2);
    RobotArm_SetTarget(ARM_GRIPPER, GRIP_CLOSE);
    s_grip_target = GRIP_CLOSE;

    /* RobotArm_Init() 已经线性插值把六轴送回中位 1500us 并确认到位，
     * 这里再兜一次底（到不了也不会卡死），然后才开始传感器标定：
     * 之后体感控制的所有角度都是相对这个中位算的偏差 */
    RobotArm_WaitReached(500);

    /* 2. 传感器 */
    if (MPU6050_Init() != 0)
        return 1;

    /* 3. 把当前姿势记为“中位” */
    MPU6050_CaptureNeutral();
    return 0;
}

/** 体感采样一步（非阻塞，由主循环按 50Hz 调用） 只做“读姿态 → 映射关节 → 写目标角度”，PWM 插值交给
 * armTask 单独跑， */
void MPU6050_GestureStep(void) {
    static uint32_t s_last_ms = 0;

    uint32_t t0 = HAL_GetTick();
    float dt;
    float pitch, roll, yaw;
    MPU6050_t d = {0};

    /* ---- 1. 计算本次与上次的间隔，作为陀螺积分步长 ---- */
    if (s_last_ms == 0U) {
        dt = (float)GESTURE_PERIOD_MS / 1000.0f;
    } else {
        dt = (float)(t0 - s_last_ms) / 1000.0f;
    }
    s_last_ms = t0;
    if (dt < 0.002f)
        dt = 0.002f;
    if (dt > 0.100f)
        dt = 0.100f;

    /* ---- 1.2 升级期间冻结体感 ----
     * 写/擦 Flash 会把 CPU 停住几百毫秒，这时手臂跟着 IMU 乱动既危险也没意义。
     * 直接不动目标角度，六轴停在当前位置；升级结束后自动恢复。 */
    if (IAP_Active() != 0U) {
        return;
    }

    /* ---- 1.5 网络远程模式：不接受体感输入 ----
     * 目标角度由 NetLink 从串口下发（ESP8266/MQTT），这里不再覆盖它；
     * PWM 插值由 armTask 独立完成，与本函数无关 */
    if (NetLink_GetMode() != NET_MODE_GESTURE) {
        return;
    }

    /* ---- 2. 采样 + 解算 ---- */
    if (MPU6050_ReadRaw(&d)) {
        MPU6050_UpdateAttitude(&d, dt);

        pitch = d.pitch - s_ref_pitch;
        roll = d.roll - s_ref_roll;
        yaw = s_yaw;

        /* 死区：小于 3° 的细微抖动直接当 0，避免机械臂“抽筋” */
        if (fabsf(pitch) < GESTURE_DEADBAND)
            pitch = 0.0f;
        if (fabsf(roll) < GESTURE_DEADBAND)
            roll = 0.0f;
        if (fabsf(yaw) < GESTURE_DEADBAND)
            yaw = 0.0f;

        /* ---- 3. 映射到 6 个关节（先整组算出来，再统一写目标） ---- */
        uint8_t tgt[ROBOTARM_AXIS_NUM];

        tgt[ARM_BASE] = Map_Joint(roll, BASE_GAIN, BASE_MIN_DEG, BASE_MAX_DEG);
        tgt[ARM_SHOULDER] = Map_Joint(pitch, SHOULDER_GAIN, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
        tgt[ARM_ELBOW] = Map_Joint(pitch, ELBOW_GAIN, ELBOW_MIN_DEG, ELBOW_MAX_DEG);
        tgt[ARM_WRIST_PITCH] = Map_Joint(pitch, WRIST_PITCH_GAIN, WRIST_P_MIN_DEG, WRIST_P_MAX_DEG);
        tgt[ARM_WRIST_ROLL] = Map_Joint(yaw, WRIST_ROLL_GAIN, WRIST_R_MIN_DEG, WRIST_R_MAX_DEG);
        tgt[ARM_GRIPPER] = s_grip_target;

#if SHAKE_ENABLE
        /* 摇一摇：必须先“六轴都到位且目标 ≥SHAKE_IDLE_MS 没变化”才允许识别，
           否则机械臂自身的运动/舵机抖动会把加速度顶过门槛，夹爪就会一直开合 */
        static uint8_t s_prev[ROBOTARM_AXIS_NUM] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        static uint32_t s_idle_ms = 0;
        uint8_t i;
        uint8_t same = 1U;

        for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
            if (tgt[i] != s_prev[i])
                same = 0U;
            s_prev[i] = tgt[i];
        }
        if ((same == 0U) || (RobotArm_TargetReached() == 0U)) {
            s_idle_ms = t0;
        } else if ((s_idle_ms != 0U) && ((uint32_t)(t0 - s_idle_ms) >= SHAKE_IDLE_MS)) {
            if (Shake_Detect(d.dyn_g)) {
                s_grip_target =
                    (s_grip_target == GRIP_OPEN) ? (uint8_t)GRIP_CLOSE : (uint8_t)GRIP_OPEN;
                tgt[ARM_GRIPPER] = s_grip_target;
            }
        }
#endif

        RobotArm_SetTargets(tgt);
    }
    /* 读取失败（接线松动/干扰）时不做任何改动，机械臂保持上一时刻姿态 */

    /* ---- 5. PWM 输出由 armTask 每 20ms 调 RobotArm_Update() 完成 ----
     * 本函数只决定“这一拍往哪走”，不阻塞、不加延时 */
}
