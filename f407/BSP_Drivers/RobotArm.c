/*
 * RobotArm.c - 六轴舵机 PWM + 轨迹插值
 *
 * 关节角用 float 算（F407 有 FPU，FreeRTOS 用的是 ARM_CM4F 端口），
 * 整数刻度做不到低速连续变化，会一格一格跳。
 * 浮点只决定"命令"多细，真实角度靠 SetCalib 量出来的两端点。
 *
 * 引脚：0=PA0(TIM2_CH1) 1=PA1 2=PA2 3=PA3 4=PA6(TIM3_CH1) 5=PA7(TIM3_CH2)
 * 时基：84MHz/(PSC+1)=2MHz，0.5us 一格，Period=39999 -> 20ms/50Hz
 */
#include "RobotArm.h"
#include "tim.h"
#include <math.h>
#include <string.h>
#include "cmsis_os.h"

/* ===================== 互斥量 ===================== */
static osMutexId_t s_lock = NULL;

void RobotArm_CreateLock(void) {
    s_lock = osMutexNew(NULL);
}

static void Lock(void) {
    if (s_lock != NULL) {
        (void)osMutexAcquire(s_lock, osWaitForever);
    }
}

static void Unlock(void) {
    if (s_lock != NULL) {
        (void)osMutexRelease(s_lock);
    }
}

/* ===================== 通道表 ===================== */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} ServoCh_t;

/* 轴 3（腕部俯仰）默认走 PA3 = TIM2_CH4。
 * 如果 PA3 这个引脚坏了（舵机不响应、示波器/万用表看不到脉冲），
 * 把下面这行改成 1，轴 3 就自动改走 **PB1 = TIM3_CH4**（这个通道现在空着），
 * 只需把舵机信号线从 PA3 挪到 PB1，其它代码和接线都不用动。 */
#define AXIS3_ON_PB1 0

static const ServoCh_t s_ch[ROBOTARM_AXIS_NUM] = {
    {&htim2, TIM_CHANNEL_1}, /* 0 底座旋转   PA0 */
    {&htim2, TIM_CHANNEL_2}, /* 1 大臂俯仰   PA1 */
    {&htim2, TIM_CHANNEL_3}, /* 2 小臂俯仰   PA2 */
#if AXIS3_ON_PB1
    {&htim3, TIM_CHANNEL_4}, /* 3 腕部俯仰   PB1（PA3 损坏时的备用） */
#else
    {&htim2, TIM_CHANNEL_4}, /* 3 腕部俯仰   PA3 */
#endif
    {&htim3, TIM_CHANNEL_1}, /* 4 腕部旋转   PA6 */
    {&htim3, TIM_CHANNEL_2}, /* 5 夹爪开合   PA7 */
};

#if AXIS3_ON_PB1
/* 把 PB1 配成 TIM3_CH4 的 PWM 输出。CubeMX 里没配 CH4，所以在这里补；
 * 不写进 tim.c 是为了 CubeMX 重新生成时不会被冲掉。 */
static void Axis3_RemapToPB1(void) {
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = (SERVO_MIN_TICKS + SERVO_MAX_TICKS) / 2U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    (void)HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_4);
}
#endif

/* ===================== 状态（全部浮点，单位：度 / 度每秒 / 度每秒²） ===================== */
static float s_pos[ROBOTARM_AXIS_NUM] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
static float s_target[ROBOTARM_AXIS_NUM] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
static float s_vel[ROBOTARM_AXIS_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float s_acc[ROBOTARM_AXIS_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static uint8_t s_min_deg[ROBOTARM_AXIS_NUM] = {0, 0, 0, 0, 0, 0};
static uint8_t s_max_deg[ROBOTARM_AXIS_NUM] = {180, 180, 180, 180, 180, 180};
static uint8_t s_reverse[ROBOTARM_AXIS_NUM] = {0, 0, 0, 0, 0, 0};
static uint16_t s_speed[ROBOTARM_AXIS_NUM] = {90, 90, 90, 90, 90, 90}; /* 度/秒 */
static uint16_t s_accel[ROBOTARM_AXIS_NUM] = {ROBOTARM_ACCEL_DPS2, ROBOTARM_ACCEL_DPS2,
                                              ROBOTARM_ACCEL_DPS2, ROBOTARM_ACCEL_DPS2,
                                              ROBOTARM_ACCEL_DPS2, ROBOTARM_ACCEL_DPS2};

/* 逐轴两点标定：命令 0°/180° 对应的 CCR 计数（0.5µs 单位） */
static uint32_t s_tmin[ROBOTARM_AXIS_NUM] = {SERVO_MIN_TICKS, SERVO_MIN_TICKS, SERVO_MIN_TICKS,
                                             SERVO_MIN_TICKS, SERVO_MIN_TICKS, SERVO_MIN_TICKS};
static uint32_t s_tmax[ROBOTARM_AXIS_NUM] = {SERVO_MAX_TICKS, SERVO_MAX_TICKS, SERVO_MAX_TICKS,
                                             SERVO_MAX_TICKS, SERVO_MAX_TICKS, SERVO_MAX_TICKS};

/* 上次写进 CCR 的计数 + 有没有写过，配合下面的死区判断用 */
static uint16_t s_ccr_last[ROBOTARM_AXIS_NUM];
static uint8_t s_ccr_valid[ROBOTARM_AXIS_NUM];

static uint8_t s_started = 0;
static uint8_t s_moving = 0;

/* 上电起始姿势（默认全 90） */
static uint8_t s_boot[ROBOTARM_AXIS_NUM] = {90, 90, 90, 90, 90, 90};

/* ===================== 内部工具 ===================== */
static uint8_t ClampDeg(uint8_t axis, int16_t deg) {
    if (deg < (int16_t)s_min_deg[axis])
        deg = (int16_t)s_min_deg[axis];
    if (deg > (int16_t)s_max_deg[axis])
        deg = (int16_t)s_max_deg[axis];
    return (uint8_t)deg;
}

/* 角度(度) -> CCR 计数。这里是整条链路唯一的量化点 */
static void Servo_Write_deg(uint8_t axis, float deg) {
    float span;
    float t;

    if (deg < 0.0f)
        deg = 0.0f;
    if (deg > 180.0f)
        deg = 180.0f;
    if (s_reverse[axis])
        deg = 180.0f - deg;

    span = (float)s_tmax[axis] - (float)s_tmin[axis];
    t = (float)s_tmin[axis] + span * deg * (1.0f / 180.0f);

    /* 硬钳位：标定值写坏了（tmax=40000 之类）也不能让 CCR>=ARR 输出恒高，
       否则舵机顶到机械限位堵转发热。合法窗口 0.5~2.5ms。 */
    if (t < (float)SERVO_MIN_TICKS) {
        t = (float)SERVO_MIN_TICKS;
    }
    if (t > (float)SERVO_MAX_TICKS) {
        t = (float)SERVO_MAX_TICKS;
    }

    /* 死区：变化小于 SERVO_WRITE_DEADBAND_TICKS 就不再写。
     * 以前 100Hz 往同一个目标反复写 ±1 格，舵机在死区边缘一直纠偏，就是嗡嗡响的来源。 */
    uint32_t ticks = (uint32_t)(t + 0.5f);
    uint32_t last = (uint32_t)s_ccr_last[axis];
    uint32_t diff = (ticks > last) ? (ticks - last) : (last - ticks);

    if ((s_ccr_valid[axis] != 0U) && (diff < SERVO_WRITE_DEADBAND_TICKS)) {
        return;
    }
    s_ccr_last[axis] = (uint16_t)ticks;
    s_ccr_valid[axis] = 1U;
    __HAL_TIM_SET_COMPARE(s_ch[axis].htim, s_ch[axis].channel, ticks);
}

static void Servo_Write(uint8_t axis, uint8_t angle) {
    Servo_Write_deg(axis, (float)angle);
}

static void Output_Pos(uint8_t axis) {
    Servo_Write_deg(axis, s_pos[axis]);
}

/* ===================== 公共 API ===================== */

/* 启动 PWM：一路一路开，每路隔 START_GAP_MS 就是为了压冲击电流 */
void RobotArm_StartPWM(void) {
    uint8_t i;

    if (s_started)
        return;

#if AXIS3_ON_PB1
    Axis3_RemapToPB1(); /* 轴 3 改走 PB1：先把 PB1 配成 TIM3_CH4 的 PWM */
#endif

    /* 以宏为准设置预分频/周期：CubeMX 里若还是 83/19999（1µs 一格），
       会被改成 SERVO_PSC/SERVO_ARR（41/39999 = 0.5µs 一格）。
       PSC/ARR/CCR 都有预装载，必须靠 EGR.UG 一次性锁进活动寄存器；
       UG 的位置在下面"6 路 CCR 都写完"之后（写在这里会把旧 CCR 锁进去）。 */
    htim2.Init.Prescaler = SERVO_PSC;
    htim2.Init.Period = SERVO_ARR;
    __HAL_TIM_SET_PRESCALER(&htim2, SERVO_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim2, SERVO_ARR);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);

    htim3.Init.Prescaler = SERVO_PSC;
    htim3.Init.Period = SERVO_ARR;
    __HAL_TIM_SET_PRESCALER(&htim3, SERVO_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_ARR);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        s_pos[i] = (float)s_boot[i];
        s_target[i] = s_pos[i];
        s_vel[i] = 0.0f;
        s_acc[i] = 0.0f;
        Servo_Write(i, s_boot[i]); /* 先给“起始姿势”的脉宽，再开通道 */
    }

    /* ⚠️ 顺序要点：CCR 也是有预装载的。必须等 6 路 CCR 都写进影子寄存器之后，
       再用 EGR.UG 把 PSC/ARR/CCR 一次性同时锁进活动寄存器。
       若 UG 放在写 CCR 之前，UG 会把 CubeMX 里旧的 CCR(1500 tick @1MHz=1500µs)
       锁进活动寄存器，而我们的 3000 只能等下一个 UEV 才生效 ——
       表现为每个定时器的 CH1（PA0、PA6）上电首帧是 750µs。 */
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE); /* UG 会置 UIF，清掉 */
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        HAL_TIM_PWM_Start(s_ch[i].htim, s_ch[i].channel);
        HAL_Delay(ROBOTARM_START_GAP_MS);
    }

    s_started = 1;

    /* 首帧已在上面 s_boot 那批写掉了，这里不再重复写一遍 —— 六路同一毫秒起动
     * 会把冲击电流叠加起来。 */
}

/* 软启动 + 固定时长线性回中位；本来就在中位就直接返回 */
void RobotArm_Init(void) {
    float from[ROBOTARM_AXIS_NUM];
    uint16_t steps;
    uint16_t s;
    uint8_t i;
    uint8_t need = 0;
    const float neutral = (float)ROBOTARM_NEUTRAL;

    RobotArm_StartPWM();

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        if (fabsf(s_pos[i] - neutral) > 0.001f)
            need = 1U;
        s_vel[i] = 0.0f;
        s_acc[i] = 0.0f;
        s_target[i] = neutral;
    }
    if (!need)
        return; /* 已经在中位，不用再走一遍 */

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        from[i] = s_pos[i];
    }

    steps = (uint16_t)(ROBOTARM_CENTER_MS / ROBOTARM_TICK_MS);
    if (steps < 1U)
        steps = 1U;

    for (s = 1; s <= steps; s++) {
        float k = (float)s / (float)steps;
        for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
            s_pos[i] = from[i] + (neutral - from[i]) * k;
            Output_Pos(i);
        }
        HAL_Delay(ROBOTARM_TICK_MS);
    }

    /* 终点对齐，吃掉浮点误差 */
    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        s_pos[i] = neutral;
        s_target[i] = neutral;
        s_vel[i] = 0.0f;
        s_acc[i] = 0.0f;
        Output_Pos(i);
    }
}

/** 设置上电时的起始姿势（必须在 RobotArm_Init 之前调用才有效） */
void RobotArm_SetBootPose(const uint8_t angle[ROBOTARM_AXIS_NUM]) {
    uint8_t i;

    if (s_started)
        return;
    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        s_boot[i] = ClampDeg(i, (int16_t)angle[i]);
    }
}

void RobotArm_SetLimit(uint8_t axis, uint8_t min_deg, uint8_t max_deg) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    if (min_deg > max_deg)
        return;

    s_min_deg[axis] = min_deg;
    s_max_deg[axis] = max_deg;

    if (s_target[axis] > (float)s_max_deg[axis])
        s_target[axis] = (float)s_max_deg[axis];
    if (s_target[axis] < (float)s_min_deg[axis])
        s_target[axis] = (float)s_min_deg[axis];
}

void RobotArm_SetReverse(uint8_t axis, uint8_t reverse) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    s_reverse[axis] = reverse ? 1U : 0U;
}

void RobotArm_SetSpeed(uint8_t axis, uint16_t deg_per_sec) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    if (deg_per_sec < 1U)
        deg_per_sec = 1U;
    s_speed[axis] = deg_per_sec;
}

void RobotArm_SetAccel(uint8_t axis, uint16_t deg_per_sec2) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    if (deg_per_sec2 < 1U)
        deg_per_sec2 = 1U;
    s_accel[axis] = deg_per_sec2;
}

void RobotArm_SetCalib(uint8_t axis, uint32_t min_ticks, uint32_t max_ticks) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    if (min_ticks < 100U || max_ticks > 40000U)
        return; /* 0.5µs 单位下的合理区间 */
    if (max_ticks < min_ticks + 400U)
        return; /* 至少 200µs 行程 */

    Lock();
    s_tmin[axis] = min_ticks;
    s_tmax[axis] = max_ticks;
    Servo_Write_deg(axis, s_pos[axis]);
    Unlock();
}

void RobotArm_GetCalib(uint8_t axis, uint32_t *min_ticks, uint32_t *max_ticks) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    if (min_ticks)
        *min_ticks = s_tmin[axis];
    if (max_ticks)
        *max_ticks = s_tmax[axis];
}

uint16_t RobotArm_GetSpeed(uint8_t axis) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return 0U;
    return s_speed[axis];
}

uint16_t RobotArm_GetAccel(uint8_t axis) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return 0U;
    return s_accel[axis];
}

void RobotArm_SetTarget(uint8_t axis, uint8_t angle) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    Lock();
    s_target[axis] = (float)ClampDeg(axis, (int16_t)angle);
    Unlock();
}

void RobotArm_SetTargets(const uint8_t angle[ROBOTARM_AXIS_NUM]) {
    uint8_t i;

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        RobotArm_SetTarget(i, angle[i]);
    }
}

/* 带时长：按位移反算各轴速度，保证六轴同时到位 */
void RobotArm_SetTargetsTimed(const uint8_t angle[ROBOTARM_AXIS_NUM], uint16_t duration_ms) {
    uint8_t i;
    float dur_s;

    if (duration_ms < ROBOTARM_TICK_MS)
        duration_ms = ROBOTARM_TICK_MS;
    dur_s = (float)duration_ms * 0.001f;

    Lock();
    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        float tgt = (float)ClampDeg(i, (int16_t)angle[i]);
        float dist = fabsf(tgt - s_pos[i]);
        float sp = dist / dur_s; /* 度/秒，浮点不截断 */
        uint32_t spu;

        spu = (uint32_t)(sp + 0.999f); /* 向上取整到 度/秒 */
        if (spu == 0U)
            spu = 1U;
        if (spu > 65535U)
            spu = 65535U;

        s_speed[i] = (uint16_t)spu;
        s_target[i] = tgt;
    }
    Unlock();
}

/* 急停：目标设成当前位置 */
void RobotArm_Stop(void) {
    uint8_t i;

    Lock();
    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        s_target[i] = s_pos[i];
        s_vel[i] = 0.0f;
        s_acc[i] = 0.0f;
    }
    s_moving = 0;
    Unlock();
}

/**
 * S 曲线插值一步，每 TICK_MS 调一次。每轴：
 *   1) 剩余距离 < v^2/(2a) 就开始减速，否则冲过头
 *   2) 加速度每拍最多变 a/JERK_STEPS（jerk 受限）
 *   3) 速度积分成位置
 * 全 float，所以低速收尾是一格一格连续的。
 */
void RobotArm_Update(void) {
    const float dt = ROBOTARM_TICK_S;
    uint8_t i;
    uint8_t moving = 0;

    Lock();
    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        float dist, rem, vmax, a, v, acc, acc_tgt, da, stopd, step;
        float lo, hi, edge, zone;

        dist = s_target[i] - s_pos[i];
        if (fabsf(dist) < 1e-4f) {
            s_pos[i] = s_target[i];
            s_vel[i] = 0.0f;
            s_acc[i] = 0.0f;
            continue; /* 已到位 */
        }

        rem = fabsf(dist);
        vmax = (float)s_speed[i];
        if (vmax < 1.0f)
            vmax = 1.0f;

        /* ---- 边界降速：目标贴到限位时只给三成速度 ---- */
        lo = (float)s_min_deg[i];
        hi = (float)s_max_deg[i];
        zone = (float)ROBOTARM_SOFT_ZONE_DEG;
        edge = (s_target[i] > s_pos[i]) ? (hi - s_target[i]) : (s_target[i] - lo);
        if (edge < 0.0f)
            edge = 0.0f;
        if (edge < zone) {
            float pct =
                (float)ROBOTARM_SOFT_PCT + (100.0f - (float)ROBOTARM_SOFT_PCT) * edge / zone;
            vmax *= pct * 0.01f;
            if (vmax < 0.5f)
                vmax = 0.5f;
        }

        a = (float)s_accel[i];
        if (a < 1.0f)
            a = 1.0f;

        v = s_vel[i];
        acc = s_acc[i];

        /* 1) 刹车距离判定 */
        stopd = (v * v) / (2.0f * a);
        acc_tgt = (rem <= stopd) ? -a : a;

        /* 2) jerk 限幅：每拍加速度最多变化 a/JERK_STEPS */
        da = a / (float)ROBOTARM_JERK_STEPS;
        if (acc < acc_tgt) {
            acc += da;
            if (acc > acc_tgt)
                acc = acc_tgt;
        } else if (acc > acc_tgt) {
            acc -= da;
            if (acc < acc_tgt)
                acc = acc_tgt;
        }

        /* 3) 积分 */
        v += acc * dt;
        if (v > vmax)
            v = vmax;
        if (v < 0.0f)
            v = 0.0f;

        step = v * dt;
        if (step >= rem) { /* 这一拍就能到：直接落位，不留抖动尾巴 */
            s_pos[i] = s_target[i];
            s_vel[i] = 0.0f;
            s_acc[i] = 0.0f;
            v = 0.0f;
        } else {
            s_pos[i] += (dist > 0.0f) ? step : -step;
            s_vel[i] = v;
            s_acc[i] = acc;
        }

        Output_Pos(i);
        if (fabsf(s_target[i] - s_pos[i]) > 1e-4f)
            moving = 1;
    }

    s_moving = moving;
    Unlock();
}

uint8_t RobotArm_TargetReached(void) {
    return (s_moving == 0U) ? 1U : 0U;
}

/* 等六轴到位，超时返回 0 */
uint8_t RobotArm_WaitReached(uint16_t timeout_ms) {
    uint16_t waited = 0;

    for (;;) {
        RobotArm_Update();
        if (RobotArm_TargetReached())
            return 1U;

        if (waited >= timeout_ms)
            return 0U;
        HAL_Delay(ROBOTARM_TICK_MS);
        waited = (uint16_t)(waited + ROBOTARM_TICK_MS);
    }
}

uint8_t RobotArm_GetAngle(uint8_t axis) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return 0;
    return (uint8_t)(s_pos[axis] + 0.5f);
}

/* 与 Servo_Write_deg 完全同一套换算，只是把结果换算成微秒返回：
 * 用来在 OLED 上显示"实际下发的脉宽"，一眼判断是固件没给 1500，还是硬件没收到。 */
uint16_t RobotArm_GetPulseUs(uint8_t axis) {
    float span;
    float t;
    float deg;

    if (axis >= ROBOTARM_AXIS_NUM)
        return 0U;

    deg = s_pos[axis];
    if (deg < 0.0f)
        deg = 0.0f;
    if (deg > 180.0f)
        deg = 180.0f;
    if (s_reverse[axis])
        deg = 180.0f - deg;

    span = (float)s_tmax[axis] - (float)s_tmin[axis];
    t = (float)s_tmin[axis] + span * deg * (1.0f / 180.0f);

    return (uint16_t)((t * (float)SERVO_TICK_NS) / 1000.0f + 0.5f);
}

float RobotArm_GetAngleF(uint8_t axis) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return 0.0f;
    return s_pos[axis];
}

/* 浮点限位夹取 */
static float ClampDegF(uint8_t axis, float deg) {
    if (deg < (float)s_min_deg[axis])
        deg = (float)s_min_deg[axis];
    if (deg > (float)s_max_deg[axis])
        deg = (float)s_max_deg[axis];
    return deg;
}

void RobotArm_SetTargetF(uint8_t axis, float deg) {
    if (axis >= ROBOTARM_AXIS_NUM)
        return;
    Lock();
    s_target[axis] = ClampDegF(axis, deg);
    Unlock();
}

void RobotArm_SetTargetsF(const float deg[ROBOTARM_AXIS_NUM]) {
    uint8_t i;

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        RobotArm_SetTargetF(i, deg[i]);
    }
}

/* 浮点版带时长（0.1° 级插补用，不先取整到度） */
void RobotArm_SetTargetsTimedF(const float deg[ROBOTARM_AXIS_NUM], uint16_t duration_ms) {
    uint8_t i;
    float dur_s;

    if (duration_ms < ROBOTARM_TICK_MS)
        duration_ms = ROBOTARM_TICK_MS;
    dur_s = (float)duration_ms * 0.001f;

    Lock();
    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        float tgt = ClampDegF(i, deg[i]);
        float dist = fabsf(tgt - s_pos[i]);
        float sp = dist / dur_s; /* 度/秒 */
        uint32_t spu;

        spu = (uint32_t)(sp + 0.999f); /* 向上取整到 度/秒 */
        if (spu == 0U)
            spu = 1U;
        if (spu > 65535U)
            spu = 65535U;

        s_speed[i] = (uint16_t)spu;
        s_target[i] = tgt;
    }
    Unlock();
}

void RobotArm_MoveSync(const uint8_t angle[ROBOTARM_AXIS_NUM], uint16_t duration_ms) {
    float from[ROBOTARM_AXIS_NUM];
    float to[ROBOTARM_AXIS_NUM];
    uint16_t steps;
    uint16_t s;
    uint8_t i;

    if (duration_ms < ROBOTARM_TICK_MS)
        duration_ms = ROBOTARM_TICK_MS;
    steps = (uint16_t)(duration_ms / ROBOTARM_TICK_MS);
    if (steps < 1U)
        steps = 1U;

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        to[i] = (float)ClampDeg(i, (int16_t)angle[i]);
        from[i] = s_pos[i];
        s_vel[i] = 0.0f;
        s_acc[i] = 0.0f;
    }

    for (s = 1; s <= steps; s++) {
        float k = (float)s / (float)steps;
        for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
            s_pos[i] = from[i] + (to[i] - from[i]) * k;
            Output_Pos(i);
        }
        HAL_Delay(ROBOTARM_TICK_MS);
    }

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        s_pos[i] = to[i];
        s_target[i] = to[i];
        s_vel[i] = 0.0f;
        s_acc[i] = 0.0f;
        Output_Pos(i);
    }
}
