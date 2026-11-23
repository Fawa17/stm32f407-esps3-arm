/** MPU6050.h - 六轴传感器 + 体感控制 接线（GY-521
 * 模块）：VCC 3.3V，GND，SCL=PB10，SDA=PB11，AD0=GND(地址 0x68) */
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================== 硬件配置 ===================== */
/* 底层是 CubeMX 生成的硬件 I2C2（PB10=SCL、PB11=SDA、100kHz），
 * 引脚/速率都在 Core/Src/i2c.c 的 MX_I2C2_Init() / HAL_I2C_MspInit() 里改 */
#define MPU6050_ADDRESS 0xD0 /* 8 位写地址（0x68 << 1），HAL 直接这么传 */

/* ===================== 寄存器地址 ===================== */
#define MPU_SAMPLE_RATE_REG 0x19
#define MPU_CFG_REG 0x1A
#define MPU_GYRO_CFG_REG 0x1B
#define MPU_ACCEL_CFG_REG 0x1C
#define MPU_FIFO_EN_REG 0x23
#define MPU_INTBP_CFG_REG 0x37
#define MPU_INT_EN_REG 0x38
#define MPU_INT_STA_REG 0x3A
#define MPU_ACCEL_XOUTH_REG 0x3B
#define MPU_TEMP_OUTH_REG 0x41
#define MPU_GYRO_XOUTH_REG 0x43
#define MPU_USER_CTRL_REG 0x6A
#define MPU_PWR_MGMT1_REG 0x6B
#define MPU_PWR_MGMT2_REG 0x6C
#define MPU_DEVICE_ID_REG 0x75

/* ===================== 夹爪预设角度 ===================== */
/** 夹爪两个端点务必留出"机械余量"： 舵机是位置伺服，一旦顶到机械限位就会持续堵转 —— 表现为嗡嗡响 +
 * 发烫 + 电流大， */
#define GRIP_OPEN 30   /* 张开（若装反了就交换这两个值） */
#define GRIP_CLOSE 118 /* 闭合：留约 17° 余量，避免顶到限位堵转 */

/* ===================== 姿态数据结构 ===================== */
typedef struct {
    int16_t ax, ay, az; /* 加速度原始值，单位 LSB（±2g）   */
    int16_t gx, gy, gz; /* 陀螺仪原始值，单位 LSB（±2000dps）*/
    float temp_c;       /* 芯片温度，°C                     */
    float pitch;        /* 俯仰角，°（抬头为正）            */
    float roll;         /* 横滚角，°（右倾为正）            */
    float yaw;          /* 航向角，°（陀螺积分，会缓慢回零）*/
    float dyn_g;        /* 动态加速度，单位 g，用于摇一摇判定*/
    uint8_t ok;         /* 1 = 本次读取有效                 */
} MPU6050_t;

/* ===================== 公共 API ===================== */

/** 初始化 GPIO/软件 I2C/传感器寄存器 + 陀螺零偏校准。返回 0 表示成功 */
uint8_t MPU6050_Init(void);

/** 读 WHO_AM_I，正常返回 0x68 */
uint8_t MPU6050_ReadID(void);

/** 陀螺仪零偏校准（传感器必须静止），samples 次采样 */
void MPU6050_CalibrateGyro(uint16_t samples);

/** 读取一次原始数据（加速度 + 陀螺仪 + 温度，一次突发读取）。返回 1 成功 */
uint8_t MPU6050_ReadRaw(MPU6050_t *data);

/** 互补滤波更新姿态，dt 单位秒 */
void MPU6050_UpdateAttitude(MPU6050_t *data, float dt);

/** 读取当前绝对姿态角（°） */
void MPU6050_GetAttitude(float *pitch, float *roll, float *yaw);

/** 读取相对“中位基准”的偏差角（°），体感控制实际用的是这个 */
void MPU6050_GetRelativeAttitude(float *pitch, float *roll, float *yaw);

/** 只取缓存姿态（不访问 I2C，供 OLED 等其它任务显示用） */
void MPU6050_GetCachedAttitude(float *pitch, float *roll, float *yaw);

/** 把当前姿态记为机械臂 90° 中位的基准（上电时自动调用一次） */
void MPU6050_CaptureNeutral(void);

/** 航向角清零 */
void MPU6050_ResetYaw(void);

/** 一次完成：舵机初始化 + 传感器初始化 + 中位标定 + 夹爪张开。返回 0 成功 */
uint8_t MPU6050_GestureInit(void);

/** 体感采样一步：读姿态 → 映射 6 轴 → 写目标角度（非阻塞） 由 gestureTask 每
 * 20ms（50Hz）调用一次；PWM 插值在 armTask 里做。 远程模式（NetLink 切到
 * NET_MODE_REMOTE）时本函数直接返回，不覆盖目标。 */
void MPU6050_GestureStep(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
