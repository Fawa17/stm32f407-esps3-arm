/** RobotArm.h - 六轴舵机驱动（角度 -> 脉宽，带限位和平滑插值） 轴 | 关节 | 通道 | 引脚 */
#ifndef __ROBOTARM_H
#define __ROBOTARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===================== 轴号定义 ===================== */
#define ARM_BASE 0        /* 底座旋转   */
#define ARM_SHOULDER 1    /* 大臂俯仰   */
#define ARM_ELBOW 2       /* 小臂俯仰   */
#define ARM_WRIST_PITCH 3 /* 腕部俯仰   */
#define ARM_WRIST_ROLL 4  /* 腕部旋转   */
#define ARM_GRIPPER 5     /* 夹爪开合   */

#define ROBOTARM_AXIS_NUM 6

/* PWM 周期。模拟舵机按 50Hz 设计，改成 5000(200Hz) 只有数字舵机能吃，
 * 模拟舵机会嗡嗡响/发热，少数直接丢帧。确认是数字舵机再改。 */
#define SERVO_PERIOD_US 20000U

#if (SERVO_PERIOD_US <= 6000U)
#define ROBOTARM_TICK_MS 5 /* 200Hz：数字舵机可选 */
#else
#define ROBOTARM_TICK_MS 10 /* 100Hz 插值：与 50Hz PWM 匹配（越小越顺滑） */
#endif
#define ROBOTARM_TICK_S (ROBOTARM_TICK_MS / 1000.0f) /* 同一周期，单位秒（浮点核心用） */
#define ROBOTARM_NEUTRAL 90                          /* 中位角度 */
#define ROBOTARM_MIN_ANGLE 0
#define ROBOTARM_MAX_ANGLE 180
#define SERVO_MIN_US 500  /* 0° 对应脉宽 */
#define SERVO_MAX_US 2500 /* 180° 对应脉宽 */

/* 每格多少纳秒。100/0.1us 只有在 5ms 周期下才放得进 16 位 ARR
 * （20ms/0.1us = 200000 > 65535），所以默认 500。 */
#define SERVO_TICK_NS 500
#define SERVO_TIM_CLK_HZ 84000000UL
#define SERVO_TICKS_PER_US (1000UL / SERVO_TICK_NS)
#define SERVO_PSC ((SERVO_TIM_CLK_HZ / (1000000000UL / SERVO_TICK_NS)) - 1UL)
#define SERVO_ARR ((SERVO_PERIOD_US * 1000UL / SERVO_TICK_NS) - 1UL)
#define SERVO_MIN_TICKS (SERVO_MIN_US * SERVO_TICKS_PER_US)
#define SERVO_MAX_TICKS (SERVO_MAX_US * SERVO_TICKS_PER_US)

/* 变化小于这个就不写 CCR。以前每拍都写 ±1 格的抖动值，舵机会一直纠偏 -> 嗡嗡响。
 * 4 格 = 2us ≈ 0.18 度，比舵机自身死区小，不影响定位。 */
#define SERVO_WRITE_DEADBAND_TICKS 4U

/* 16 位 ARR 上限检查：超了编译器直接报错，免得现场才发现脉冲周期不对 */
#if (SERVO_ARR > 65535UL)
#error "SERVO_PERIOD_US/SERVO_TICK_NS 超过 16 位 ARR：请加大 SERVO_TICK_NS 或减小 SERVO_PERIOD_US"
#endif

#define ROBOTARM_START_GAP_MS                                                                      \
    200 /* 上电时每路舵机的使能间隔：分时上电，避免 6 路同时启动的冲击电流 */
#define ROBOTARM_ACCEL_DPS2 400   /* 默认加速度（度/秒²）：越小越柔和，电流冲击也越小 */
#define ROBOTARM_CENTER_MS 1500   /* 上电回中位的总时长（线性插值，六轴同时到位） */
#define ROBOTARM_JERK_STEPS 6     /* S 曲线：加速度从 0 爬到最大用几拍（越大越柔，起停冲击越小） */
#define ROBOTARM_SOFT_ZONE_DEG 12 /* 边界降速：目标角离限位小于这个度数时开始限速 */
#define ROBOTARM_SOFT_PCT 30      /* 贴到限位时最多只给额定的百分比（30 = 三成速度） */

/* ===================== 公共 API ===================== */

/** 建互斥量，必须在 osKernelStart 之前（MX_FREERTOS_Init 里）调 */
void RobotArm_CreateLock(void);

/** 初始化：逐路使能 PWM，再在 ROBOTARM_CENTER_MS 内线性回中位（阻塞）。 已经在中位就直接返回。 */
void RobotArm_Init(void);

/** 仅启动 PWM（逐路使能，首次调用时带软启动间隔；不改变目标角度） */
void RobotArm_StartPWM(void);

/**
 * 填上电时机械臂实际停的姿势（必须在 Init 之前调），上电就不会跳。
 */
void RobotArm_SetBootPose(const uint8_t angle[ROBOTARM_AXIS_NUM]);

/** 设置某个关节的软件限位（单位：度），用于防止结构干涉/堵转 */
void RobotArm_SetLimit(uint8_t axis, uint8_t min_deg, uint8_t max_deg);

/** 设置某个关节的安装方向：1 = 反向（角度 a 实际输出 180-a） */
void RobotArm_SetReverse(uint8_t axis, uint8_t reverse);

/** 设置某个关节的最大速度（度/秒），默认 90。越小越柔和 */
void RobotArm_SetSpeed(uint8_t axis, uint16_t deg_per_sec);

/** 设置某个关节的加速度（度/秒²），默认 ROBOTARM_ACCEL_DPS2。越小越柔和 */
void RobotArm_SetAccel(uint8_t axis, uint16_t deg_per_sec2);

/** 两点标定：给出该关节 0° 和 180° 对应的 CCR 计数（0.5us 单位，1000/5000 = 500/2500us）。
 * 每个舵机实际行程都不一样，量出来比提高分辨率有用。 */
void RobotArm_SetCalib(uint8_t axis, uint32_t min_ticks, uint32_t max_ticks);

/** 读回某个关节的标定值（单位：0.5µs 计数） */
void RobotArm_GetCalib(uint8_t axis, uint32_t *min_ticks, uint32_t *max_ticks);

/** 读回速度/加速度（度/秒、度/秒²），用于持久化 */
uint16_t RobotArm_GetSpeed(uint8_t axis);
uint16_t RobotArm_GetAccel(uint8_t axis);

/** 设置单个/全部关节的目标角度（立即返回，由 RobotArm_Update 平滑逼近） */
void RobotArm_SetTarget(uint8_t axis, uint8_t angle);
void RobotArm_SetTargets(const uint8_t angle[ROBOTARM_AXIS_NUM]);

/**
 * 带时长：按"距离/时间"反算各轴速度，保证同时到位。
 */
void RobotArm_SetTargetsTimed(const uint8_t angle[ROBOTARM_AXIS_NUM], uint16_t duration_ms);

/* ---- 浮点版：接小数的目标（串口 P 命令带小数时走这里）---- */
void RobotArm_SetTargetF(uint8_t axis, float deg);
void RobotArm_SetTargetsF(const float deg[ROBOTARM_AXIS_NUM]);
void RobotArm_SetTargetsTimedF(const float deg[ROBOTARM_AXIS_NUM], uint16_t duration_ms);
float RobotArm_GetAngleF(uint8_t axis);

/** 立即停止：目标设成当前位置，速度清零（保持姿态，不卸力） */
void RobotArm_Stop(void);

/** 插值一步，每 ROBOTARM_TICK_MS(10ms) 调一次 */
void RobotArm_Update(void);

/** 6 个关节是否都到达目标位置 */
uint8_t RobotArm_TargetReached(void);

/** 阻塞等六轴到位；1=到了 0=超时 */
uint8_t RobotArm_WaitReached(uint16_t timeout_ms);

/** 读取关节当前（插值中的）角度 */
uint8_t RobotArm_GetAngle(uint8_t axis);
/* 读当前下发的脉宽(us)，给 OLED 看看到底有没有真的给 1500 */
uint16_t RobotArm_GetPulseUs(uint8_t axis);

/** 阻塞式动作组：duration_ms 内六轴一起走到目标 */
void RobotArm_MoveSync(const uint8_t angle[ROBOTARM_AXIS_NUM], uint16_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOTARM_H */
