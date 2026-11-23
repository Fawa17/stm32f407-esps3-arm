/** @file Watchdog.h 独立看门狗 + 任务心跳监视 */
#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 调试开关：0 = 完全不启用（所有函数变空操作） */
#define WATCHDOG_ENABLE 1

/* 任务编号（与 freertos.c 里的报到调用一一对应） */
#define WD_TASK_ARM 0U
#define WD_TASK_GESTURE 1U
#define WD_TASK_NET 2U
#define WD_TASK_OLED 3U
#define WD_TASK_NUM 4U

/**
 * @brief 启动 IWDG（约 3s 超时）。重复调用无副作用，只有第一次生效
 */
void Watchdog_Start(void);

/** 某任务报到（每跑完一圈调用一次） */
void Watchdog_KeepAlive(uint8_t id);

/** 四个任务是否都在期限内报到过 */
uint8_t Watchdog_AllAlive(void);

/**
 * @brief 喂狗（IWDG->KR = 0xAAAA）
 */
void Watchdog_Feed(void);

/**
 * @brief 当前各任务的“超期”位图，bit=1 表示该任务超期（调试/上屏用）
 */
uint16_t Watchdog_OverdueMask(void);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H */
