/** @file OLED.h SSD1306 128x64 OLED 状态监视器驱动（硬件 I2C1，PB6=SCL / PB7=SDA） */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* SSD1306 的 7 位地址是 0x3C，HAL 要左移一位 */
#define OLED_I2C_ADDR 0x78U
#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U
#define OLED_LINE_NUM 8U /* 8 行，每行 8px */
#define OLED_COL_NUM 21U /* 6px 一个字，128/6 = 21 */

/** 初始化 SSD1306（复用 CubeMX 的 hi2c1） */
uint8_t OLED_Init(void);

/**
 * @brief 屏幕是否可用（Init 成功且后续刷新没连续失败）
 */
uint8_t OLED_IsReady(void);

/**
 * @brief 清显存（不立即上屏，需再调 OLED_Update）
 */
void OLED_Clear(void);

/**
 * @brief 把显存整帧推到屏幕（8 页 x 128 字节）
 */
void OLED_Update(void);

/**
 * @brief 在第 line 行显示格式化字符串（line: 0~7，最多 21 字符，自动截断）
 */
void OLED_Text(uint8_t line, const char *fmt, ...);

/**
 * @brief 渲染一帧“状态监视器”内容并上屏；任务里周期调用即可
 */
void OLED_StatusTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
