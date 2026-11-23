/** Param.h - 标定表 / 速度 / 加速度存到 W25Q16，掉电不丢 位置：W25Q16 第 0 扇区（4KB 只用前 80
 * 字节左右）。 */
#ifndef __PARAM_H
#define __PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/** 初始化 W25Q16 并尝试载入参数（有效就应用到 RobotArm） 1 = Flash 可用但没有有效参数（用默认值）
 */
uint8_t Param_Init(void);

/** 把当前 RobotArm 的标定/速度/加速度写回 Flash（擦一个扇区 + 写 80 字节，约 50ms） */
uint8_t Param_Save(void);

/** Flash 是否可用（Param_Init 检测过） */
uint8_t Param_Ready(void);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_H */
