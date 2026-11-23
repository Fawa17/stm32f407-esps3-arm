/** W25Qxx.h - W25Q16（2MB SPI NOR）读 / 页写 / 4KB 扇区擦 接线（SPI2，全在 GPIOB）： */
#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---- 引脚（要改接线只改这里）---- */
#define W25Q_CS_PORT GPIOB
#define W25Q_CS_PIN GPIO_PIN_12
#define W25Q_SCK_PORT GPIOB
#define W25Q_SCK_PIN GPIO_PIN_13
#define W25Q_MISO_PORT GPIOB
#define W25Q_MISO_PIN GPIO_PIN_14
#define W25Q_MOSI_PORT GPIOB
#define W25Q_MOSI_PIN GPIO_PIN_15

#define W25Q_SECTOR_SIZE 4096U
#define W25Q_PAGE_SIZE 256U
#define W25Q16_CAPACITY (2UL * 1024UL * 1024UL)

/** 初始化 SPI2 + 引脚，读 JEDEC ID 校验芯片 */
uint8_t W25Q_Init(void);

/** 读 JEDEC ID（0xEF4015 = W25Q16） */
uint32_t W25Q_ReadID(void);

/** 从任意地址读 len 字节（跨页、跨扇区都行） */
void W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len);

/** 从任意地址写 len 字节 调用者负责先擦除（NOR 只能 1→0）。内部按 256 字节页边界切分。 */
uint8_t W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len);

/** 擦除一个 4KB 扇区（addr 会被对齐到扇区首地址） */
uint8_t W25Q_EraseSector(uint32_t addr);

/** 把 [addr, addr+len) 覆盖到的扇区全部擦掉（会丢掉同扇区里的其他数据） */
uint8_t W25Q_EraseRange(uint32_t addr, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */
