/** @file FlashDrv.h 内部 Flash 操作驱动（HAL 后端，应用与引导程序共用一份） */

#ifndef __FLASHDRV_H
#define __FLASHDRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- 分区表：必须与链接脚本（IROM）和引导程序的跳转目标严格一致 ---- */
#define FLASHDRV_APP_BASE 0x08010000UL   /* 应用区起始（sector 4）  */
#define FLASHDRV_APP_MAX 0x00030000UL    /* 应用区最大 192KB        */
#define FLASHDRV_STAGE_BASE 0x08040000UL /* 暂存区起始（sector 6）  */
#define FLASHDRV_STAGE_MAX 0x00020000UL  /* 暂存区 128KB            */
#define FLASHDRV_META_BASE 0x08060000UL  /* 元数据起始（sector 7）  */
#define FLASHDRV_META_MAX 0x00020000UL

#define FLASHDRV_META_SECTOR 7U
#define FLASHDRV_STAGE_SECTOR 6U

/* ---- 返回值：统一用 1=成功 / 0=失败，调用方不需要看 HAL_StatusTypeDef ---- */
uint8_t FlashDrv_Unlock(void);
void FlashDrv_Lock(void);
uint8_t FlashDrv_EraseSector(uint32_t sector);
uint8_t FlashDrv_ProgramWord(uint32_t addr, uint32_t data);
uint8_t FlashDrv_ProgramBuffer(uint32_t addr, const uint8_t *buf, uint32_t len);
uint8_t FlashDrv_IsBlank(uint32_t addr, uint32_t len);
uint8_t FlashDrv_CopyWords(uint32_t dst, uint32_t src, uint32_t len);
void FlashDrv_FlushCaches(void);

/* ---- CRC32（反射多项式 0xEDB88320，结果与 zlib.crc32 完全一致） ---- */
uint32_t FlashDrv_Crc32Update(uint32_t crc, const void *data, uint32_t len);
uint32_t FlashDrv_Crc32(uint32_t addr, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __FLASHDRV_H */
