/** Backup.h - 固件备份 + 启动健康印章，用来做"新固件起不来就自动回滚" IAP
 * 只能保证新固件完整送到了暂存区，保证不了它能跑。新固件要是一上电就 */
#ifndef __BACKUP_H
#define __BACKUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---- W25Q16 内的备份地址（与 bootloader_main.c 必须一致） ---- */
#define BAK_HDR_ADDR 0x001000UL
#define BAK_IMG_ADDR 0x002000UL
#define BAK_MAX_IMAGE (192UL * 1024UL)
#define BAK_MAGIC 0x314B4142UL /* "BAK1" */

/* ---- sector 7 内的偏移（与 bootloader_main.c 必须一致） ---- */
#define BAK_SEAL_OFF 0x30UL
#define BAK_SEAL_MAGIC 0x4C414553UL /* "SEAL" */
#define BAK_SEALVER_OFF 0x34UL

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc;
    uint32_t tag; /* 固件标识，仅供显示/日志 */
    uint32_t reserved[4];
} backup_hdr_t;

/** 初始化 W25Q16 并读备份头 1 = 没检测到 W25Q16（备份/回滚功能不可用，其他功能正常） */
uint8_t Backup_Init(void);

/** 备份头是否有效且 CRC 与镜像匹配（这一步会读整份镜像算 CRC，约 10ms） */
uint8_t Backup_HasValid(void);

/** 取出备份头信息（无效时返回全 0） */
void Backup_GetInfo(backup_hdr_t *out);

/** 启动健康上报（由 WdTask 每 200ms 调一次，直到返回 1） 第 1 步：写 "SEAL"
 * 印章（很便宜，一次编程） */
uint8_t Backup_Healthy(void);

/** 立即把当前运行的应用镜像存进 W25Q16（返回 0 = 成功） */
uint8_t Backup_SaveRunning(void);

/** 当前运行镜像的字节数（扫描应用区末尾的 0xFF 填充得出） */
uint32_t Backup_ImageSize(void);

#ifdef __cplusplus
}
#endif

#endif /* __BACKUP_H */
