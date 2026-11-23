/** IAP.h - 应用内升级：新固件先收到暂存区，校验通过才写元数据， 下次重启由引导程序搬进应用区。 */
#ifndef __IAP_H
#define __IAP_H

#include <stdint.h>

#define IAP_OK 1U
#define IAP_ERR 0U

/* 与引导程序（bootloader_main.c）里的常量必须完全一致 */
#define IAP_APP_BASE 0x08010000UL
#define IAP_STAGE_BASE 0x08040000UL
#define IAP_STAGE_MAX 0x00020000UL
#define IAP_MIN_APP_SIZE 4096UL /* 小于 4KB 的一定不是应用镜像，直接拒收 */
#define IAP_META_BASE 0x08060000UL
#define IAP_META_MAGIC 0x54414D45UL
#define IAP_META_PENDING 0x00000001UL
#define IAP_META_DONE 0x00000000UL

uint8_t IAP_Begin(uint32_t size, uint32_t crc);
uint8_t IAP_Write(uint32_t offset, const uint8_t *data, uint32_t len);
uint8_t IAP_End(uint32_t crc);
uint32_t IAP_Progress(void); /* 已接收字节数，供状态显示 */
uint8_t IAP_Active(void);
void IAP_Touch(void); /* 收到一条 U/W/E 时调用，刷新"上位机还活着" */
void IAP_Tick(void);  /* netTask 周期调用：超时自动退出维护模式 */
void IAP_Reboot(void);

/** 往元数据区（sector 7）编程一个 32 位字，不擦除 备份模块用它写 "SEAL" 健康印章（偏移
 * 0x30），引导程序读同一地址。 */
uint8_t IAP_WriteMetaWord(uint32_t offset, uint32_t value);

#endif /* __IAP_H */
