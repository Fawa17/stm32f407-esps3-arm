/*
 * Backup.c - 固件备份区 + 启动健康印章
 * 依赖 W25Qxx 存镜像，IAP 读写 sector 7 元数据，RobotArm 判断是否已停稳。
 */

#include "Backup.h"
#include "W25Qxx.h"
#include "IAP.h"
#include "RobotArm.h"
#include "NetLink.h"

#include <string.h>

/* 应用区大小（引导程序那边叫 APP 区，sector 4+5） */
#define APP_REGION_SIZE 0x00030000UL

/* 自动备份开关：
 *   1 = 应用跑健康后，把当前镜像整份写进 W25Q16（首次约 40KB，会短暂阻塞 ~1s）
 *   0 = 只盖健康印章，不写备份（调试/SPI 没接好时用，避免反复写盘把任务饿死）
 * 注意：关掉自动备份后，"连续 3 次起不来就从 W25Q16 回滚"这条兜底会失效，
 *       但健康印章照盖，所以不会误判成"新固件坏了"。 */
#define BACKUP_AUTO_SAVE 1

/* Backup_Healthy() 的内部状态 */
#define BK_IDLE 0U
#define BK_SEALED 1U
#define BK_DONE 2U

static uint8_t s_ready = 0U; /* 0 = 芯片可用 */
static uint8_t s_state = BK_IDLE;
static backup_hdr_t s_hdr; /* 最近一次从 W25Q16 读到的备份头 */

/* ------------------------------------------------------------------ */
/* 工具：CRC32（与 IAP / 引导程序同一套）                              */
/* ------------------------------------------------------------------ */
static uint32_t Bak_CRC32(const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    int b;

    for (i = 0U; i < len; i++) {
        crc ^= (uint32_t)p[i];
        for (b = 0; b < 8; b++) {
            if ((crc & 1UL) != 0UL) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

/* 固件标识：把版本字符串揉成一个 32 位数，只用来在串口/网页上显示"备份是哪一版" */
static uint32_t Bak_Tag(void) {
    const char *v = NET_FW_VERSION;
    uint32_t h = 2166136261UL;

    while (*v != '\0') {
        h ^= (uint32_t)(uint8_t)(*v);
        h *= 16777619UL;
        v++;
    }
    return h;
}

/* ------------------------------------------------------------------ */
/* 镜像长度：从应用区末尾往前跳过 0xFF 填充                            */
/* ------------------------------------------------------------------ */
uint32_t Backup_ImageSize(void) {
    const uint32_t *base = (const uint32_t *)IAP_APP_BASE;
    const uint32_t *p = (const uint32_t *)(IAP_APP_BASE + APP_REGION_SIZE - 4U);

    while ((p > base) && (*p == 0xFFFFFFFFUL)) {
        p--;
    }

    return (uint32_t)(((const uint8_t *)p) - ((const uint8_t *)base)) + 4U;
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
uint8_t Backup_Init(void) {
    s_ready = 0U;
    s_state = BK_IDLE;
    (void)memset(&s_hdr, 0, sizeof(s_hdr));

    if (W25Q_Init() != 0U) {
        s_ready = 1U;
        return 1U;
    }

    W25Q_Read(BAK_HDR_ADDR, (uint8_t *)&s_hdr, (uint32_t)sizeof(s_hdr));
    return 0U;
}

uint8_t Backup_HasValid(void) {
    if (s_ready != 0U) {
        return 0U;
    }
    if (s_hdr.magic != BAK_MAGIC) {
        return 0U;
    }
    if ((s_hdr.size == 0U) || (s_hdr.size > BAK_MAX_IMAGE)) {
        return 0U;
    }

    /* 校验镜像本体（约 40KB，10ms 量级） */
    static uint8_t buf[256];
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t off = 0U;
    uint32_t n;
    uint32_t i;
    int b;

    while (off < s_hdr.size) {
        n = s_hdr.size - off;
        if (n > sizeof(buf)) {
            n = sizeof(buf);
        }
        W25Q_Read(BAK_IMG_ADDR + off, buf, n);
        for (i = 0U; i < n; i++) {
            crc ^= (uint32_t)buf[i];
            for (b = 0; b < 8; b++) {
                if ((crc & 1UL) != 0UL) {
                    crc = (crc >> 1) ^ 0xEDB88320UL;
                } else {
                    crc >>= 1;
                }
            }
        }
        off += n;
    }

    return ((~crc) == s_hdr.crc) ? 1U : 0U;
}

void Backup_GetInfo(backup_hdr_t *out) {
    if (out == NULL) {
        return;
    }
    (void)memcpy(out, &s_hdr, sizeof(s_hdr));
}

/* ------------------------------------------------------------------ */
/* 存一份当前镜像                                                      */
/* ------------------------------------------------------------------ */
uint8_t Backup_SaveRunning(void) {
    backup_hdr_t h;
    uint32_t size;
    uint32_t crc;

    if (s_ready != 0U) {
        return 1U;
    }

    size = Backup_ImageSize();
    if ((size == 0U) || (size > BAK_MAX_IMAGE)) {
        return 2U;
    }

    crc = Bak_CRC32((const void *)IAP_APP_BASE, size);

    /* 先写镜像，最后写头：中途掉电时旧头仍指向旧镜像，不会出现"头新身旧" */
    if (W25Q_EraseRange(BAK_IMG_ADDR, size) != 0U) {
        return 3U;
    }
    if (W25Q_Write(BAK_IMG_ADDR, (const uint8_t *)IAP_APP_BASE, size) != 0U) {
        return 4U;
    }

    h.magic = BAK_MAGIC;
    h.size = size;
    h.crc = crc;
    h.tag = Bak_Tag();
    (void)memset(h.reserved, 0, sizeof(h.reserved));

    if (W25Q_EraseSector(BAK_HDR_ADDR) != 0U) {
        return 5U;
    }
    if (W25Q_Write(BAK_HDR_ADDR, (const uint8_t *)&h, (uint32_t)sizeof(h)) != 0U) {
        return 6U;
    }

    s_hdr = h;
    return 0U;
}

/* ------------------------------------------------------------------ */
/* 启动健康上报                                                        */
/* ------------------------------------------------------------------ */
static uint8_t SealBoot(void) {
    volatile uint32_t *seal = (volatile uint32_t *)(IAP_META_BASE + BAK_SEAL_OFF);

    if (*seal == BAK_SEAL_MAGIC) {
        return 0U; /* 已经盖过，别重复编程（只能 1→0） */
    }
    if (*seal != 0xFFFFFFFFUL) {
        return 1U; /* 这个字已经被别的数据占了，不敢动 */
    }

    if (IAP_WriteMetaWord(BAK_SEAL_OFF, BAK_SEAL_MAGIC) != IAP_OK) {
        return 1U;
    }
    (void)IAP_WriteMetaWord(BAK_SEALVER_OFF, Bak_Tag());
    return 0U;
}

uint8_t Backup_Healthy(void) {
    if (s_state == BK_DONE) {
        return 1U;
    }

    if (s_state == BK_IDLE) {
        /* 先盖章：盖上了，即使后面备份失败，下次上电也不会被回滚 */
        if (SealBoot() == 0U) {
            s_state = BK_SEALED;
        }
        return 0U;
    }

    /* 机械臂还在动就先不折腾存储，避免升级瞬间动作卡顿 */
    if (RobotArm_TargetReached() == 0U) {
        return 0U;
    }

#if (BACKUP_AUTO_SAVE == 0)
    s_state = BK_DONE; /* 只盖章，不写备份 */
    return 1U;
#else

    uint32_t size = Backup_ImageSize();
    uint32_t crc = Bak_CRC32((const void *)IAP_APP_BASE, size);

    if (s_ready == 0U && s_hdr.magic == BAK_MAGIC && s_hdr.size == size && s_hdr.crc == crc) {
        s_state = BK_DONE; /* 备份已经是最新，什么都不用做 */
        return 1U;
    }

    if (Backup_SaveRunning() == 0U) {
        s_state = BK_DONE;
        return 1U;
    }

    return 0U; /* 这次失败，下个 200ms 再试 */
#endif /* BACKUP_AUTO_SAVE */
}
