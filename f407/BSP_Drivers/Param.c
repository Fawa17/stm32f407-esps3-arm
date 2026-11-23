/** ***************************************************************************** 参数持久化（W25Q16
 * 第 0 扇区） ***************************************************************************** */
#include "Param.h"
#include "W25Qxx.h"
#include "RobotArm.h"
#include <string.h>

#define PARAM_ADDR 0x000000UL
#define PARAM_MAGIC 0x4D524150UL /* "PARM" */
#define PARAM_VERSION 1U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t tmin[ROBOTARM_AXIS_NUM];
    uint32_t tmax[ROBOTARM_AXIS_NUM];
    uint16_t speed[ROBOTARM_AXIS_NUM];
    uint16_t accel[ROBOTARM_AXIS_NUM];
    uint32_t crc; /* 结构体前面所有字节的 CRC32 */
} Param_t;

static uint8_t s_flash_ok = 0;
static Param_t s_cur; /* 影子副本，同时也是写入缓冲 */

/* 与 IAP/引导程序同一套 CRC-32（反射 0xEDB88320，与 zlib 一致） */
static uint32_t Crc32(const uint8_t *p, uint32_t n) {
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t b;

    for (i = 0; i < n; i++) {
        crc ^= (uint32_t)p[i];
        for (b = 0; b < 8U; b++) {
            if ((crc & 1UL) != 0UL) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

static uint32_t StructCrc(const Param_t *p) {
    return Crc32((const uint8_t *)p, (uint32_t)(sizeof(Param_t) - sizeof(uint32_t)));
}

/* 把影子副本应用到底层驱动 */
static void Param_Apply(void) {
    uint8_t i;

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        if (s_cur.tmin[i] >= 100UL && s_cur.tmax[i] > s_cur.tmin[i] + 400UL &&
            s_cur.tmax[i] <= 40000UL) {
            RobotArm_SetCalib(i, s_cur.tmin[i], s_cur.tmax[i]);
        }
        if (s_cur.speed[i] > 0U) {
            RobotArm_SetSpeed(i, s_cur.speed[i]);
        }
        if (s_cur.accel[i] > 0U) {
            RobotArm_SetAccel(i, s_cur.accel[i]);
        }
    }
}

/* 从底层驱动抓一份当前值到影子副本 */
static void Param_Snapshot(void) {
    uint8_t i;

    s_cur.magic = PARAM_MAGIC;
    s_cur.version = PARAM_VERSION;
    s_cur.count = ROBOTARM_AXIS_NUM;

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        RobotArm_GetCalib(i, &s_cur.tmin[i], &s_cur.tmax[i]);
        s_cur.speed[i] = RobotArm_GetSpeed(i);
        s_cur.accel[i] = RobotArm_GetAccel(i);
    }

    s_cur.crc = StructCrc(&s_cur);
}

uint8_t Param_Init(void) {
    uint8_t rc = W25Q_Init();

    if (rc != 0U) {
        s_flash_ok = 0U;
        return 2U;
    } /* 没芯片：不影响其他功能 */
    s_flash_ok = 1U;

    memset(&s_cur, 0, sizeof(s_cur));
    W25Q_Read(PARAM_ADDR, (uint8_t *)&s_cur, (uint32_t)sizeof(s_cur));

    if (s_cur.magic != PARAM_MAGIC || s_cur.count != ROBOTARM_AXIS_NUM) {
        return 1U; /* 空白/别的内容 */
    }
    if (s_cur.crc != StructCrc(&s_cur)) {
        return 1U; /* 损坏，丢弃 */
    }

    Param_Apply();
    return 0U;
}

uint8_t Param_Save(void) {
    if (!s_flash_ok) {
        return 1U;
    }

    Param_Snapshot();

    if (W25Q_EraseSector(PARAM_ADDR) != 0U) {
        return 2U;
    }
    if (W25Q_Write(PARAM_ADDR, (const uint8_t *)&s_cur, (uint32_t)sizeof(s_cur)) != 0U) {
        return 3U;
    }

    return 0U;
}

uint8_t Param_Ready(void) {
    return s_flash_ok;
}
