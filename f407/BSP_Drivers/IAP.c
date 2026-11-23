/*
 * IAP.c - 应用内升级
 *
 * 数据只写暂存区（sector 6），不碰正在跑的程序。U 命令时先把元数据区（sector 7）
 * 擦干净，等整段 CRC 校验通过才写 {magic,size,crc,PENDING}。中途掉电 = 元数据不
 * 存在 = 下次照常启动老固件。
 */

#include "IAP.h"
#include "main.h"
#include "Watchdog.h"
#include "FlashDrv.h"

#define IAP_KEY1 0x45670123UL
#define IAP_KEY2 0xCDEF89ABUL
#define CR_PG (1UL << 0)
#define CR_SER (1UL << 1)
#define CR_SNB_Pos 3U
#define CR_STRT (1UL << 16)
#define CR_LOCK (1UL << 31)
#define CR_PSIZE_Pos 8U
#define CR_PSIZE_32 (2UL << CR_PSIZE_Pos) /* x32 位宽：写 32 位字必须设这个 */
#define SR_BSY (1UL << 16)
#define SR_ERRORS ((1UL << 1) | (1UL << 2) | (1UL << 4) | (1UL << 6) | (1UL << 7))

#define META_SECTOR 7U
#define STAGE_SECTOR 6U

static uint32_t s_size;    /* "U" 声明的总长度 */
static uint32_t s_written; /* 已接收字节数 */
static uint32_t s_crc;     /* 运行中的 CRC（未取反） */
static uint8_t s_word[4];  /* 未凑满一个字的尾巴 */
static uint32_t s_word_n;
static uint8_t s_active;   /* 已经 Begin 过、还在收数据 */
static uint32_t s_last_ms; /* 最近一次 IAP 活动的时刻（ms） */

/* 上位机半路断线（WiFi 掉了、网页关了）时，不能永远卡在维护模式里，
   否则机械臂再也收不到运动指令。这个超时一到就自动退出。 */
#define IAP_IDLE_TIMEOUT_MS 30000UL

/* -------------------------------------------------------------------------- */

/* 擦除/编程/缓存刷新统一走 FlashDrv（与引导程序同一份实现）。
 * 这里保留薄封装，是为了让下面 IAP 状态机读起来只关心"业务"，
 * 不必到处写 FlashDrv_xxx。 */
static void Flash_Unlock(void) {
    (void)FlashDrv_Unlock();
}

static void Flash_Lock(void) {
    FlashDrv_Lock();
}

static uint8_t Flash_EraseSector(uint32_t sector) {
    return FlashDrv_EraseSector(sector);
}

static uint8_t Flash_WriteWord(uint32_t addr, uint32_t data) {
    return FlashDrv_ProgramWord(addr, data);
}

/* 增量 CRC：与 FlashDrv_Crc32 用同一张表，结果与 zlib.crc32 一致 */
static uint32_t Crc32_Byte(uint32_t crc, uint8_t b) {
    return FlashDrv_Crc32Update(crc, &b, 1U);
}

/* -------------------------------------------------------------------------- */

uint8_t IAP_Active(void) {
    return s_active;
}

/* 每处理一条 U/W/E 就刷新一次，代表"上位机还活着" */
void IAP_Touch(void) {
    s_last_ms = HAL_GetTick();
}

/* 由 netTask 每轮调用：超过 IAP_IDLE_TIMEOUT_MS 没有 U/W/E 就自动退出维护模式 */
void IAP_Tick(void) {
    if (s_active == 0U) {
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_last_ms) > IAP_IDLE_TIMEOUT_MS) {
        s_active = 0U; /* 上位机断线：机械臂恢复可用，应用固件不受影响 */
    }
}

/* 给元数据区写一个 32 位字（备份模块用来盖"健康印章"）。
 * 只做"编程"，不擦除：调用者必须确认目标字还是 0xFFFFFFFF（NOR 只能 1→0）。 */
uint8_t IAP_WriteMetaWord(uint32_t offset, uint32_t value) {
    uint8_t rc;

    if (offset > 0x4000UL) { /* 只允许写 sector 7 的前 16KB */
        return IAP_ERR;
    }

    Flash_Unlock();
    rc = Flash_WriteWord(IAP_META_BASE + offset, value);
    Flash_Lock();

    return (rc == 0U) ? IAP_OK : IAP_ERR;
}

uint32_t IAP_Progress(void) {
    return s_written;
}

uint8_t IAP_Begin(uint32_t size, uint32_t crc) {
    (void)crc; /* 长度以本条为准，CRC 在 E 阶段比对 */

    if (size < IAP_MIN_APP_SIZE || size > IAP_STAGE_MAX) {
        return IAP_ERR; /* 太小一定不是应用镜像：直接拒，别让它进引导程序 */
    }

    /* 上位机重发同一个 U（因为擦除太慢导致超时重试）：不要重复擦，直接认 */
    if (s_active != 0U && s_size == size && s_written == 0UL) {
        s_last_ms = HAL_GetTick(); /* 重发也算活动，别让 IAP_Tick 把会话超时清掉 */
        return IAP_OK;
    }

    s_active = 0U;
    Flash_Unlock();

    /* 擦一个 128KB 扇区要 1~2 秒，这期间喂不了狗：擦之前先喂一次，
       两次擦除之间再喂一次（IWDG 约 6 秒，单次擦除不会超）。 */
    Watchdog_Feed();
    if (Flash_EraseSector(META_SECTOR) == 0U) {
        Flash_Lock();
        return IAP_ERR;
    }
    Watchdog_Feed();
    if (Flash_EraseSector(STAGE_SECTOR) == 0U) {
        Flash_Lock();
        return IAP_ERR;
    }
    Flash_Lock();
    Watchdog_Feed();

    s_size = size;
    s_written = 0UL;
    s_crc = 0xFFFFFFFFUL;
    s_word_n = 0UL;
    s_active = 1U;
    /* 记一次"刚活动过"。漏了这行会出大问题：netTask 每 5ms 调一次 IAP_Tick()，
     * 它拿 HAL_GetTick() 减 s_last_ms 判空闲；s_last_ms 还停在很久以前（0），
     * 减出来远超 30 秒 → 下一拍就把刚建立的会话清掉 → 第一条 W 必回 ERR。
     * 表现就是"U 成功、W 全失败、网页一直 OTA 失败"。 */
    s_last_ms = HAL_GetTick();
    return IAP_OK;
}

uint8_t IAP_Write(uint32_t offset, const uint8_t *data, uint32_t len) {
    uint32_t i;
    uint8_t ok = 1U;

    if (s_active == 0U || data == NULL) {
        return IAP_ERR;
    }
    if (offset != s_written) {
        return IAP_ERR; /* 必须严格顺序 */
    }
    if (s_written + len > s_size) {
        return IAP_ERR;
    }

    Flash_Unlock();

    for (i = 0U; i < len; i++) {
        uint8_t b = data[i];

        s_crc = Crc32_Byte(s_crc, b);
        s_word[s_word_n++] = b;
        s_written++;

        if (s_word_n == 4UL) {
            uint32_t w = (uint32_t)s_word[0] | ((uint32_t)s_word[1] << 8) |
                         ((uint32_t)s_word[2] << 16) | ((uint32_t)s_word[3] << 24);
            if (Flash_WriteWord(IAP_STAGE_BASE + (s_written - 4UL), w) == 0U) {
                ok = 0U;
                break;
            }
            s_word_n = 0UL;
        }
    }

    Flash_Lock();
    return ok;
}

uint8_t IAP_End(uint32_t crc) {
    uint32_t calc;
    uint32_t words[4];
    uint32_t i;

    if (s_active == 0U) {
        return IAP_ERR;
    }

    /* 补满最后一个字（补 0xFF，与擦除态一致；CRC 只算到 s_size） */
    if (s_word_n != 0UL) {
        uint32_t w = 0UL;
        uint32_t k;

        for (k = 0UL; k < 4UL; k++) {
            uint32_t b = (k < s_word_n) ? (uint32_t)s_word[k] : 0xFFUL;
            w |= b << (8UL * k);
        }
        Flash_Unlock();
        if (Flash_WriteWord(IAP_STAGE_BASE + (s_written - s_word_n), w) == 0U) {
            Flash_Lock();
            return IAP_ERR;
        }
        Flash_Lock();
        s_word_n = 0UL;
    }

    if (s_written != s_size) {
        return IAP_ERR; /* 少传了 */
    }

    calc = ~s_crc;
    if (calc != crc) {
        return IAP_ERR; /* 传输/写入出错 */
    }

    /* 写元数据：{magic, size, crc, PENDING} */
    words[0] = IAP_META_MAGIC;
    words[1] = s_size;
    words[2] = calc;
    words[3] = IAP_META_PENDING;

    Flash_Unlock();
    Watchdog_Feed(); /* 写元数据后马上就要重启，这里喂一次防止半路被狗咬 */
    for (i = 0UL; i < 4UL; i++) {
        if (Flash_WriteWord(IAP_META_BASE + i * 4UL, words[i]) == 0U) {
            Flash_Lock();
            return IAP_ERR;
        }
    }
    Flash_Lock();

    s_active = 0U;
    return IAP_OK;
}

void IAP_Reboot(void) {
    __disable_irq();
    NVIC_SystemReset();
}
