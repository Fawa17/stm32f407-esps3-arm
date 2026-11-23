// FlashDrv.c - 内部 Flash 读写，应用和引导程序共用同一份实现。
//
// 走 HAL 而不是自己撸寄存器，主要是三点：
//   - HAL_FLASHEx_Erase 里已经设好了 PSIZE，自己写最容易漏这一步，
//     结果是写 32 位字静默失败（SR 里置错误位但函数看着"成功了"）
//   - 擦/写的超时和错误标志 HAL 会处理，写坏了能返回错误码
//   - 应用和引导程序都编这一份，不会出现两套实现行为不一致
//
// 依赖 HAL_GetTick，引导程序里记得先 HAL_Init()。

#include "FlashDrv.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/* 基础操作                                                                     */
/* -------------------------------------------------------------------------- */

uint8_t FlashDrv_Unlock(void) {
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }
    return 1U;
}

void FlashDrv_Lock(void) {
    (void)HAL_FLASH_Lock();
}

uint8_t FlashDrv_EraseSector(uint32_t sector) {
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0U;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = sector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7~3.6V：允许 x32 并行写 */

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        return 0U;
    }
    FlashDrv_FlushCaches();
    return 1U;
}

uint8_t FlashDrv_ProgramWord(uint32_t addr, uint32_t data) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data) != HAL_OK) {
        return 0U;
    }
    FlashDrv_FlushCaches();
    return 1U;
}

uint8_t FlashDrv_ProgramBuffer(uint32_t addr, const uint8_t *buf, uint32_t len) {
    uint32_t i, word;

    for (i = 0U; i < len; i += 4U) {
        word = (uint32_t)buf[i] | ((uint32_t)buf[i + 1U] << 8) | ((uint32_t)buf[i + 2U] << 16) |
               ((uint32_t)buf[i + 3U] << 24);
        if (FlashDrv_ProgramWord(addr + i, word) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

uint8_t FlashDrv_IsBlank(uint32_t addr, uint32_t len) {
    uint32_t i;

    for (i = 0U; i < len; i += 4U) {
        if (*(volatile uint32_t *)(addr + i) != 0xFFFFFFFFUL) {
            return 0U;
        }
    }
    return 1U;
}

/** 把 src 的内容按字拷到 dst（跳过擦除态 0xFFFFFFFF，省时间） 只用于"暂存区/备份区 →
 * 应用区"这种整段搬运 */
uint8_t FlashDrv_CopyWords(uint32_t dst, uint32_t src, uint32_t len) {
    uint32_t i, word;

    for (i = 0U; i < len; i += 4U) {
        word = *(volatile uint32_t *)(src + i);
        if (word == 0xFFFFFFFFUL) {
            continue;
        }
        if (FlashDrv_ProgramWord(dst + i, word) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

/** 复位指令/数据 Cache F4 的 ART 在编程/擦除后会失效，不复位的话"写完立刻读回算 CRC" */
void FlashDrv_FlushCaches(void) {
    FLASH->ACR &= ~((1UL << 11) | (1UL << 12)); /* ICRST / DCRST */
    FLASH->ACR |= ((1UL << 11) | (1UL << 12));
    FLASH->ACR &= ~((1UL << 11) | (1UL << 12));
    FLASH->ACR |= ((1UL << 9) | (1UL << 10)); /* ICEN / DCEN */
}

/* -------------------------------------------------------------------------- */
/* CRC32                                                                       */
/* -------------------------------------------------------------------------- */

static const uint32_t s_crc_table[16] = {0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
                                         0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
                                         0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
                                         0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL};

uint32_t FlashDrv_Crc32Update(uint32_t crc, const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t i;

    for (i = 0U; i < len; i++) {
        crc ^= (uint32_t)p[i];
        crc = (crc >> 4) ^ s_crc_table[crc & 0x0FUL];
        crc = (crc >> 4) ^ s_crc_table[crc & 0x0FUL];
    }
    return crc;
}

uint32_t FlashDrv_Crc32(uint32_t addr, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFUL;

    crc = FlashDrv_Crc32Update(crc, (const void *)addr, len);
    return ~crc; /* 与 zlib.crc32 的结果一致 */
}
