/**
 * @file    bootloader_main.c
 * @brief   STM32F407 最小引导程序（不依赖 HAL，只用 CMSIS 寄存器）
 *
 * 职责：
 *   1) 上电后判断"有没有待安装的新固件"（看元数据区 + 暂存区 CRC）；
 *   2) 有：把暂存区（staging）的镜像搬到应用区（app），校验通过后标记完成；
 *   3) 没有 / 装完：数一数"连续几次没跑起来"，必要时从 W25Q16 备份回滚；
 *   4) 记一次启动，跳到应用区执行。
 *
 * 为什么能"刷不死"：
 *   - 引导程序只占 0x08000000~0x0800FFFF，应用区在它之后，IAP 永远碰不到它；
 *   - 新固件先落暂存区，元数据里的 state 只有在"整段 CRC 通过"之后才写；
 *   - 拷贝途中掉电 → state 仍是 PENDING，下次上电重新拷一遍，暂存区还在；
 *   - 暂存区坏了 / 没元数据 → 直接启动老应用；
 *   - 新固件能烧进去但"跑不起来"（HardFault / 卡死）→ 连续 3 次没盖健康印章，
 *     就从 W25Q16 里那份上一版好镜像恢复（见下）。
 *
 * Flash 布局（F407VE 512KB，扇区 0~3 各 16KB、4=64KB、5/6/7 各 128KB）：
 *   0x08000000 - 0x0800FFFF  64KB   引导程序（sector 0~3，ST-Link 烧一次即可）
 *   0x08010000 - 0x0803FFFF  192KB  应用区  APP_BASE（sector 4~5）
 *   0x08040000 - 0x0805FFFF  128KB  暂存区  STAGE_BASE（sector 6）
 *   0x08060000 - 0x0807FFFF  128KB  元数据  META_BASE（sector 7）
 *
 * sector 7 的用法（与 BSP_Drivers/Backup.h 必须一致）：
 *   +0x00  meta_t {magic, size, crc, state}
 *   +0x10  uint32_t boot_slots[8]   每次启动占一个（写 "BOOK"），最多 8 次
 *   +0x30  uint32_t seal            应用跑健康后写 "SEAL"
 *   +0x34  uint32_t seal_ver
 *
 * W25Q16 备份（与 Backup.h 必须一致）：
 *   0x001000  备份头 {magic "BAK1", size, crc, tag, ...}
 *   0x002000  镜像本体
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "FlashDrv.h"

#define APP_BASE 0x08010000UL
#define APP_SECTORS_LO 4U /* 应用区 = sector 4 + 5 */
#define APP_SECTORS_HI 5U
#define APP_REGION_SIZE 0x00030000UL
#define STAGE_BASE 0x08040000UL
#define STAGE_MAX 0x00020000UL /* 128KB */

#define META_BASE 0x08060000UL
#define META_MAGIC 0x54414D45UL    /* "EMAT" */
#define META_PENDING 0x00000001UL  /* 待安装 */
#define META_RESTORED 0x00000002UL /* 已回滚过，别再回滚 */

#define BOOT_SLOT_OFF 0x10UL
#define BOOT_SLOT_NUM 8U
#define BOOT_MARK 0x4B4F4F42UL /* "BOOK" */
#define BOOT_MAX_TRIES 3U      /* 连续 3 次没印章 → 回滚 */
#define BL_MIN_APP_SIZE 4096UL /* 小于 4KB 的一定不是应用镜像 */

#define SEAL_OFF 0x30UL
#define SEAL_MAGIC 0x4C414553UL /* "SEAL" */

#define BAK_HDR_ADDR 0x001000UL
#define BAK_IMG_ADDR 0x002000UL
#define BAK_MAGIC 0x314B4142UL /* "BAK1" */

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc;
    uint32_t state;
} meta_t;

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc;
    uint32_t tag;
    uint32_t reserved[4];
} backup_hdr_t;

/* --- 寄存器级 Flash --- */
#define BL_CR_PG (1UL << 0)
#define BL_CR_SER (1UL << 1)
#define BL_CR_SNB_Pos 3U
#define BL_CR_STRT (1UL << 16)
#define BL_CR_LOCK (1UL << 31)
#define BL_CR_PSIZE_Pos 8U
#define BL_CR_PSIZE_32 (2UL << BL_CR_PSIZE_Pos) /* x32 位宽：写 32 位字必须设 */
#define BL_SR_BSY (1UL << 16)
#define BL_SR_ERRORS ((1UL << 1) | (1UL << 2) | (1UL << 4) | (1UL << 6) | (1UL << 7))

/* 启动文件会调用它；这里故意留空时钟：HSI 16MHz 足够跑 CRC 与拷贝，
   也避免 HSE 没起振时卡死在时钟配置里（应用自己会配 168MHz）。
   但 FPU 必须在这里打开：armcc 的启动代码会执行 _fp_init（VMSR FPSCR），
   CPACR 没开 CP10/CP11 就会直接进 HardFault —— 这正是"引导程序一上电就挂、
   元数据区永远是 FF"的真因。 */
void SystemInit(void) {
    SCB->CPACR |= (0xFUL << 20); /* CP10/CP11 full access */
    __DSB();
    __ISB();
}

/* -------------------------------------------------------------------------- */
/* HAL 支撑：引导程序不跑 RTOS，但仍然用 HAL 的 Flash 驱动（见文件头说明）      */
/* -------------------------------------------------------------------------- */

/* system_stm32f4xx.c 不在引导工程里，HAL_Init 需要这个变量判断 SysTick 分频 */
uint32_t SystemCoreClock = 16000000UL; /* 引导程序固定跑 HSI 16MHz */

/* HAL_IncTick 的入口：keil 的启动文件里 SysTick_Handler 是弱定义的死循环，
   必须自己提供，否则 HAL_Init() 打开 SysTick 后 1ms 就卡死。 */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/* -------------------------------------------------------------------------- */
/* CRC 与 Flash：全部委托给 FlashDrv（与应用侧同一份实现）                      */
/* -------------------------------------------------------------------------- */

/* 增量 CRC32（流式）：BL_CRC32 和"先校验备份、后擦应用区"都靠它 */
static uint32_t BL_Crc32Update(uint32_t crc, const uint8_t *p, uint32_t len) {
    return FlashDrv_Crc32Update(crc, p, len);
}

static uint32_t BL_CRC32(const void *data, uint32_t len) {
    return FlashDrv_Crc32((uint32_t)data, len);
}

static void BL_FlashUnlock(void) {
    (void)FlashDrv_Unlock();
}

static void BL_FlashLock(void) {
    FlashDrv_Lock();
}

static void BL_EraseSector(uint32_t sector) {
    (void)FlashDrv_EraseSector(sector);
}

static void BL_WriteWord(uint32_t addr, uint32_t data) {
    (void)FlashDrv_ProgramWord(addr, data);
}

static void BL_EraseMeta(void) {
    BL_FlashUnlock();
    BL_EraseSector(7U);
    BL_FlashLock();
}

/* ------------------------------------------------------------------ */
/* 位翻转 SPI2：PB12=CS PB13=SCK PB14=MISO PB15=MOSI                    */
/* 为什么不用 HAL：引导程序要保持 1KB 级体积，且 CubeMX 重新生成也不受影响 */
/* ------------------------------------------------------------------ */
#define W25_CS_LOW() (GPIOB->BSRR = (1UL << (12U + 16U)))
#define W25_CS_HIGH() (GPIOB->BSRR = (1UL << 12U))
#define W25_SCK_LOW() (GPIOB->BSRR = (1UL << (13U + 16U)))
#define W25_SCK_HIGH() (GPIOB->BSRR = (1UL << 13U))
#define W25_MOSI_LOW() (GPIOB->BSRR = (1UL << (15U + 16U)))
#define W25_MOSI_HIGH() (GPIOB->BSRR = (1UL << 15U))
#define W25_MISO() ((GPIOB->IDR >> 14U) & 1UL)

static void Bb_Delay(void) {
    volatile int i;

    for (i = 0; i < 6; i++) {
        /* 空转；HSI 16MHz 下把 SCK 控制在 ~1MHz 量级，飞线也稳 */
    }
}

static void Bb_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    /* PB12/PB13/PB15 = 通用推挽输出；PB14 = 输入 + 上拉 */
    GPIOB->MODER &=
        ~((3UL << (12U * 2U)) | (3UL << (13U * 2U)) | (3UL << (14U * 2U)) | (3UL << (15U * 2U)));
    GPIOB->MODER |= ((1UL << (12U * 2U)) | (1UL << (13U * 2U)) | (1UL << (15U * 2U)));
    GPIOB->OTYPER &= ~((1UL << 12U) | (1UL << 13U) | (1UL << 15U));
    GPIOB->OSPEEDR |= ((3UL << (12U * 2U)) | (3UL << (13U * 2U)) | (3UL << (15U * 2U)));
    GPIOB->PUPDR &= ~((3UL << (12U * 2U)) | (3UL << (13U * 2U)));
    GPIOB->PUPDR &= ~(3UL << (14U * 2U));
    GPIOB->PUPDR |= (1UL << (14U * 2U));

    W25_CS_HIGH();
    W25_SCK_LOW();
}

static uint8_t Bb_Xfer(uint8_t out) {
    uint8_t in = 0U;
    int b;

    for (b = 7; b >= 0; b--) {
        if (((out >> b) & 1U) != 0U) {
            W25_MOSI_HIGH();
        } else {
            W25_MOSI_LOW();
        }
        Bb_Delay();
        W25_SCK_HIGH();
        Bb_Delay();
        in = (uint8_t)((in << 1) | (uint8_t)W25_MISO());
        W25_SCK_LOW();
    }
    return in;
}

static void Bb_Read(uint32_t addr, uint8_t *buf, uint32_t len) {
    uint32_t i;

    W25_CS_LOW();
    (void)Bb_Xfer(0x03U); /* READ */
    (void)Bb_Xfer((uint8_t)(addr >> 16));
    (void)Bb_Xfer((uint8_t)(addr >> 8));
    (void)Bb_Xfer((uint8_t)addr);

    for (i = 0U; i < len; i++) {
        buf[i] = Bb_Xfer(0xFFU);
    }
    W25_CS_HIGH();
}

/* ------------------------------------------------------------------ */
/* 从 W25Q16 备份恢复应用区                                            */
/* ------------------------------------------------------------------ */
static uint32_t BL_RestoreFromBackup(void) {
    backup_hdr_t h;
    uint8_t buf[64];
    uint32_t off, i, n, words, crc;

    Bb_Init();
    Bb_Read(BAK_HDR_ADDR, (uint8_t *)&h, (uint32_t)sizeof(h));

    if (h.magic != BAK_MAGIC) {
        return 0UL;
    }
    if ((h.size < BL_MIN_APP_SIZE) || (h.size > APP_REGION_SIZE)) {
        return 0UL;
    }

    /* ⚠️ 关键顺序：先把备份镜像从头到尾算一遍 CRC，确认它**真的可用**，再去擦应用区。
       反过来的话，备份一旦损坏（写盘掉电 / SPI 干扰），就会把好固件和暂存镜像
       一起毁掉 —— 这是最容易把板子变成砖的一条路径。 */
    crc = 0xFFFFFFFFUL;
    for (off = 0U; off < h.size; off += n) {
        n = h.size - off;
        if (n > sizeof(buf)) {
            n = sizeof(buf);
        }
        Bb_Read(BAK_IMG_ADDR + off, buf, n);
        crc = BL_Crc32Update(crc, buf, n);
    }
    if ((~crc) != h.crc) {
        return 0UL; /* 备份坏了：一个字节都别动应用区 */
    }

    words = (h.size + 3UL) / 4UL;

    BL_FlashUnlock();
    BL_EraseSector(APP_SECTORS_LO);
    BL_EraseSector(APP_SECTORS_HI);

    for (off = 0U; off < words * 4UL; off += n) {
        uint32_t w;

        n = (words * 4UL) - off;
        if (n > sizeof(buf)) {
            n = sizeof(buf);
        }
        Bb_Read(BAK_IMG_ADDR + off, buf, n);

        for (i = 0U; i < n; i += 4UL) {
            w = (uint32_t)buf[i] | ((uint32_t)buf[i + 1U] << 8) | ((uint32_t)buf[i + 2U] << 16) |
                ((uint32_t)buf[i + 3U] << 24);
            if ((off + i) >= h.size) {
                break; /* 尾巴不足 4 字节，补 0xFF 由擦除态负责 */
            }
            if (w != 0xFFFFFFFFUL) {
                BL_WriteWord(APP_BASE + off + i, w);
            }
        }
    }
    BL_FlashLock();

    return (BL_CRC32((const void *)APP_BASE, h.size) == h.crc) ? 1UL : 0UL;
}

/* ------------------------------------------------------------------ */
/* 元数据区辅助                                                        */
/* ------------------------------------------------------------------ */
static uint32_t BL_MetaWord(uint32_t off) {
    return *(volatile uint32_t *)(META_BASE + off);
}

static void BL_MetaWrite(uint32_t off, uint32_t val) {
    BL_FlashUnlock();
    BL_WriteWord(META_BASE + off, val);
    BL_FlashLock();
}

static void BL_MarkBoot(void) {
    uint32_t i;

    for (i = 0U; i < BOOT_SLOT_NUM; i++) {
        if (BL_MetaWord(BOOT_SLOT_OFF + i * 4UL) != BOOT_MARK) {
            BL_MetaWrite(BOOT_SLOT_OFF + i * 4UL, BOOT_MARK);
            return;
        }
    }
}

static uint32_t BL_CountBoots(void) {
    uint32_t i;
    uint32_t n = 0U;

    for (i = 0U; i < BOOT_SLOT_NUM; i++) {
        if (BL_MetaWord(BOOT_SLOT_OFF + i * 4UL) == BOOT_MARK) {
            n++;
        }
    }
    return n;
}

/* 暂存区里的东西像不像一个 F407 应用？
 * 只查两件事：向量表第一个字是合法的初始 MSP（落在 SRAM），
 * 第二个字是复位入口（落在应用区内）。
 * 目的：防止把"CRC 对但内容不是固件"的小文件装进去，把应用区开头覆盖掉。
 * 注意：引导程序里没有 printf，校验失败只能靠"没有装进去"这个行为判断。 */
static uint32_t BL_StagedLooksValid(uint32_t size) {
    uint32_t sp;
    uint32_t pc;

    if (size < BL_MIN_APP_SIZE) {
        return 0UL; /* 比任何正常应用都小，直接拒 */
    }
    sp = *(const volatile uint32_t *)(STAGE_BASE);
    pc = *(const volatile uint32_t *)(STAGE_BASE + 4UL);

    if ((sp & 0x2FFE0000UL) != 0x20000000UL) {
        return 0UL; /* MSP 不在 SRAM（0x2000_0000 段） */
    }
    if ((pc < APP_BASE) || (pc >= (APP_BASE + size))) {
        return 0UL; /* 入口不在应用区里 */
    }
    return 1UL;
}

/* 把暂存区整段拷到应用区；返回 1 表示应用区 CRC 与元数据一致 */
static uint32_t BL_Install(uint32_t size, uint32_t crc) {
    uint32_t words = (size + 3UL) / 4UL;
    uint32_t i;

    BL_FlashUnlock();
    BL_EraseSector(APP_SECTORS_LO);
    BL_EraseSector(APP_SECTORS_HI);

    for (i = 0U; i < words; i++) {
        uint32_t w = *(const volatile uint32_t *)(STAGE_BASE + i * 4UL);
        if (w != 0xFFFFFFFFUL) { /* 擦除态跳过，省时间 */
            BL_WriteWord(APP_BASE + i * 4UL, w);
        }
    }
    BL_FlashLock();

    return (BL_CRC32((const void *)APP_BASE, size) == crc) ? 1UL : 0UL;
}

static uint32_t BL_JumpToApp(void) {
    uint32_t sp = *(volatile uint32_t *)(APP_BASE);
    uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4UL);
    void (*entry)(void);

    /* 判据：栈顶落在 SRAM(0x20000000..0x20020000)、PC 在应用区且 Thumb 位为 1。
       sp 全 1 / pc 全 1 都被这里挡掉。 */
    if ((sp < 0x20000000UL) || (sp > 0x20020000UL) || ((sp & 0x3UL) != 0UL)) {
        return 0UL;
    }
    if ((pc & 0xFF000000UL) != 0x08000000UL) {
        return 0UL;
    }
    if ((pc & 1UL) == 0UL) {
        return 0UL; /* Thumb 位必须为 1，否则跳过去就 HardFault */
    }

    /* ARMv7-M 要求：先关中断、写 VTOR、再等 DSB/ISB 生效，之后才能取指。
       少了这两条屏障，偶发会"跳过去又跑回引导程序"。 */
    __disable_irq();
    SCB->VTOR = APP_BASE;
    __DSB();
    __ISB();

    __set_MSP(sp); /* CMSIS 内联函数，armcc / armclang 都能编 */
    __DSB();
    entry = (void (*)(void))pc;
    entry();

    return 1UL; /* 走不到这里（entry 不返回） */
}

int main(void) {
    meta_t *m = (meta_t *)META_BASE;
    uint32_t boots;

    /* 0) HAL 初始化：只为得到 1ms tick（Flash 驱动的超时判断依赖 HAL_GetTick）。
        FPU 已在 SystemInit 里打开；这里不配时钟，保持 HSI 16MHz。 */
    (void)HAL_Init();
    BL_FlashUnlock();
    BL_FlashLock(); /* 确认一次开关可用，异常时能早暴露 */

    /* 1) 装新固件：暂存区 CRC 对得上才覆盖应用区 */
    if ((m->magic == META_MAGIC) && (m->state == META_PENDING) && (m->size > 0UL) &&
        (m->size <= STAGE_MAX)) {
        if (BL_CRC32((const void *)STAGE_BASE, m->size) == m->crc) {
            if (BL_StagedLooksValid(m->size) == 0UL) {
                BL_EraseMeta(); /* 不像应用镜像：直接丢弃，绝不动应用区 */
            } else if (BL_Install(m->size, m->crc) != 0UL) {
                BL_EraseMeta(); /* 装上并校验通过：计数/印章一起清零 */
            }
        } else {
            BL_EraseMeta(); /* 暂存区坏了：清掉 PENDING，免得每次上电重试 */
        }
    }

    /* 2) 上次运行是健康的（应用盖过章）：计数归零，从头开始数 */
    if (BL_MetaWord(SEAL_OFF) == SEAL_MAGIC) {
        BL_EraseMeta();
    }

    /* 3) 连续多少次没跑起来？ */
    boots = BL_CountBoots();

    /* 4) 连着 3 次没印章 → 回滚到 W25Q16 里的上一版（只回滚一次） */
    if ((boots >= BOOT_MAX_TRIES) && (BL_MetaWord(12UL) != META_RESTORED)) {
        if (BL_RestoreFromBackup() != 0UL) {
            BL_EraseMeta();
            BL_FlashUnlock();
            BL_WriteWord(META_BASE, META_MAGIC);
            BL_WriteWord(META_BASE + 12UL, META_RESTORED);
            BL_FlashLock();
        }
    }

    /* 5) 记一次启动，然后交给应用 */
    BL_MarkBoot();

    if (BL_JumpToApp() == 0UL) {
        /* 应用区不可用（镜像坏了 / 还没烧过 app）。先试一次备份恢复，
           恢不回来就停在这里等 SWD —— 静默 for(;;) 至少不会把好固件继续擦掉。 */
        if (BL_RestoreFromBackup() != 0UL) {
            BL_EraseMeta();
            BL_FlashUnlock();
            BL_WriteWord(META_BASE, META_MAGIC);
            BL_WriteWord(META_BASE + 12UL, META_RESTORED);
            BL_FlashLock();
            NVIC_SystemReset(); /* 复位后按新镜像重新走一遍 */
        }
    }

    for (;;) {
        /* 应用区和备份都不可用：停在这里，用 SWD 重新烧 bootloader + app */
    }
}
