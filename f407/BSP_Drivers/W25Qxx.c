/** ***************************************************************************** W25Q16
 * 驱动（SPI2，PB12~PB15，HAL 阻塞收发，10.5MHz） 只做最少的几件事：读 ID / 读 / 页写 / 扇区擦除。
 */
#include "W25Qxx.h"

/* ---- 指令 ---- */
#define CMD_WRITE_ENABLE 0x06U
#define CMD_READ_STATUS1 0x05U
#define CMD_READ_DATA 0x03U
#define CMD_PAGE_PROGRAM 0x02U
#define CMD_SECTOR_ERASE 0x20U
#define CMD_JEDEC_ID 0x9FU

#define SR1_BUSY 0x01U

static SPI_HandleTypeDef s_spi2;
static uint8_t s_ready = 0;

/* ---------------- 底层 ---------------- */
static void Cs_Low(void) {
    HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_RESET);
}
static void Cs_High(void) {
    HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET);
}

static void Spi_Tx(const uint8_t *d, uint16_t n) {
    HAL_SPI_Transmit(&s_spi2, (uint8_t *)d, n, 100U);
}

static void Spi_Rx(uint8_t *d, uint16_t n) {
    HAL_SPI_Receive(&s_spi2, d, n, 100U);
}

static uint8_t Spi_TxRx(uint8_t b) {
    uint8_t r = 0xFFU;
    HAL_SPI_TransmitReceive(&s_spi2, &b, &r, 1U, 100U);
    return r;
}

/** 等 BUSY 清零（擦除/写 期间芯片自己忙） */
static uint8_t Wait_Ready(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    uint8_t sr = 0xFFU;

    Cs_Low();
    (void)Spi_TxRx(CMD_READ_STATUS1);
    do {
        sr = Spi_TxRx(0xFFU);
        if ((sr & SR1_BUSY) == 0U) {
            break;
        }
    } while ((HAL_GetTick() - t0) < timeout_ms);
    Cs_High();

    return ((sr & SR1_BUSY) == 0U) ? 0U : 1U;
}

static void Write_Enable(void) {
    Cs_Low();
    (void)Spi_TxRx(CMD_WRITE_ENABLE);
    Cs_High();
}

/* ---------------- 引脚 / 外设初始化 ---------------- */
static void Spi2_GpioInit(void) {
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* CS：普通推挽输出，默认拉高（不选中） */
    HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET);
    g.Pin = W25Q_CS_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(W25Q_CS_PORT, &g);

    /* SCK / MISO / MOSI：AF5 = SPI2 */
    g.Pin = W25Q_SCK_PIN | W25Q_MISO_PIN | W25Q_MOSI_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP; /* 飞线时 MISO 上拉更稳 */
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);
}

static uint8_t Spi2_Init(void) {
    s_spi2.Instance = SPI2;
    s_spi2.Init.Mode = SPI_MODE_MASTER;
    s_spi2.Init.Direction = SPI_DIRECTION_2LINES;
    s_spi2.Init.DataSize = SPI_DATASIZE_8BIT;
    s_spi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_spi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    s_spi2.Init.NSS = SPI_NSS_SOFT;
    s_spi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; /* 42MHz/4 = 10.5MHz */
    s_spi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_spi2.Init.TIMode = SPI_TIMODE_DISABLE;
    s_spi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    s_spi2.Init.CRCPolynomial = 10U;

    return (HAL_SPI_Init(&s_spi2) == HAL_OK) ? 0U : 1U;
}

/* ---------------- 公共 API ---------------- */
uint32_t W25Q_ReadID(void) {
    uint8_t id[3] = {0, 0, 0};

    Cs_Low();
    (void)Spi_TxRx(CMD_JEDEC_ID);
    Spi_Rx(id, 3U);
    Cs_High();

    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | (uint32_t)id[2];
}

uint8_t W25Q_Init(void) {
    uint32_t id;

    if (s_ready != 0U) {
        return 0U; /* 已经初始化过：Param / Backup 都会调它，别重复折腾 SPI */
    }

    Spi2_GpioInit();
    if (Spi2_Init() != 0U) {
        s_ready = 0U;
        return 2U;
    }

    HAL_Delay(1);
    id = W25Q_ReadID();

    /* 0xEF4015 = W25Q16JV；0xEF4014 = W25Q80；0xEF4016 = W25Q32，都算这一族。
       判据只看厂商号（高字节 0xEF = Winbond）即可 —— 原来的
       (id & 0xFF00FF) == 0x00EF00 对真芯片 0xEF4015 恒为假，会把整块 Flash 判成"无芯片"，
       连带 Param（标定持久化）和 Backup（回滚兜底）全部失效。 */
    if (((id >> 16) & 0xFFUL) == 0xEFUL) {
        s_ready = 1U;
        return 0U;
    }
    if (id == 0x000000UL || id == 0xFFFFFFUL) {
        s_ready = 0U;
        return 1U;
    }

    s_ready = 1U;
    return 2U;
}

void W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len) {
    uint8_t cmd[4];

    if (!s_ready || len == 0U || (addr + len) > W25Q16_CAPACITY) {
        return;
    }

    cmd[0] = CMD_READ_DATA;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)addr;

    Cs_Low();
    Spi_Tx(cmd, 4U);
    Spi_Rx(buf, (uint16_t)len);
    Cs_High();
}

uint8_t W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len) {
    uint32_t done = 0;

    if (!s_ready || len == 0U || (addr + len) > W25Q16_CAPACITY) {
        return 1U;
    }

    while (done < len) {
        uint32_t cur = addr + done;
        uint16_t page = (uint16_t)(cur % W25Q_PAGE_SIZE);
        uint16_t chunk = (uint16_t)(W25Q_PAGE_SIZE - page);
        uint16_t want = (uint16_t)(len - done);

        if (chunk > want) {
            chunk = want;
        }

        Write_Enable();

        Cs_Low();
        (void)Spi_TxRx(CMD_PAGE_PROGRAM);
        (void)Spi_TxRx((uint8_t)(cur >> 16));
        (void)Spi_TxRx((uint8_t)(cur >> 8));
        (void)Spi_TxRx((uint8_t)cur);
        Spi_Tx(&buf[done], chunk);
        Cs_High();

        if (Wait_Ready(100U) != 0U) {
            return 2U;
        } /* 写超时 */

        done += chunk;
    }

    return 0U;
}

uint8_t W25Q_EraseSector(uint32_t addr) {
    uint32_t base = addr & ~((uint32_t)W25Q_SECTOR_SIZE - 1U);

    if (!s_ready || base >= W25Q16_CAPACITY) {
        return 1U;
    }

    Write_Enable();

    Cs_Low();
    (void)Spi_TxRx(CMD_SECTOR_ERASE);
    (void)Spi_TxRx((uint8_t)(base >> 16));
    (void)Spi_TxRx((uint8_t)(base >> 8));
    (void)Spi_TxRx((uint8_t)base);
    Cs_High();

    return Wait_Ready(1000U); /* 4KB 扇区擦除典型 45~400ms */
}

uint8_t W25Q_EraseRange(uint32_t addr, uint32_t len) {
    uint32_t a = addr & ~((uint32_t)W25Q_SECTOR_SIZE - 1U);
    uint32_t end = addr + len;

    while (a < end) {
        if (W25Q_EraseSector(a) != 0U) {
            return 1U;
        }
        a += W25Q_SECTOR_SIZE;
    }

    return 0U;
}
