/*
 * NetLink.c - 和 ESP32-S3 的串口链路
 *
 * USART1 + DMA2_Stream2 循环接收，按行解析；中断里只搬数据，字符串处理都在
 * NetLink_Poll() 里做，不占用中断时间。
 */
#include "NetLink.h"
#include "usart.h"
#include "RobotArm.h"
#include "MPU6050.h"
#include "IAP.h"
#include "Param.h"
#include <string.h>

/* ===================== 内部状态 ===================== */
static uint8_t s_rx_buf[NET_RX_BUF_SIZE]; /* DMA 环形缓冲 */
static uint16_t s_rd;                     /* 解析读指针 */

static char s_line[NET_LINE_MAX];
static uint16_t s_len;

static uint8_t s_mode = NET_MODE_GESTURE;
static uint32_t s_last_cmd_ms;
static uint8_t s_hb_en;
static uint32_t s_last_hb_ms;

static uint32_t s_cmd_cnt;
static uint32_t s_err_cnt;

/* 固件版本号现在定义在 NetLink.h（Backup.c 也要用），这里不再重复定义 */

/* 发送数据，阻塞式，因为发的数据量不大；huart1 由 CubeMX 的 MX_USART1_UART_Init() 初始化 */
static void Net_Send(const char *s) {
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), NET_TX_TIMEOUT_MS);
}

/* 无符号整数转字符串（避免引入 printf，省 Flash） */
static uint16_t Net_U16ToStr(char *dst, uint16_t v) {
    char tmp[6];
    uint8_t n = 0;
    uint8_t i;

    if (v == 0U) {
        dst[0] = '0';
        return 1U;
    }
    while (v > 0U && n < (uint8_t)sizeof(tmp)) {
        tmp[n++] = (char)('0' + (uint8_t)(v % 10U));
        v /= 10U;
    }
    for (i = 0; i < n; i++) {
        dst[i] = tmp[n - 1U - i];
    }
    return (uint16_t)n;
}

/* 发送 OK\n */
static void Net_Ok(void) {
    s_last_cmd_ms = HAL_GetTick();
    s_cmd_cnt++;
    Net_Send("OK\n");
}

/* 发送 ERR\n */
static void Net_Err(void) {
    s_err_cnt++;
    Net_Send("ERR\n");
}
/** 发送状态帧给上位机（ESP8266） 拼装后的数据格式示例： */
/** 解析一个"度"值，允许小数（最多 3 位） 为了 0.1° 级直线插补：P 命令里写 90.5 就真的是 90.5°， */
static uint8_t Net_ParseDeg(const char **pp, float max, float *out) {
    const char *p = *pp;
    float v = 0.0f;
    float frac = 0.0f;
    float scale = 0.1f;
    uint8_t d = 0;
    uint8_t f = 0;

    while (*p == ' ')
        p++;

    while (*p >= '0' && *p <= '9') {
        v = v * 10.0f + (float)(*p - '0');
        if (v > 1000.0f)
            return 0U;
        p++;
        d++;
    }

    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            if (f < 3U) {
                frac += (float)(*p - '0') * scale;
                scale *= 0.1f;
                f++;
            }
            p++;
        }
    }

    if (d == 0U && f == 0U)
        return 0U;

    v += frac;
    if (v > max)
        return 0U;

    *out = v;
    *pp = p;
    return 1U;
}

/** 发一帧 0.1° 精度的状态：ST2,<mode>,<a0x10>,...,<a5x10>,<alive>
 * 用整数发"十分之一度"，避免为了打印 float 拖进几 KB 的 printf。 */
static void Net_SendStatus10(uint8_t with_alive) {
    static const char head[] = "ST2,";
    char buf[64];
    uint16_t n = 0;
    uint8_t i;

    for (i = 0; head[i] != '\0'; i++)
        buf[n++] = head[i];
    n += Net_U16ToStr(&buf[n], s_mode);

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        float a = RobotArm_GetAngleF(i) * 10.0f;
        buf[n++] = ',';
        n += Net_U16ToStr(&buf[n], (uint16_t)(a + 0.5f));
    }

    if (with_alive) {
        buf[n++] = ',';
        n += Net_U16ToStr(&buf[n], (uint16_t)(NetLink_LinkAlive() ? 1U : 0U));
    }
    buf[n++] = '\n';
    buf[n] = '\0';
    Net_Send(buf);
}

static void Net_SendStatus(const char *head, uint8_t with_alive) {
    char buf[64];
    uint16_t n = 0;
    uint8_t i;

    while (*head != '\0' && n < 8U) {
        buf[n++] = *head++;
    }
    buf[n++] = ',';
    n += Net_U16ToStr(&buf[n], s_mode);

    for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
        buf[n++] = ',';
        n += Net_U16ToStr(&buf[n], RobotArm_GetAngle(i));
    }

    if (with_alive) {
        buf[n++] = ',';
        n += Net_U16ToStr(&buf[n], NetLink_LinkAlive());
    }

    buf[n++] = '\n';
    buf[n] = '\0';
    Net_Send(buf);
}

/** 解析无符号16位整数 解析一个 0~max 的十进制数，后面不跟分隔符时也能停 */
static uint8_t Net_ParseU16(const char **pp, uint16_t max, uint16_t *out) {
    const char *p = *pp;
    uint32_t v = 0;
    uint8_t d = 0;

    while (*p == ' ')
        p++;

    while (*p >= '0' && *p <= '9') {
        v = v * 10U + (uint32_t)(*p - '0');
        if (v > 65535U)
            return 0U;
        p++;
        d++;
    }
    if (d == 0U || v > max)
        return 0U;

    *out = (uint16_t)v;
    *pp = p;
    return 1U;
}

/** 解析标志位（0或1） M,0|1、G,0|1、H,0|1 这种带逗号前缀，且值只能是 0 或 1 的指令 */
static uint8_t Net_ParseFlag(const char **pp, uint16_t *out) {
    const char *p = *pp;

    if (*p != ',')
        return 0U;
    p++;
    if (!Net_ParseU16(&p, 1U, out))
        return 0U;

    *pp = p;
    return 1U;
}

/**
 * @brief 解析无符号32位整数（IAP 的 size / offset / crc 用得到）
 */
static uint8_t Net_ParseU32(const char **pp, uint32_t max, uint32_t *out) {
    const char *p = *pp;
    uint32_t v = 0;
    uint8_t d = 0;

    while (*p == ' ')
        p++;

    while (*p >= '0' && *p <= '9') {
        uint32_t nv = v * 10U + (uint32_t)(*p - '0');
        if (nv < v)
            return 0U; /* 溢出 */
        v = nv;
        p++;
        d++;
        if (d > 10U)
            return 0U;
    }
    if (d == 0U || v > max)
        return 0U;

    *out = v;
    *pp = p;
    return 1U;
}

/** 解析十六进制字符串（两字符 = 一字节） */
static uint8_t Net_ParseHex(const char **pp, uint8_t *out, uint8_t max, uint8_t *outlen) {
    const char *p = *pp;
    uint8_t n = 0;

    while (n < max) {
        uint8_t hi, lo;
        char c1 = p[0];
        char c2 = p[1];

        if (c1 >= '0' && c1 <= '9')
            hi = (uint8_t)(c1 - '0');
        else if (c1 >= 'a' && c1 <= 'f')
            hi = (uint8_t)(c1 - 'a' + 10);
        else if (c1 >= 'A' && c1 <= 'F')
            hi = (uint8_t)(c1 - 'A' + 10);
        else
            break;

        if (c2 >= '0' && c2 <= '9')
            lo = (uint8_t)(c2 - '0');
        else if (c2 >= 'a' && c2 <= 'f')
            lo = (uint8_t)(c2 - 'a' + 10);
        else if (c2 >= 'A' && c2 <= 'F')
            lo = (uint8_t)(c2 - 'A' + 10);
        else
            break;

        out[n++] = (uint8_t)((hi << 4) | lo);
        p += 2;
    }

    if (n == 0U)
        return 0U;

    *pp = p;
    *outlen = n;
    return 1U;
}

/** 处理接收到的每一行数据 */
static void Net_HandleLine(char *line) {
    const char *p = line;
    float fdeg[ROBOTARM_AXIS_NUM];  // 目标角度（浮点，P 命令支持小数）
    uint16_t v;                     // 解析结果
    uint8_t i;                      // 循环变量
    char cmd;                       // 命令字母
    uint32_t u32;                   // IAP 的 size / offset
    uint32_t crc;                   // IAP 的 CRC32
    uint8_t chunk[48];              // IAP 一帧数据（hex 解码后 ≤48 字节）
    uint8_t n;                      // hex 解出的字节数

    while (*p == ' ')
        p++;
    if (*p == '#')
        p++; /* 行首 '#' 可有可无，方便人眼/日志区分 */
    if (*p == '\0')
        return;

    cmd = *p++;  // 取命令字母
    if (cmd >= 'a' && cmd <= 'z')
        cmd = (char)(cmd - 'a' + 'A');  // 转大写，方便上位机大小写都能识别

    /* 升级期间（U 之后到 E 结束）只认 U/W/E：
       其它命令一律回 ERR，绝不能让机械臂在写 Flash 的时候被指令带走，
       也不能让一次误发的 P 打乱升级节奏。 */
    if (IAP_Active() != 0U) {
        if (cmd == 'U' || cmd == 'W' || cmd == 'E') {
            IAP_Touch();
        } else {
            Net_Err();
            return;
        }
    }

    // 开始解析参数
    switch (cmd) {
    case 'P': /* P,a0..a5[,ms]，a 允许小数（如 90.5） */
        for (i = 0; i < ROBOTARM_AXIS_NUM; i++) {
            if (*p != ',') {
                Net_Err();
                return;
            }
            p++;
            if (!Net_ParseDeg(&p, (float)ROBOTARM_MAX_ANGLE, &fdeg[i])) {
                Net_Err();
                return;
            }
        }
        if (*p == ',') {  // 有指定动作需要的时间
            p++;
            if (!Net_ParseU16(&p, 60000U, &v)) {
                Net_Err();
                return;
            }
            if (v > 0U)
                RobotArm_SetTargetsTimedF(fdeg, v);
            else
                RobotArm_SetTargetsF(fdeg);
        } else {
            RobotArm_SetTargetsF(fdeg);
        }
        Net_Ok();
        break;

    case 'M': /* M,0|1 */
        if (!Net_ParseFlag(&p, &v)) {
            Net_Err();
            return;
        }
        NetLink_SetMode((uint8_t)v);
        Net_Ok();
        Net_SendStatus("ST", 1U); /* 顺带回一帧，上位机好确认模式 */
        Net_SendStatus10(0U);     /* 0.1° 精度版本（ST2），上位机要细就用它 */
        break;

    case 'G': /* G,0|1  夹爪 张开/闭合 */
        if (!Net_ParseFlag(&p, &v)) {
            Net_Err();
            return;
        }
        RobotArm_SetTarget(ARM_GRIPPER, (v == 0U) ? (uint8_t)GRIP_OPEN : (uint8_t)GRIP_CLOSE);
        Net_Ok();
        break;

    case 'C': { /* C,<axis>,<tmin>,<tmax>  单轴两点标定 */
        uint32_t ax, tmin, tmax;

        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseU32(&p, 5U, &ax)) {
            Net_Err();
            return;
        }
        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseU32(&p, 40000U, &tmin)) {
            Net_Err();
            return;
        }
        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseU32(&p, 40000U, &tmax)) {
            Net_Err();
            return;
        }

        RobotArm_SetCalib((uint8_t)ax, tmin, tmax);
        (void)Param_Save(); /* 顺手写进 W25Q16，掉电不丢 */
        Net_Ok();
        break;
    }

    case 'S': /* 急停：保持当前角度 */
        RobotArm_Stop();
        Net_Ok();
        break;

    case 'Q': /* 查询状态 */
        s_last_cmd_ms = HAL_GetTick();
        s_cmd_cnt++;
        Net_Send("VR," NET_FW_VERSION "\n"); /* 版本握手：上位机据此确认两端固件 */
        Net_SendStatus("ST", 1U);
        Net_SendStatus10(0U); /* 追加 0.1° 精度帧 */
        break;

    case 'H': /* H,0|1  周期回传开关 */
        if (!Net_ParseFlag(&p, &v)) {
            Net_Err();
            return;
        }
        s_hb_en = (uint8_t)v;
        s_last_hb_ms = HAL_GetTick();
        Net_Ok();
        break;

    case 'U': /* U,<size>,<crc>  开始升级：擦暂存区与元数据区 */
        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseU32(&p, IAP_STAGE_MAX, &u32)) {
            Net_Err();
            return;
        }
        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseU32(&p, 0xFFFFFFFFUL, &crc)) {
            Net_Err();
            return;
        }
        if (IAP_Begin(u32, crc) != 0U) {
            Net_Ok(); /* 擦除要 1~2 秒，上位机要给足超时 */
        } else {
            Net_Err();
        }
        break;

    case 'W': /* W,<offset>,<hex>  写一段数据（offset 必须严格递增） */
        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseU32(&p, IAP_STAGE_MAX, &u32)) {
            Net_Err();
            return;
        }
        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseHex(&p, chunk, (uint8_t)sizeof(chunk), &n)) {
            Net_Err();
            return;
        }
        if (IAP_Write(u32, chunk, n) != 0U) {
            Net_Ok();
        } else {
            Net_Err();
        }
        break;

    case 'E': /* E,<crc>  结束：校验通过后写元数据并重启 */
        if (*p != ',') {
            Net_Err();
            return;
        }
        p++;
        if (!Net_ParseU32(&p, 0xFFFFFFFFUL, &crc)) {
            Net_Err();
            return;
        }
        if (IAP_End(crc) != 0U) {
            Net_Send("OK\n");
            HAL_Delay(30); /* 等串口把 OK 发完再复位 */
            IAP_Reboot();
        } else {
            Net_Err();
        }
        break;

    default:
        Net_Err();
        break;
    }
}

/** 初始化网络链接（USART1 异步 115200 + DMA2_Stream2 循环接收） F407 版：USART1 在 APB2，RX 对应
 * DMA2_Stream2 通道 4。 */
void NetLink_Init(void) {
    GPIO_InitTypeDef gpio = {0};

    /* ---------- 1. PA10 上拉：ESP 没插/没上电时不会收到一堆噪声字节 ---------- */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---------- 2. DMA2_Stream2 / Channel4：USART1_RX → s_rx_buf ----------
     * 字节宽、内存地址递增、循环模式，直接写寄存器，不依赖 CubeMX 的 DMA 配置 */
    __HAL_RCC_DMA2_CLK_ENABLE();

    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    while ((DMA2_Stream2->CR & DMA_SxCR_EN) != 0U) {
        /* 等流真正关闭才能改配置 */
    }
    DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 |
                  DMA_LIFCR_CTCIF2;

    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream2->M0AR = (uint32_t)s_rx_buf;
    DMA2_Stream2->NDTR = NET_RX_BUF_SIZE;
    DMA2_Stream2->CR = (4U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_PL_1;

    /* 打开 USART1 的 DMA 接收请求 */
    USART1->CR3 |= USART_CR3_DMAR;

    DMA2_Stream2->CR |= DMA_SxCR_EN;

    /* 清一次可能残留的接收标志 */
    (void)USART1->SR;
    (void)USART1->DR;

    s_rd = 0;
    s_len = 0;
    s_mode = NET_MODE_GESTURE;
    s_hb_en = 0;
    s_cmd_cnt = 0;
    s_err_cnt = 0;
    s_last_cmd_ms = HAL_GetTick();
    s_last_hb_ms = HAL_GetTick();
}

/**
 * @brief 清空接收缓冲区
 */
void NetLink_Flush(void) {
    s_rd = (uint16_t)(NET_RX_BUF_SIZE - DMA2_Stream2->NDTR);
    if (s_rd >= NET_RX_BUF_SIZE)
        s_rd = 0;
    s_len = 0;
}

/**
 * @brief 主循环的非阻塞轮询：把 DMA 收到的字节解析成命令并执行
 */
void NetLink_Poll(void) {
    uint16_t pos = (uint16_t)(NET_RX_BUF_SIZE - DMA2_Stream2->NDTR);  // DMA 写指针位置
    uint8_t c;

    IAP_Tick(); /* 升级没人喂了（WiFi 掉线/网页关掉）就自动退出维护模式 */

    if (pos >= NET_RX_BUF_SIZE)
        pos = NET_RX_BUF_SIZE - 1U;
    // 判断读写指针是否相等，若不相等则说明有新数据到来，需要处理
    while (s_rd != pos) {
        c = s_rx_buf[s_rd];
        s_rd++;
        if (s_rd >= NET_RX_BUF_SIZE)
            s_rd = 0;  // 环形：读到末尾就回到开头
        // 收到换行符就认为是一行数据结束，开始解析
        if (c == '\n') {
            if (s_len > 0U) {
                s_line[s_len] = '\0';
                Net_HandleLine(s_line);
            }
            s_len = 0;
        }
        // 收到回车符就忽略，避免上位机发送 CRLF 时多解析一次
        else if (c != '\r') {
            if (s_len < (NET_LINE_MAX - 1U)) {
                s_line[s_len++] = (char)c;
            } else {
                s_len = 0; /* 超长行：整行丢弃，等下一个换行重新同步 */
                s_err_cnt++;
            }
        }
    }

    /* 周期状态回传（H,1 开启） */
    if (s_hb_en) {
        uint32_t now = HAL_GetTick();
        if ((now - s_last_hb_ms) >= NET_HEARTBEAT_MS) {
            s_last_hb_ms = now;
            Net_SendStatus("HB", 0U);
            Net_SendStatus10(0U); /* 顺带一帧 0.1° 精度状态 */
        }
    }
}

/** 获取当前模式 */
uint8_t NetLink_GetMode(void) {
    return s_mode;
}
/** 设置当前模式 */
void NetLink_SetMode(uint8_t mode) {
    s_mode = (mode == NET_MODE_REMOTE) ? (uint8_t)NET_MODE_REMOTE : (uint8_t)NET_MODE_GESTURE;
    s_last_cmd_ms = HAL_GetTick();
}
/** 检查链接是否存活 */
uint8_t NetLink_LinkAlive(void) {
    return ((uint32_t)(HAL_GetTick() - s_last_cmd_ms) < NET_LINK_TIMEOUT_MS) ? 1U : 0U;
}

/** 获取命令计数 */
uint32_t NetLink_GetCmdCount(void) {
    return s_cmd_cnt;
}

/** 获取错误计数 */
uint32_t NetLink_GetErrCount(void) {
    return s_err_cnt;
}
