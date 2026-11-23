/** ***************************************************************************** SSD1306 128x64
 * OLED 状态监视器（硬件 I2C1） 硬件：PB6 = I2C1_SCL，PB7 = I2C1_SDA（CubeMX 配成 AF4 开漏，400kHz）
 */
#include "OLED.h"
#include "oled_font.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "MPU6050.h"
#include "NetLink.h"
#include "RobotArm.h"

#define OLED_TIMEOUT_MS 20U /* 单次 I2C 传输超时 */
#define OLED_FAIL_LIMIT 5U  /* 连续失败这么多次就认为屏掉了 */

/* 0.96 寸 128x64 模块的两种常见 I2C 地址（7 位 0x3C / 0x3D，HAL 用左移后的值） */
#define OLED_ADDR_1 0x78U
#define OLED_ADDR_2 0x7AU

/* 屏幕整幅右移 2 像素、右边有 2 列花屏 → 这块是 SH1106 控制器，把这里改成 1 */
#define OLED_SH1106 0
#define OLED_COL_OFFSET (OLED_SH1106 ? 2U : 0U)

static uint16_t s_addr = OLED_ADDR_1;           /* 实际应答的从机地址 */
static uint8_t s_fb[OLED_LINE_NUM][OLED_WIDTH]; /* 屏上当前内容：1 = 亮点 */
static uint16_t s_dirty;                        /* bit n = 第 n 行需要重发 */
static uint8_t s_ready = 0;
static uint8_t s_failCnt = 0;

/* ============================ 底层：I2C1 + SSD1306 ============================ */

static uint8_t OLED_WriteCmds(const uint8_t *cmds, uint16_t len) {
    /* 控制字节 0x00 = 后面都是命令 */
    if (HAL_I2C_Mem_Write(&hi2c1, s_addr, 0x00U, I2C_MEMADD_SIZE_8BIT, (uint8_t *)cmds, len,
                          OLED_TIMEOUT_MS) != HAL_OK) {
        return 1;
    }
    return 0;
}

static uint8_t OLED_WritePage(uint8_t page, const uint8_t *data) {
    uint16_t col = OLED_COL_OFFSET;
    uint8_t cmd[3];

    cmd[0] = (uint8_t)(0xB0U | page);                 /* 页地址 */
    cmd[1] = (uint8_t)(col & 0x0FU);                  /* 列地址低 4 位 */
    cmd[2] = (uint8_t)(0x10U | ((col >> 4) & 0x0FU)); /* 列地址高 4 位 */
    if (OLED_WriteCmds(cmd, 3U) != 0U)
        return 1U;

    /* 控制字节 0x40 = 后面都是显存数据 */
    if (HAL_I2C_Mem_Write(&hi2c1, s_addr, 0x40U, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, OLED_WIDTH,
                          OLED_TIMEOUT_MS) != HAL_OK) {
        return 1U;
    }
    return 0U;
}

/* ============================ 对外接口 ============================ */

uint8_t OLED_Init(void) {
    static const uint8_t init_cmds[] = {
        0xAE,       /* 关显示 */
        0xD5, 0x80, /* 时钟分频 */
        0xA8, 0x3F, /* 复用率 1/64 */
        0xD3, 0x00, /* 显示偏移 0 */
        0x40,       /* 起始行 0 */
#if OLED_SH1106
        0xAD, 0x8B, /* SH1106：开 DC-DC */
#else
        0x8D, 0x14, /* SSD1306：开电荷泵 */
        0x20, 0x00, /* 水平寻址模式（SH1106 不支持，用页寻址） */
#endif
        0xA1,       /* 段重映射：左右正常 */
        0xC8,       /* COM 扫描方向：上下正常 */
        0xDA, 0x12, /* COM 引脚配置 */
        0x81, 0xCF, /* 对比度 */
        0xD9, 0xF1, /* 预充电周期 */
        0xDB, 0x40, /* VCOMH */
        0xA4,       /* 整体显示跟随显存 */
        0xA6,       /* 正常显示（非反色） */
        0xAF        /* 开显示 */
    };

    /* 0.96 寸模块多数是 0x3C(0x78)，少数是 0x3D(0x7A)：两个都试一下 */
    s_addr = OLED_ADDR_1;
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_1, 3U, OLED_TIMEOUT_MS) != HAL_OK) {
        s_addr = OLED_ADDR_2;
        if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_2, 3U, OLED_TIMEOUT_MS) != HAL_OK) {
            s_ready = 0;
            return 1U;
        }
    }

    if (OLED_WriteCmds(init_cmds, (uint16_t)sizeof(init_cmds)) != 0U) {
        s_ready = 0;
        return 1U;
    }

    s_failCnt = 0;
    s_ready = 1;

    OLED_Clear(); /* 清屏会把 8 行都标脏 */
    OLED_Update();

    return 0U;
}

uint8_t OLED_IsReady(void) {
    return s_ready;
}

void OLED_Clear(void) {
    memset(s_fb, 0x00, sizeof(s_fb));
    s_dirty = 0xFFFFU; /* 整屏都要重发 */
}

void OLED_Update(void) {
    uint8_t page;

    if (!s_ready)
        return;

    for (page = 0; page < OLED_LINE_NUM; page++) {
        if ((s_dirty & (1U << page)) == 0U) {
            continue; /* 这一行没变，省一次传输 */
        }
        if (OLED_WritePage(page, s_fb[page]) != 0U) {
            if (++s_failCnt >= OLED_FAIL_LIMIT) {
                s_ready = 0; /* 连续失败：认为屏被拔了 */
            }
            return;
        }
        s_dirty &= (uint16_t)~(1U << page);
    }
    s_failCnt = 0;
}

void OLED_Text(uint8_t line, const char *fmt, ...) {
    char buf[OLED_COL_NUM + 1];
    uint8_t linebuf[OLED_WIDTH];
    va_list ap;
    uint8_t i;

    if (line >= OLED_LINE_NUM)
        return;

    memset(buf, 0, sizeof(buf));
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    memset(linebuf, 0x00, OLED_WIDTH);

    for (i = 0; i < OLED_COL_NUM; i++) {
        uint8_t ch = (uint8_t)buf[i];
        uint8_t base = 6U * i;
        uint8_t k;

        if (ch == 0U)
            break;
        if (ch < 0x20U || ch > 0x7EU)
            ch = (uint8_t)'?';

        for (k = 0; k < 5U; k++) {
            /* 字库 bit0 是最上面一行，正好对上显存每页的行序 */
            linebuf[base + k] = s_font[ch - 0x20U][k];
        }
        linebuf[base + 5U] = 0x00; /* 字间距 1 列 */
    }

    if (memcmp(linebuf, s_fb[line], OLED_WIDTH) != 0) {
        memcpy(s_fb[line], linebuf, OLED_WIDTH); /* 只有内容变了才重发这一行 */
        s_dirty |= (uint16_t)(1U << line);
    }
}

/* ============================ 状态监视画面 ============================ */

/* 把浮点转成“整数第十度”，避免依赖 printf 的浮点支持 */
static int16_t OLED_F10(float v) {
    float t = v * 10.0f;

    if (t > 32767.0f)
        t = 32767.0f;
    if (t < -32767.0f)
        t = -32767.0f;

    return (int16_t)(t >= 0.0f ? (t + 0.5f) : (t - 0.5f));
}

/* 打印 "+12.3" / "-4.5" 这样的定点数 */
static void OLED_TextF10(uint8_t line, const char *label, float v) {
    int16_t t = OLED_F10(v);
    int16_t a = (int16_t)(t >= 0 ? t : -t);

    OLED_Text(line, "%s%s%d.%d", label, (t < 0) ? "-" : "+", a / 10, a % 10);
}

void OLED_StatusTask(void) {
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    uint8_t mode = NetLink_GetMode();
    uint8_t alive = NetLink_LinkAlive();

    if (!s_ready)
        return;

    /* 只用缓存值，绝不在这个任务里读 I2C2，避免和 gestureTask 抢总线 */
    MPU6050_GetCachedAttitude(&pitch, &roll, &yaw);

    /* 不调 OLED_Clear()：OLED_Text 会逐行比对，内容没变就不重发 */
    OLED_Text(0, "ARM  LINK:%s", alive ? "OK " : "---");
    OLED_Text(1, "MODE:%s", mode ? "REMOTE" : "GESTURE");

    OLED_Text(2, "B%3u S%3u E%3u", (unsigned)RobotArm_GetAngle(ARM_BASE),
              (unsigned)RobotArm_GetAngle(ARM_SHOULDER), (unsigned)RobotArm_GetAngle(ARM_ELBOW));

    OLED_Text(3, "P%3u R%3u G%3u", (unsigned)RobotArm_GetAngle(ARM_WRIST_PITCH),
              (unsigned)RobotArm_GetAngle(ARM_WRIST_ROLL),
              (unsigned)RobotArm_GetAngle(ARM_GRIPPER));

    /* 第 4、5 行显示"实际下发的脉宽(µs)"：中位应该是 1500 1500 1500。
     * 这是排查"上电没回中位"的关键证据 —— 屏上是 1500 但舵机不动，就是硬件侧。 */
    OLED_Text(4, "us%4u%5u%5u", (unsigned)RobotArm_GetPulseUs(ARM_BASE),
              (unsigned)RobotArm_GetPulseUs(ARM_SHOULDER),
              (unsigned)RobotArm_GetPulseUs(ARM_ELBOW));
    OLED_Text(5, "us%4u%5u%5u", (unsigned)RobotArm_GetPulseUs(ARM_WRIST_PITCH),
              (unsigned)RobotArm_GetPulseUs(ARM_WRIST_ROLL),
              (unsigned)RobotArm_GetPulseUs(ARM_GRIPPER));

    OLED_TextF10(6, "pitch:", pitch);

    OLED_Text(7, "CMD:%lu ERR:%lu", (unsigned long)NetLink_GetCmdCount(),
              (unsigned long)NetLink_GetErrCount());

    OLED_Update();
}
