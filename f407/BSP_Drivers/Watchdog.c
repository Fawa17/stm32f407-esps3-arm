/*
 * Watchdog.c - 独立看门狗 + 任务心跳
 *
 * 超时 = RLR(3000) * 预分频(64) / 32000 ≈ 6s；LSI 在 17~47kHz 之间飘，实际 4~11s。
 * 留这么长是因为 IAP 擦 128KB 扇区要 1~2 秒，擦除期间喂不了狗。
 * 直接写寄存器，不依赖 CubeMX 生成的 hiwdg，重新生成工程也不会丢。
 */

#include "Watchdog.h"
#include "main.h" /* 提供 IWDG 寄存器定义（stm32f4xx.h） */
#include "cmsis_os.h"

#if WATCHDOG_ENABLE

/* 各任务允许的最大“沉默时间”（ms），一般取 3~8 倍任务周期 */
static const uint32_t s_deadline_ms[WD_TASK_NUM] = {
    200U, /* armTask     ：10ms  周期 → 200ms 很宽松 */
    200U, /* gestureTask ：20ms  周期 */
    200U, /* netTask     ：5ms   周期 */
    800U, /* oledTask    ：100ms 周期，且优先级最低，给足余量 */
};

static uint32_t s_last_ms[WD_TASK_NUM];
static volatile uint8_t s_started;

void Watchdog_Start(void) {
    if (s_started) {
        return;
    }
    s_started = 1U;

    /* 1. 解除 PR/RLR 的写保护 */
    IWDG->KR = 0x5555U;
    /* 2. 预分频 64（LSI/64） */
    IWDG->PR = 0x04U;
    /* 3. 重装值 1500 ≈ 3s */
    IWDG->RLR = 3000U;
    /* 4. 先喂一次，把计数器装载好 */
    IWDG->KR = 0xAAAAU;
    /* 5. 启动 */
    IWDG->KR = 0xCCCCU;

    /* 心跳计时从现在开始算，避免刚启动就被判超期 */
    for (uint8_t i = 0; i < WD_TASK_NUM; i++) {
        s_last_ms[i] = osKernelGetTickCount();
    }
}

void Watchdog_KeepAlive(uint8_t id) {
    if (!s_started || id >= WD_TASK_NUM) {
        return;
    }
    s_last_ms[id] = osKernelGetTickCount();
}

uint16_t Watchdog_OverdueMask(void) {
    uint16_t mask = 0U;
    uint32_t now = osKernelGetTickCount();

    if (!s_started) {
        return 0U;
    }

    for (uint8_t i = 0; i < WD_TASK_NUM; i++) {
        /* tick 是 32 位、1000Hz，做减法后强转有符号即可安全处理回绕 */
        if ((int32_t)(now - s_last_ms[i]) > (int32_t)s_deadline_ms[i]) {
            mask |= (uint16_t)(1U << i);
        }
    }
    return mask;
}

uint8_t Watchdog_AllAlive(void) {
    return (Watchdog_OverdueMask() == 0U) ? 1U : 0U;
}

void Watchdog_Feed(void) {
    if (!s_started) {
        return;
    }
    IWDG->KR = 0xAAAAU;
}

#else /* !WATCHDOG_ENABLE -------------------------------------------------*/

void Watchdog_Start(void) {
}
void Watchdog_KeepAlive(uint8_t id) {
    (void)id;
}
uint8_t Watchdog_AllAlive(void) {
    return 1U;
}
void Watchdog_Feed(void) {
}
uint16_t Watchdog_OverdueMask(void) {
    return 0U;
}

#endif /* WATCHDOG_ENABLE */
