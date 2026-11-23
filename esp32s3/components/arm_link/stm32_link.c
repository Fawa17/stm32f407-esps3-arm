// stm32_link.c：与 STM32 的串口协议：发送、解析、ACK 重发、心跳

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "app_config.h"
#include "arm_state.h"
#include "arm_bus.h"
#include "stm32_link.h"

static const char *TAG = "arm";

static SemaphoreHandle_t s_tx_lock = NULL;
static volatile int64_t s_last_rx_us = 0;
volatile uint32_t s_cmd_cnt = 0;
volatile uint32_t s_err_cnt = 0;
static char s_pend[192];
static int64_t s_pend_us = 0;
static int s_pend_tries = 0;
static bool s_pend_active = false;
volatile int s_retry_cnt = 0;
volatile int s_retry_fail = 0;
static volatile uint32_t s_ack_seq = 0;
static volatile bool s_ack_ok = false;
volatile bool s_busy_iap = false;
char s_stm32_fw[20];

void link_send(const char *fmt, ...) {
    if (!s_tx_lock) {
        return;
    }
    char line[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(50)) == pdTRUE) {
        uart_write_bytes(LINK_UART, line, strlen(line));
        uart_write_bytes(LINK_UART, "\n", 1);
        xSemaphoreGive(s_tx_lock);
    }
    s_cmd_cnt++;
    ESP_LOGI(TAG, "> %s", line);
}

bool link_ok(void) {
    int64_t last = s_last_rx_us;
    return (last != 0) && ((esp_timer_get_time() - last) < LINK_STALE_US);
}

void link_pend_clear(void) {
    s_pend_active = false;
    s_pend_tries = 0;
}

bool link_wait_ack(int timeout_ms) {
    uint32_t base = s_ack_seq;
    int64_t t0 = esp_timer_get_time();
    while ((esp_timer_get_time() - t0) < (int64_t)timeout_ms * 1000) {
        if (s_ack_seq != base) {
            return s_ack_ok;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

void link_pend_track(const char *line) {
    strncpy(s_pend, line, sizeof(s_pend) - 1);
    s_pend[sizeof(s_pend) - 1] = '\0';
    s_pend_us = esp_timer_get_time();
    s_pend_tries = 1;
    s_pend_active = true;
}

void link_pend_tick(void) {
    if (!s_pend_active) {
        return;
    }
    if ((esp_timer_get_time() - s_pend_us) < (int64_t)PEND_TIMEOUT_MS * 1000) {
        return;
    }
    if (s_pend_tries >= PEND_MAX_TRIES) {
        s_pend_active = false;
        s_pend_tries = 0;
        s_retry_fail++;
        ESP_LOGW(TAG, "命令无应答，放弃重发: %s", s_pend);
        return;
    }
    s_pend_tries++;
    s_pend_us = esp_timer_get_time();
    s_retry_cnt++;
    ESP_LOGW(TAG, "命令无应答，第 %d 次重发: %s", s_pend_tries, s_pend);
    link_send("%s", s_pend);
}

void link_touch(void) {
    s_last_rx_us = esp_timer_get_time();
}

void link_handle_line(char *s) {
    if (strncmp(s, "OK", 2) == 0) {
        s_ack_ok = true;
        s_ack_seq++;
        link_pend_clear();
        link_touch();
        return;
    }
    if (strncmp(s, "ERR", 3) == 0) {
        s_err_cnt++;
        s_ack_ok = false;
        s_ack_seq++;
        link_pend_clear();
        link_touch();
        return;
    }
    if (strncmp(s, "VR,", 3) == 0) /* 版本握手：STM32 固件版本 */ {
        strncpy(s_stm32_fw, s + 3, sizeof(s_stm32_fw) - 1);
        s_stm32_fw[sizeof(s_stm32_fw) - 1] = '\0';
        link_touch();
        return;
    }
    bool is_st = (strncmp(s, "ST,", 3) == 0);
    bool is_st2 = (strncmp(s, "ST2,", 4) == 0);
    /* 十分之一度，比 ST 多一位精度 */

    bool is_hb = (strncmp(s, "HB,", 3) == 0);
    if (!is_st && !is_st2 && !is_hb) {
        return;
    }
    char *p = strchr(s, ',');
    if (!p) {
        return;
    }
    p++;
    int v[8] = {0};
    int cnt = 0;
    while (cnt < 8 && p && *p) {
        v[cnt++] = atoi(p);
        p = strchr(p, ',');
        if (p) {
            p++;
        }
    }
    if (cnt >= 1) {
        s_mode = (v[0] == 1) ? 1 : 0;
    }
    for (int i = 0; i < 6 && (i + 1) < cnt; i++) {
        int a = v[i + 1];
        if (is_st2) {
            a = (a + 5) / 10;
        }
        /* 0.1° → 1° */
        /* ST 里是"线上角"(已镜像)，存回逻辑域，否则下次 jog 会再翻一次 */

        if (i < 5 && (s_mirror & (1 << i))) {
            a = 180 - a;
        }
        if (a < 0) {
            a = 0;
        }
        if (a > 180) {
            a = 180;
        }
        s_angles[i] = a;
        s_model[i] = (float)a;
    }
    link_touch();
}

void link_rx_task(void *arg) {
    (void)arg;
    char chunk[256];
    char line[192];
    size_t len = 0;
    for (;;) {
        size_t avail = 0;
        int n = 0;

        if (uart_get_buffered_data_len(LINK_UART, &avail) == ESP_OK && avail > 0) {
            if (avail > sizeof(chunk)) {
                avail = sizeof(chunk);
            }
            n = uart_read_bytes(LINK_UART, (uint8_t *)chunk, avail, pdMS_TO_TICKS(20));
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        for (int i = 0; i < n; i++) {
            char c = chunk[i];
            if (c == '\r') {
                continue;
            }
            if (c == '\n') {
                line[len] = '\0';
                if (len > 0) {
                    link_handle_line(line);
                }
                len = 0;
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else {
                len = 0;
                /* 超长行丢弃，重新找行首 */
            }
        }
        link_pend_tick();
        /* 没有应答就重发上一条命令 */
    }
}

void link_hb_task(void *arg) {
    (void)arg;
    int64_t last_hb = 0;
    for (;;) {
        int64_t now;
        if (s_busy_iap) {
            /* 正在给 STM32 刷固件：不要打断*/

            vTaskDelay(pdMS_TO_TICKS(HB_PERIOD_MS));
            continue;
        }
        now = esp_timer_get_time();
        if (!link_ok() && (now - last_hb) > (int64_t)HB_RESEND_MS * 1000LL) {
            link_send("H,1");
            link_send("M,%d", s_desired_mode);
            last_hb = now;
        } else if (link_ok() && s_mode != s_desired_mode && (now - last_hb) > 1000LL * 1000LL) {
            /* 链路正常但模式不一致（比如 STM32 刚重启回来）→ 补一条 M 命令 */

            link_send("M,%d", s_desired_mode);
            last_hb = now;
        }
        vTaskDelay(pdMS_TO_TICKS(HB_PERIOD_MS));
    }
}

void link_init(void) {
    s_tx_lock = xSemaphoreCreateMutex();
    uart_config_t uc = {0};
    uc.baud_rate = LINK_BAUD;
    uc.data_bits = UART_DATA_8_BITS;
    uc.parity = UART_PARITY_DISABLE;
    uc.stop_bits = UART_STOP_BITS_1;
    uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uc.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &uc));
    ESP_ERROR_CHECK(
        uart_set_pin(LINK_UART, LINK_TX_PIN, LINK_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#if LINK_ON_UART0
    esp_log_level_set("*", ESP_LOG_NONE);
#else
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 4096, 2048, 0, NULL, 0));
#endif
    xTaskCreate(link_rx_task, "link_rx", 4096, NULL, 10, NULL);
    xTaskCreate(link_hb_task, "link_hb", 3072, NULL, 5, NULL);
}
