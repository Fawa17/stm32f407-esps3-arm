#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 跨模块共享状态 */
extern volatile uint32_t s_cmd_cnt;
extern volatile uint32_t s_err_cnt;
extern volatile int s_retry_cnt;
extern volatile int s_retry_fail;
extern volatile bool s_busy_iap;
extern char s_stm32_fw[20];

void link_init(void);
void link_send(const char *fmt, ...);
bool link_ok(void);
void link_touch(void);
bool link_wait_ack(int timeout_ms);
void link_pend_track(const char *line);
void link_handle_line(char *s);
void link_rx_task(void *arg);
void link_hb_task(void *arg);
