#pragma once

#include <stdbool.h>

/* arm_link 对外提供的少量函数，单独放一层，避免 arm_state 与 arm_link 互相依赖 */
void link_send(const char *fmt, ...);
bool link_ok(void);
void link_touch(void);
bool link_wait_ack(int timeout_ms);
void link_pend_track(const char *line);