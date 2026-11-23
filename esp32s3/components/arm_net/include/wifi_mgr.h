#pragma once

#include <stdbool.h>
#include <stdint.h>

const char *cur_ip_str(void);
void wifi_start(void);
void discovery_task(void *arg);
#if WIFI_FALLBACK_AP
void start_ap_fallback(void);
#endif
