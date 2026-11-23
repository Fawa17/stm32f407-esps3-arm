#pragma once

/* 手机热点 */
#define WIFI_SSID "ESP"
#define WIFI_PASS "12345678"

/* 0 = 向热点要 IP（DHCP）；1 = 用下面 STATIC_* 写死 */
#define USE_STATIC_IP 0
#define STATIC_IP_A 172
#define STATIC_IP_B 19
#define STATIC_IP_C 62
#define STATIC_IP_D 150
#define STATIC_GW_A 172
#define STATIC_GW_B 19
#define STATIC_GW_C 62
#define STATIC_GW_D 85
#define STATIC_DNS_A 172
#define STATIC_DNS_B 19
#define STATIC_DNS_C 62
#define STATIC_DNS_D 85

#define HTTP_HOSTNAME "stm32-arm"
#define FW_TAG "s3-15-rxfast"
#define DEFAULT_MODE 1

/* UDP 广播自己的地址，供电脑端 find_s3 找设备 */
#define DISCOVERY_PORT 9999
#define DISCOVERY_PERIOD_MS 3000

/* 兜底热点：默认不开（热点本身没外网） */
#define WIFI_FALLBACK_AP 0
#define AP_ALWAYS 0
#define AP_SSID "STM32-ARM"
#define AP_PASS "12345678"
#define AP_FALLBACK_FAILS 6

/* 与 STM32 的串口：1 = 板载 TXD0/RXD0(GPIO43/44)，0 = UART1(GPIO17/18) */
#define LINK_ON_UART0 0
#if LINK_ON_UART0
#define LINK_UART UART_NUM_0
#define LINK_TX_PIN 43
#define LINK_RX_PIN 44
#else
#define LINK_UART UART_NUM_1
#define LINK_TX_PIN 17
#define LINK_RX_PIN 18
#endif
#define LINK_BAUD 115200
#define LINK_STALE_US 6000000LL
#define HB_PERIOD_MS 1000
#define HB_RESEND_MS 3000

/* 与 STM32 侧相同的速度，用来估算六轴协同时间 */
#define JOINT_SPEED_DPS 180
#define GRIP_SPEED_DPS 600

/* 夹爪端点：别取满行程，顶到机械限位会堵转发烫 */
#define GRIP_OPEN_DEG 30
#define GRIP_CLOSE_DEG 118

/* 命令确认与重发：300ms 没回音就重发，最多 3 次 */
#define PEND_TIMEOUT_MS 300
#define PEND_MAX_TRIES 3

/* 笛卡尔直线插补的航点间隔 */
#define LINE_STEP_MS 50

/* 给 STM32 刷固件时的分帧参数 */
#define IAP_CHUNK_BYTES 48
#define IAP_U_TIMEOUT_MS 15000
#define IAP_W_TIMEOUT_MS 1000
#define IAP_E_TIMEOUT_MS 5000
#define IAP_TOTAL_TIMEOUT_MS 90000
#define IAP_RECV_MAX_TIMEOUT 4