// wifi_mgr.c：WiFi 连接、AP 兜底、设备发现广播

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "app_config.h"
#include "stm32_link.h"
#include "wifi_mgr.h"

static const char *TAG = "arm";

static int s_wifi_fail = 0;
#if WIFI_FALLBACK_AP
static bool s_ap_on = false;
#endif

#if WIFI_FALLBACK_AP
void start_ap_fallback(void) {
    if (s_ap_on) {
        return;
    }
    s_ap_on = true;
    wifi_config_t apc = {0};
    strncpy((char *)apc.ap.ssid, AP_SSID, sizeof(apc.ap.ssid) - 1);
    apc.ap.ssid_len = strlen(AP_SSID);
    strncpy((char *)apc.ap.password, AP_PASS, sizeof(apc.ap.password) - 1);
    apc.ap.max_connection = 4;
    apc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &apc);
    ESP_LOGW(TAG, "连不上手机热点，已开启兜底热点：");
    ESP_LOGW(TAG, "    WiFi : %s    密码: %s", AP_SSID, AP_PASS);
    ESP_LOGW(TAG, "    网页 : http://192.168.4.1/");
}
#endif

const char *wifi_reason_str(int r) {
    switch (r) {
    case 1:
        return "未指定";
    case 2:
        return "认证过期";
    case 3:
        return "AP 主动踢开";
    case 4:
        return "关联超时";
    case 5:
        return "AP 连接的设备太多";
    case 6:
        return "未认证";
    case 7:
        return "未关联";
    case 8:
        return "取消关联";
    case 15:
        return "四次握手超时（密码错？）";
    case 200:
        return "ACK/beacon 超时（信号差）";
    case 201:
        return "扫不到这个 SSID（名字错/5GHz/热点关了）";
    case 202:
        return "认证失败";
    case 203:
        return "关联失败";
    case 204:
        return "握手超时";
    case 205:
        return "连接失败";
    default:
        return "其他";
    }
}

void wifi_scan_report(void) {
    wifi_scan_config_t sc = {0};
    sc.show_hidden = true;
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) {
        ESP_LOGW(TAG, "WiFi 扫描失败");
        return;
    }
    uint16_t total = 0;
    esp_wifi_scan_get_ap_num(&total);
    wifi_ap_record_t rec[12];
    uint16_t n = 12;
    if (esp_wifi_scan_get_ap_records(&n, rec) != ESP_OK) {
        return;
    }
    ESP_LOGW(TAG, "可见 WiFi 共 %u 个：", (unsigned)total);
    bool found = false;
    for (uint16_t i = 0; i < n; i++) {
        ESP_LOGW(TAG, "  [%s]  信道 %-3d  %d dBm  加密 %d", (const char *)rec[i].ssid,
                 rec[i].primary, rec[i].rssi, (int)rec[i].authmode);
        if (strcmp((const char *)rec[i].ssid, WIFI_SSID) == 0) {
            found = true;
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "扫不到 \"%s\"。三种最常见原因：", WIFI_SSID);
        ESP_LOGE(TAG, "  1) 手机热点开在 5GHz —— ESP32-S3 只支持 2.4GHz，"
                      "去热点设置里把「频段」改成 2.4GHz");
        ESP_LOGE(TAG, "  2) SSID 名字/大小写/前后空格和固件里的不一致");
        ESP_LOGE(TAG, "  3) 热点此刻没开、或手机热点名被改过");
    }
}

void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)data;
        s_wifi_fail++;
        ESP_LOGW(TAG, "WiFi 断开 reason=%d（%s）第 %d 次，重连中...", ev->reason,
                 wifi_reason_str(ev->reason), s_wifi_fail);
        if (s_wifi_fail == 3) {
            wifi_scan_report();
            /* 只扫一次，避免刷屏 */
        }
#if WIFI_FALLBACK_AP
        if (s_wifi_fail >= AP_FALLBACK_FAILS) {
            start_ap_fallback();
            /* 再连不上就自己开热点，保证有办法进去 */
        }
#endif
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        s_wifi_fail = 0;
        ESP_LOGI(TAG, "WiFi OK  IP=" IPSTR "  网页: http://" IPSTR "/", IP2STR(&ev->ip_info.ip),
                 IP2STR(&ev->ip_info.ip));
    }
}

const char *cur_ip_str(void) {
    static char buf[20];
    esp_netif_ip_info_t info;
    esp_netif_t *n = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (n && esp_netif_get_ip_info(n, &info) == ESP_OK && info.ip.addr != 0) {
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&info.ip));
    } else {
        snprintf(buf, sizeof(buf), "未拿到IP");
    }
    return buf;
}

void wifi_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    (void)netif;
    /* 只有固定 IP 那段会用到 */
#if WIFI_FALLBACK_AP
    esp_netif_create_default_wifi_ap();
    /* 兜底热点要用，AP+STA 可同时存在 */
#endif
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        wifi_event_handler, NULL, NULL));
    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);
    /* 这两行不改：之前能连上时就是 WPA_PSK + 默认扫描， * 上次我把它们改成 OPEN +
     * 全信道扫描，之后就出现连不上的情况，先退回原样。 */

    wc.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
    wc.sta.pmf_cfg.capable = true;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
#if USE_STATIC_IP
    esp_netif_dhcpc_stop(netif);
    esp_netif_ip_info_t ip = {0};
    esp_netif_set_ip4_addr(&ip.ip, STATIC_IP_A, STATIC_IP_B, STATIC_IP_C, STATIC_IP_D);
    esp_netif_set_ip4_addr(&ip.gw, STATIC_GW_A, STATIC_GW_B, STATIC_GW_C, STATIC_GW_D);
    esp_netif_set_ip4_addr(&ip.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip));
    esp_netif_dns_info_t dns = {0};
    esp_netif_set_ip4_addr(&dns.ip.u_addr.ip4, STATIC_DNS_A, STATIC_DNS_B, STATIC_DNS_C,
                           STATIC_DNS_D);
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
#endif
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    /* 关省电，避免网页/OTA 卡顿 */
}

void discovery_task(void *arg) {
    (void)arg;
    int s = -1;
    while (s < 0) {
        s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (s < 0) {
            ESP_LOGW(TAG, "discovery: socket 还没就绪，稍后重试");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    int bcast = 1;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(DISCOVERY_PORT);
    dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    for (;;) {
        esp_netif_ip_info_t ip = {0};
        if (nif && esp_netif_get_ip_info(nif, &ip) == ESP_OK && ip.ip.addr != 0) {
            char msg[96];
            int n = snprintf(msg, sizeof(msg), "STM32ARM;ip=" IPSTR ";fw=%s;link=%d",
                             IP2STR(&ip.ip), FW_TAG, link_ok() ? 1 : 0);
            sendto(s, msg, n, 0, (struct sockaddr *)&dst, sizeof(dst));
        }
        vTaskDelay(pdMS_TO_TICKS(DISCOVERY_PERIOD_MS));
    }
}
