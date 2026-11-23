// app_main.c：只做启动编排，具体逻辑在下面的模块里
#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "arm_ik.h"
#include "arm_state.h"
#include "stm32_link.h"
#include "web_server.h"
#include "wifi_mgr.h"

static const char *TAG = "arm";

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    mirror_load(); /* 上次用 /api/mirror 定的关节方向 */

    link_init();
    xTaskCreate(line_task, "line", 4096, NULL, 4, NULL); /* 笛卡尔直线插补 */

    wifi_start();
    start_http();

    xTaskCreate(discovery_task, "discover", 3072, NULL, 3, NULL);
#if WIFI_FALLBACK_AP && AP_ALWAYS
    start_ap_fallback(); /* 排查模式：不等 STA 失败，直接开热点 */
#endif

    ESP_LOGI(TAG, "ready  fw=%s  几何: D1=%.2f A1=%.2f A2=%.2f A3=%.2f", FW_TAG, (double)ARM_D1,
             (double)ARM_A1, (double)ARM_A2, (double)ARM_A3);

    /* 开机先要一次状态，并把模式设成默认值（1 = 网络控制） */
    vTaskDelay(pdMS_TO_TICKS(300));
    link_send("H,1");
    link_send("M,%d", DEFAULT_MODE);
    s_mode = DEFAULT_MODE;
    link_send("G,1"); /* 上电先把夹爪合上 */
    link_send("Q");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "ip=%s mode=%d link=%d angles=%d/%d/%d/%d/%d/%d", cur_ip_str(), (int)s_mode,
                 link_ok() ? 1 : 0, s_angles[0], s_angles[1], s_angles[2], s_angles[3], s_angles[4],
                 s_angles[5]);
    }
}
