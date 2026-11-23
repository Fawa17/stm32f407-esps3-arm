// web_server.c：HTTP 接口与网页

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "app_config.h"
#include "arm_ik.h"
#include "arm_state.h"
#include "stm32_link.h"
#include "wifi_mgr.h"
#include "web_server.h"

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static const char *TAG = "arm";

bool q_get(httpd_req_t *req, const char *key, char *out, size_t out_len) {
    char buf[320];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }
    return httpd_query_key_value(buf, key, out, out_len) == ESP_OK;
}

int q_get_int(httpd_req_t *req, const char *key, int def) {
    char val[32];
    if (!q_get(req, key, val, sizeof(val))) {
        return def;
    }
    return atoi(val);
}

esp_err_t reply_json(httpd_req_t *req, int code, const char *body) {
    httpd_resp_set_status(req, code == 200 ? "200 OK" : "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t reply_text(httpd_req_t *req, const char *body) {
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

void req_log(httpd_req_t *r) {
    ESP_LOGI(TAG, "HTTP %s %s", r->method == HTTP_POST ? "POST" : "GET", r->uri);
}

esp_err_t h_root(httpd_req_t *req) {
    req_log(req);
    size_t len = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start, len);
}

esp_err_t h_state(httpd_req_t *req) {
    req_log(req);
    /* 用当前模型姿态正解一次，把末端坐标一起报出来（网页上实时显示） */

    float tx = 0.0f, ty = 0.0f, tz = 0.0f;
    ArmIK_FK(s_model[0], s_model[1], s_model[2], s_model[3], &tx, &ty, &tz);
    char b[320];
    snprintf(b, sizeof(b),
             "{\"mode\":%d,\"angles\":[%d,%d,%d,%d,%d,%d],\"link\":%d,"
             "\"cmd\":%lu,\"err\":%lu,\"fw\":\"%s\",\"ip\":\"%s\",\"mirror\":%d,"
             "\"stm32\":\"%s\",\"retry\":%d,\"retry_fail\":%d,"
             "\"xyz\":[%.1f,%.1f,%.1f]}",
             (int)s_mode, s_angles[0], s_angles[1], s_angles[2], s_angles[3], s_angles[4],
             s_angles[5], link_ok() ? 1 : 0, (unsigned long)s_cmd_cnt, (unsigned long)s_err_cnt,
             FW_TAG, cur_ip_str(), s_mirror, s_stm32_fw[0] ? s_stm32_fw : "-", s_retry_cnt,
             s_retry_fail, (double)tx, (double)ty, (double)tz);
    return reply_json(req, 200, b);
}

bool iap_busy_reject(httpd_req_t *req) {
    if (!s_busy_iap) {
        return false;
    }
    reply_json(req, 409, "{\"ok\":0,\"err\":\"正在升级 STM32 固件，暂不接受运动指令\"}");
    return true;
}

esp_err_t h_pose(httpd_req_t *req) {
    req_log(req);
    if (iap_busy_reject(req)) {
        return ESP_OK;
    }
    char val[96];
    if (!q_get(req, "angles", val, sizeof(val))) {
        return reply_json(req, 400, "{\"ok\":0,\"err\":\"缺 angles 参数\"}");
    }
    int a[6] = {0};
    int cnt = 0;
    char *p = val;
    while (cnt < 6 && p && *p) {
        a[cnt++] = atoi(p);
        p = strchr(p, ',');
        if (p) {
            p++;
        }
    }
    if (cnt != 6) {
        return reply_json(req, 400, "{\"ok\":0,\"err\":\"angles 必须是 6 个数字\"}");
    }
    int ms = q_get_int(req, "ms", 0);
    cmd_pose(a, ms);
    char b[64];
    snprintf(b, sizeof(b), "OK angles=%d,%d,%d,%d,%d,%d ms=%d", a[0], a[1], a[2], a[3], a[4], a[5],
             ms);
    return reply_text(req, b);
}

esp_err_t h_cart(httpd_req_t *req) {
    req_log(req);
    if (iap_busy_reject(req)) {
        return ESP_OK;
    }
    char vx[24], vy[24], vz[24], vp[24];
    if (!q_get(req, "x", vx, sizeof(vx)) || !q_get(req, "y", vy, sizeof(vy)) ||
        !q_get(req, "z", vz, sizeof(vz))) {
        return reply_json(req, 400, "{\"ok\":0,\"err\":\"缺 x/y/z 参数\"}");
    }
    float x = (float)atof(vx);
    float y = (float)atof(vy);
    float z = (float)atof(vz);
    bool auto_tool = false;
    float tool = 0.0f;
    if (q_get(req, "pitch", vp, sizeof(vp))) {
        if (strcmp(vp, "auto") == 0) {
            auto_tool = true;
        } else {
            tool = (float)atof(vp);
        }
    } else if (q_get(req, "tool", vp, sizeof(vp))) {
        if (strcmp(vp, "auto") == 0) {
            auto_tool = true;
        } else {
            tool = (float)atof(vp);
        }
    }
    int ms = q_get_int(req, "ms", 0);
    ArmIKResult r;
    bool ok = ArmIK_SolveEx(x, y, z, tool, auto_tool, s_model, &r);
    if (!ok || !r.ok) {
        char b[220];
        snprintf(b, sizeof(b), "{\"ok\":0,\"err\":\"%s\",\"dev\":%.2f,\"fw\":\"%s\"}",
                 (r.err && r.err[0]) ? r.err : "无解", (double)r.reachErrCm, FW_TAG);
        return reply_json(req, 400, b);
    }
    int a[6];
    for (int i = 0; i < 4; i++) {
        int s = (int)(r.servo[i] + 0.5f);
        a[i] = s < 0 ? 0 : (s > 180 ? 180 : s);
    }
    a[4] = s_angles[4];
    /* 腕旋转保持不动 */

    a[5] = s_angles[5];
    /* 夹爪保持不动 */

    cmd_pose(a, ms);
    s_cart_xyz[0] = x;
    s_cart_xyz[1] = y;
    s_cart_xyz[2] = z;
    s_cart_valid = true;
    char b[280];
    snprintf(b, sizeof(b),
             "{\"ok\":1,\"angles\":[%d,%d,%d,%d,%d,%d],\"tool\":%d,"
             "\"j\":[%.1f,%.1f,%.1f],\"clamped\":%d,\"dev\":%.2f,"
             "\"g\":[%.2f,%.2f,%.2f],\"fw\":\"%s\"}",
             a[0], a[1], a[2], a[3], a[4], a[5], (int)(r.toolDeg + 0.5f), (double)r.shoulder,
             (double)r.elbow, (double)r.wrist, r.clamped ? 1 : 0, (double)r.reachErrCm,
             (double)ARM_D1, (double)ARM_A1, (double)ARM_A2, FW_TAG);
    return reply_json(req, 200, b);
}

esp_err_t h_line(httpd_req_t *req) {
    req_log(req);
    if (iap_busy_reject(req)) {
        return ESP_OK;
    }
    char vx[24], vy[24], vz[24], vp[16];
    if (!q_get(req, "x", vx, sizeof(vx)) || !q_get(req, "y", vy, sizeof(vy)) ||
        !q_get(req, "z", vz, sizeof(vz))) {
        return reply_json(req, 400, "{\"ok\":0,\"err\":\"缺 x/y/z 参数\"}");
    }
    if (!s_cart_valid) {
        return reply_json(req, 400, "{\"ok\":0,\"err\":\"还没有起点：先用 /api/cart 定位一次\"}");
    }
    float x = (float)atof(vx);
    float y = (float)atof(vy);
    float z = (float)atof(vz);
    bool auto_tool = false;
    float tool = 0.0f;
    if (q_get(req, "pitch", vp, sizeof(vp))) {
        if (strcmp(vp, "auto") == 0) {
            auto_tool = true;
        } else {
            tool = (float)atof(vp);
        }
    }
    /* 起终点都必须有解，否则跑一半停下很难看 */

    ArmIKResult r;
    if (!ArmIK_SolveEx(x, y, z, tool, auto_tool, s_model, &r) || !r.ok) {
        char b[200];
        snprintf(b, sizeof(b), "{\"ok\":0,\"err\":\"终点无解：%s\",\"dev\":%.2f}",
                 (r.err && r.err[0]) ? r.err : "无解", (double)r.reachErrCm);
        return reply_json(req, 400, b);
    }
    int ms = q_get_int(req, "ms", 600);
    if (ms < 100) {
        ms = 100;
    }
    if (ms > 4000) {
        ms = 4000;
    }
    s_line_from[0] = s_cart_xyz[0];
    s_line_from[1] = s_cart_xyz[1];
    s_line_from[2] = s_cart_xyz[2];
    s_line_to[0] = x;
    s_line_to[1] = y;
    s_line_to[2] = z;
    s_line_pitch = tool;
    s_line_auto = auto_tool;
    s_line_ms = ms;
    s_line_err = 0;
    s_line_gen++;
    /* 让上一段插补任务知道"已经换人了" */

    s_line_active = true;
    char b[180];
    snprintf(b, sizeof(b),
             "{\"ok\":1,\"from\":[%.1f,%.1f,%.1f],\"to\":[%.1f,%.1f,%.1f],"
             "\"ms\":%d,\"steps\":%d}",
             (double)s_line_from[0], (double)s_line_from[1], (double)s_line_from[2], (double)x,
             (double)y, (double)z, ms, ms / LINE_STEP_MS);
    return reply_json(req, 200, b);
}

esp_err_t h_jog(httpd_req_t *req) {
    req_log(req);
    if (iap_busy_reject(req)) {
        return ESP_OK;
    }
    int axis = q_get_int(req, "axis", -1);
    int deg = q_get_int(req, "deg", -1);
    if (axis < 0 || axis > 5 || deg < 0 || deg > 180) {
        return reply_json(req, 400, "{\"ok\":0,\"err\":\"axis 0~5, deg 0~180\"}");
    }
    int a[6];
    for (int i = 0; i < 6; i++) {
        a[i] = s_angles[i];
    }
    a[axis] = deg;
    cmd_pose(a, 0);
    char b[64];
    snprintf(b, sizeof(b), "OK,axis=%d,deg=%d", axis, deg);
    return reply_text(req, b);
}

esp_err_t h_gripper(httpd_req_t *req) {
    req_log(req);
    if (iap_busy_reject(req)) {
        return ESP_OK;
    }
    int open = q_get_int(req, "open", q_get_int(req, "g", 1));
    int a[6];
    for (int i = 0; i < 6; i++) {
        a[i] = s_angles[i];
    }
    a[5] = open ? GRIP_OPEN_DEG : GRIP_CLOSE_DEG;
    /*
     * 走和滑块完全相同的路径（模式、镜像、协同时间都过一遍）。
     * 以前这里单独发 "G,x"，那条路既不参与镜像也不参与协同，夹爪会单独抽动一下。
     */

    cmd_pose(a, 0);
    char b[72];
    snprintf(b, sizeof(b), "{\"ok\":1,\"open\":%d,\"g\":%d,\"link\":%d}", open ? 1 : 0, a[5],
             link_ok() ? 1 : 0);
    return reply_json(req, 200, b);
}

esp_err_t h_mode(httpd_req_t *req) {
    req_log(req);
    int remote = q_get_int(req, "remote", 1);
    s_desired_mode = remote ? 1 : 0;
    /*
     * 改 s_mode：命令可能丢帧，改了会两端永久不一致。
     * 真正的 s_mode 等 STM32 的 ST/ST2/HB 回来再更新。
     */

    link_send("M,%d", remote ? 1 : 0);
    return reply_text(req, "OK");
}

esp_err_t h_stop(httpd_req_t *req) {
    req_log(req);
    link_send("S");
    return reply_text(req, "OK");
}

esp_err_t h_mirror(httpd_req_t *req) {
    req_log(req);
    char v[32];
    if (q_get(req, "axes", v, sizeof(v))) {
        int m = 0;
        for (char *p = v; *p; p++) {
            if (*p >= '1' && *p <= '5') {
                m |= 1 << (*p - '1');
            }
        }
        if (strchr(v, '0')) {
            m = 0;
        }
        s_mirror = m;
        mirror_save();
    }
    char b[48];
    snprintf(b, sizeof(b), "{\"ok\":1,\"mirror\":%d}", s_mirror);
    return reply_json(req, 200, b);
}

esp_err_t h_query(httpd_req_t *req) {
    req_log(req);
    link_send("Q");
    return reply_text(req, "OK");
}

esp_err_t h_ota(httpd_req_t *req) {
    req_log(req);
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        return reply_text(req, "ERR no ota partition");
    }
    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(part, OTA_SIZE_UNKNOWN, &handle) != ESP_OK) {
        return reply_text(req, "ERR ota_begin");
    }
    char buf[1460];
    size_t remain = req->content_len;
    ESP_LOGI(TAG, "OTA start: %u bytes -> %s", (unsigned)remain, part->label);
    while (remain > 0) {
        int n = httpd_req_recv(req, buf, remain < sizeof(buf) ? remain : sizeof(buf));
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            esp_ota_abort(handle);
            ESP_LOGE(TAG, "OTA recv failed: %d", n);
            return reply_text(req, "ERR recv");
        }
        if (esp_ota_write(handle, buf, n) != ESP_OK) {
            esp_ota_abort(handle);
            return reply_text(req, "ERR write");
        }
        remain -= (size_t)n;
    }
    if (esp_ota_end(handle) != ESP_OK || esp_ota_set_boot_partition(part) != ESP_OK) {
        return reply_text(req, "ERR finish");
    }
    reply_text(req, "OK rebooting");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint32_t)p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc;
}

void bytes_to_hex(const uint8_t *d, size_t n, char *out) {
    static const char *H = "0123456789ABCDEF";
    for (size_t i = 0; i < n; i++) {
        out[2 * i] = H[d[i] >> 4];
        out[2 * i + 1] = H[d[i] & 0x0F];
    }
    out[2 * n] = '\0';
}

esp_err_t h_update_stm32(httpd_req_t *req) {
    char json[160];
    char line[128];
    char hex[IAP_CHUNK_BYTES * 2 + 1];
    uint8_t buf[IAP_CHUNK_BYTES];
    size_t total = req->content_len;
    size_t done = 0;
    uint32_t crc = 0xFFFFFFFFu;
    bool ok = false;
    int64_t t_start = esp_timer_get_time();
    if (total == 0 || total > 0x20000u) {
        snprintf(json, sizeof(json), "{\"ok\":0,\"err\":\"镜像大小不合法: %u\"}", (unsigned)total);
        return reply_json(req, 400, json);
    }
    ESP_LOGI(TAG, "开始给 STM32 刷固件：%u 字节", (unsigned)total);
    s_busy_iap = true;
    vTaskDelay(pdMS_TO_TICKS(80));
    /* 让在途的心跳命令先落地 */
    /*
     * 0) 先回停放姿态再动手：升级中途 CPU
     * 会停顿几百毫秒、结束还要重启一次，
     * 手臂如果在半空或夹着东西，重启那一下 PWM 消失会直接甩下来。
     * 先给它 1.2 秒回到全 90 + 夹爪闭合，等真到位了再擦 Flash。
     */
    {
        int park[6] = {90, 90, 90, 90, 90, GRIP_CLOSE_DEG};
        pose_send(park, 1200);
        vTaskDelay(pdMS_TO_TICKS(1400));
    }
    /* 1) U：擦除暂存区 + 元数据区 */

    snprintf(line, sizeof(line), "U,%u,0", (unsigned)total);
    link_send("%s", line);
    if (!link_wait_ack(IAP_U_TIMEOUT_MS)) {
        goto finish;
    }
    /* 2) W：逐帧写数据 */

    while (done < total) {
        size_t want = total - done;
        size_t got = 0;
        int recv_timeouts = 0;
        if (esp_timer_get_time() - t_start > (int64_t)IAP_TOTAL_TIMEOUT_MS * 1000) {
            ESP_LOGE(TAG, "IAP 总超时，已写 %u/%u", (unsigned)done, (unsigned)total);
            goto finish;
        }
        if (want > IAP_CHUNK_BYTES) {
            want = IAP_CHUNK_BYTES;
        }
        while (got < want) {
            int n = httpd_req_recv(req, (char *)buf + got, (int)(want - got));
            if (n == HTTPD_SOCK_ERR_TIMEOUT) {
                /*
                 * 连续几次都收不到数据 = 客户端已经放弃或连接半开，立刻收手，
                 * 否则这里会无限 continue，把唯一的 httpd 任务焊死在这个请求上。
                 */

                if (++recv_timeouts >= IAP_RECV_MAX_TIMEOUT) {
                    ESP_LOGE(TAG, "收包超时 %d 次，放弃本次升级", recv_timeouts);
                    goto finish;
                }
                continue;
            }
            if (n <= 0) {
                goto finish;
            }
            recv_timeouts = 0;
            got += (size_t)n;
        }
        crc = crc32_update(crc, buf, got);
        bytes_to_hex(buf, got, hex);
        snprintf(line, sizeof(line), "W,%u,%s", (unsigned)done, hex);
        /*
         * 单帧最多重发一次：偶发的 UART 抖动不该让整次升级报废，
         * 但连续两次无应答说明链路真有问题，立刻收手（否则会拖到 90 秒上限）。
         */
        {
            int tries = 0;
            for (;;) {
                link_send("%s", line);
                if (link_wait_ack(IAP_W_TIMEOUT_MS)) {
                    break;
                }
                if (++tries >= 2) {
                    ESP_LOGE(TAG, "写第 %u 字节两次无应答，放弃", (unsigned)done);
                    goto finish;
                }
            }
        }
        done += got;
        if ((done % 4000u) < got) {
            ESP_LOGI(TAG, "  IAP %u/%u", (unsigned)done, (unsigned)total);
        }
    }
    /* 3) E：校验 + 重启 */

    crc = ~crc;
    snprintf(line, sizeof(line), "E,%lu", (unsigned long)crc);
    link_send("%s", line);
    ok = link_wait_ack(IAP_E_TIMEOUT_MS);
finish:
    s_busy_iap = false;
    if (!ok) {
        snprintf(json, sizeof(json), "{\"ok\":0,\"err\":\"传输失败（到第 %u 字节）\",\"bytes\":%u}",
                 (unsigned)done, (unsigned)done);
        ESP_LOGE(TAG, "STM32 固件上传失败，停在 %u 字节", (unsigned)done);
        return reply_json(req, 400, json);
    }
    snprintf(json, sizeof(json),
             "{\"ok\":1,\"bytes\":%u,\"crc\":\"0x%08lx\",\"note\":\"STM32 正在重启并安装\"}",
             (unsigned)done, (unsigned long)crc);
    ESP_LOGI(TAG, "STM32 固件上传完成：%u 字节", (unsigned)done);
    return reply_json(req, 200, json);
}

esp_err_t h_iap_selftest(httpd_req_t *req) {
    char line[128];
    char hex[IAP_CHUNK_BYTES * 2 + 1];
    char json[640];
    uint8_t z[IAP_CHUNK_BYTES];
    int off = 0;
    int64_t t0;
    bool ok;
    memset(z, 0xAA, sizeof(z));
    bytes_to_hex(z, sizeof(z), hex);
    s_busy_iap = true;
    off += snprintf(json + off, sizeof(json) - off, "{\"u_ok\":");
    snprintf(line, sizeof(line),
             "U,4096,0"); /* 必须 ≥ IAP_MIN_APP_SIZE(4096)，否则 STM32 直接回 ERR */

    t0 = esp_timer_get_time();
    link_send("%s", line);
    ok = link_wait_ack(IAP_U_TIMEOUT_MS);
    off += snprintf(json + off, sizeof(json) - off, "%d,\"u_ms\":%d,\"frames\":[", ok ? 1 : 0,
                    (int)((esp_timer_get_time() - t0) / 1000));
    for (int i = 0; ok && i < 10; i++) {
        snprintf(line, sizeof(line), "W,%u,%s", (unsigned)(i * IAP_CHUNK_BYTES), hex);
        t0 = esp_timer_get_time();
        link_send("%s", line);
        bool a = link_wait_ack(IAP_W_TIMEOUT_MS);
        if (!a) {
            ok = false;
        }
        off += snprintf(json + off, sizeof(json) - off, "%s%d%s", i ? "," : "",
                        (int)((esp_timer_get_time() - t0) / 1000), a ? "" : "!");
    }
    off += snprintf(json + off, sizeof(json) - off, "],\"ok\":%d}", ok ? 1 : 0);
    s_busy_iap = false;
    ESP_LOGI(TAG, "IAP 自检: %s", json);
    return reply_json(req, ok ? 200 : 500, json);
}

httpd_handle_t start_http(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 18;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start 失败");
        return NULL;
    }
    ESP_LOGI(TAG, "HTTP 服务器已启动: 端口 80");
    const httpd_uri_t uris[] = {
        {"/", HTTP_GET, h_root, NULL},
        {"/api/state", HTTP_GET, h_state, NULL},
        {"/api/pose", HTTP_GET, h_pose, NULL},
        {"/api/cart", HTTP_GET, h_cart, NULL},
        {"/api/jog", HTTP_GET, h_jog, NULL},
        {"/api/line", HTTP_GET, h_line, NULL},
        {"/api/gripper", HTTP_GET, h_gripper, NULL},
        {"/api/mode", HTTP_GET, h_mode, NULL},
        {"/api/stop", HTTP_GET, h_stop, NULL},
        {"/api/query", HTTP_GET, h_query, NULL},
        {"/api/mirror", HTTP_GET, h_mirror, NULL},
        {"/update", HTTP_POST, h_ota, NULL},
        {"/update_stm32", HTTP_POST, h_update_stm32, NULL},
        {"/api/iap_selftest", HTTP_GET, h_iap_selftest, NULL},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    return server;
}
