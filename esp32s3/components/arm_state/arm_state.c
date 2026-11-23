// arm_state.c：关节目标、镜像换算与直线插补（状态都在这里）

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "app_config.h"
#include "arm_ik.h"
#include "arm_state.h"
#include "arm_bus.h"

static const char *TAG = "arm";

volatile int s_mode = 0;
volatile int s_desired_mode = DEFAULT_MODE;
volatile int s_angles[6] = {90, 90, 90, 90, 90, GRIP_CLOSE_DEG};
float s_model[6] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 25.0f};
int s_mirror = 7;
volatile bool s_line_active = false;
volatile uint32_t s_line_gen = 0;
volatile int s_line_err = 0;
float s_line_from[3];
float s_line_to[3];
float s_line_pitch = 0.0f;
bool s_line_auto = false;
int s_line_ms = 600;
float s_cart_xyz[3];
bool s_cart_valid = false;

void apply_mirror(int a[6]) {
    for (int i = 0; i < 5; i++) {
        if (s_mirror & (1 << i)) {
            a[i] = 180 - a[i];
        }
    }
}

void apply_mirror_f(float f[6]) {
    if (s_mirror == 0) {
        return;
    }
    for (int i = 0; i < 5; i++) {
        if (s_mirror & (1 << i)) {
            f[i] = 180.0f - f[i];
        }
    }
}

void mirror_save(void) {
    nvs_handle_t h;
    if (nvs_open("arm", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "mirror", (uint8_t)s_mirror);
        nvs_commit(h);
        nvs_close(h);
    }
}

void mirror_load(void) {
    nvs_handle_t h;
    if (nvs_open("arm", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 0;
        if (nvs_get_u8(h, "mirror", &v) == ESP_OK) {
            s_mirror = v;
        }
        nvs_close(h);
    }
}

void pose_send(const int logic[6], int ms) {
    char buf[80];
    int a[6];
    for (int i = 0; i < 6; i++) {
        a[i] = logic[i];
    }
    apply_mirror(a); /* 只在这里产生"线上角" ，下发到stm32*/

    if (ms <= 0) {
        int tmax = 60;
        for (int i = 0; i < 6; i++) {
            int d = logic[i] - (int)s_angles[i]; /* 逻辑角对逻辑角，别拿镜像后的比 */

            int sp = (i == 5) ? GRIP_SPEED_DPS : JOINT_SPEED_DPS;
            int t = (d < 0 ? -d : d) * 1000 / sp;
            if (t > tmax) {
                tmax = t;
            }
        }
        if (tmax > 1500) {
            tmax = 1500;
        }
        ms = tmax;
    }
    if (ms > 0) {
        snprintf(buf, sizeof(buf), "P,%d,%d,%d,%d,%d,%d,%d", a[0], a[1], a[2], a[3], a[4], a[5],
                 ms);
    } else {
        snprintf(buf, sizeof(buf), "P,%d,%d,%d,%d,%d,%d", a[0], a[1], a[2], a[3], a[4], a[5]);
    }
    link_pend_track(buf); /* 没收到 OK 就自动重发 */
    link_send("%s", buf);
    /* 两条真相源都只存逻辑角：镜像结果算完就扔，绝不回写 */

    for (int i = 0; i < 6; i++) {
        s_angles[i] = logic[i];
        s_model[i] = (float)logic[i];
    }
}

void pose_send_f(const float logic[6], int ms) {
    char buf[128];
    int a[6];
    float f[6];
    for (int i = 0; i < 6; i++) {
        f[i] = logic[i];
        if (f[i] < 0.0f) {
            f[i] = 0.0f;
        }
        if (f[i] > 180.0f) {
            f[i] = 180.0f;
        }
        a[i] = (int)(f[i] + 0.5f);
    }
    if (ms <= 0) {
        int tmax = 60;
        for (int i = 0; i < 6; i++) {
            float d = f[i] - (float)s_angles[i];
            /* 逻辑角对逻辑角 */

            float da = d < 0 ? -d : d;
            float sp = (i == 5) ? (float)GRIP_SPEED_DPS : (float)JOINT_SPEED_DPS;
            int t = (int)(da * 1000.0f / sp);
            if (t > tmax) {
                tmax = t;
            }
        }
        if (tmax > 1500) {
            tmax = 1500;
        }
        ms = tmax;
    }
    apply_mirror_f(f);
    /* 镜像只在这里做一次 */

    snprintf(buf, sizeof(buf), "P,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d", f[0], f[1], f[2], f[3], f[4],
             f[5], ms);
    link_pend_track(buf);
    link_send("%s", buf);
    for (int i = 0; i < 6; i++) {
        s_angles[i] = a[i];
        /* 存逻辑角 */

        s_model[i] = logic[i];
    }
}

void cmd_pose(const int a_in[6], int ms) {
    int a[6];
    for (int i = 0; i < 6; i++) {
        a[i] = a_in[i];
    }
    if (a[0] < 0 || a[0] > 180 || a[1] < 0 || a[1] > 180 || a[2] < 0 || a[2] > 180 || a[3] < 0 ||
        a[3] > 180 || a[4] < 0 || a[4] > 180 || a[5] < 0 || a[5] > 180) {
        return;
    }
    s_line_active = false;
    /* 人手动介入时，机器自动轨迹让路 */
    /* 记下镜像前的"模型姿态"：IK
                                                就近择优要用它，否则每换一个点都可能换一组解，
                                                看起来就是手臂在同一个高度附近来回翻肘 */

    for (int i = 0; i < 6; i++) {
        s_model[i] = (float)a[i];
    }
    /* 注意：镜像换算在 pose_send 内部做，这里千万不能再 apply_mirror 一次 ——       否则 h_jog /
     * h_gripper 这种"读 s_angles 再下发"的路径会被翻两遍。 */

    if (s_mode != 1) {
        link_pend_track("M,1");
        /* 切远程也走确认，丢了就重发 */

        link_send("M,1");
        s_mode = 1;
    }
    s_desired_mode = 1;
    /* 发了姿态命令就说明想用网络控制，别下一秒又被拉回体感 */

    pose_send(a, ms);
}

void line_task(void *arg) {
    (void)arg;
    for (;;) {
        if (!s_line_active) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        int steps = s_line_ms / LINE_STEP_MS;
        if (steps < 1) {
            steps = 1;
        }
        const float fx = s_line_from[0], fy = s_line_from[1], fz = s_line_from[2];
        const float tx = s_line_to[0], ty = s_line_to[1], tz = s_line_to[2];
        const uint32_t gen = s_line_gen;
        /* 这一段归我管；期间来了新命令就自增 */

        bool ok = true;
        for (int k = 1; k <= steps && s_line_active && s_line_gen == gen; k++) {
            float t = (float)k / (float)steps;
            float x = fx + (tx - fx) * t;
            float y = fy + (ty - fy) * t;
            float z = fz + (tz - fz) * t;
            ArmIKResult r;
            if (!ArmIK_SolveEx(x, y, z, s_line_pitch, s_line_auto, s_model, &r) || !r.ok) {
                s_line_err = 1;
                ok = false;
                break;
            }
            /* 保留 0.1°：整度下发时 40cm 外的末端误差就有 0.7cm/度，直线会变成折线 */

            float fs[6];
            for (int i = 0; i < 4; i++) {
                float s = r.servo[i];
                fs[i] = s < 0.0f ? 0.0f : (s > 180.0f ? 180.0f : s);
            }
            fs[4] = (float)s_angles[4];
            fs[5] = (float)s_angles[5];
            for (int i = 0; i < 6; i++) {
                s_model[i] = fs[i];
            }
            /* 镜像交给 pose_send_f 内部做，这里绝不能再 apply_mirror_f 一次 */

            if (s_mode != 1) {
                link_send("M,1");
                s_mode = 1;
            }
            s_desired_mode = 1;
            pose_send_f(fs, LINE_STEP_MS);
            vTaskDelay(pdMS_TO_TICKS(LINE_STEP_MS));
            /* 手动点动/新一段插补会改标志；这里必须再查一次，
             * 否则会多走一格，把用户刚点的位置又顶回去。 */

            if (!s_line_active || s_line_gen != gen) {
                ok = false;
                break;
            }
        }
        /* 只有"还归我管"才清标志：否则会把用户刚发起的新一段插补吞掉 */

        if (s_line_gen == gen) {
            if (ok) {
                s_cart_xyz[0] = tx;
                s_cart_xyz[1] = ty;
                s_cart_xyz[2] = tz;
            }
            s_line_active = false;
            ESP_LOGI(TAG, "直线插补结束: %s", ok ? "到位" : "中途无解已停止");
        }
    }
}
