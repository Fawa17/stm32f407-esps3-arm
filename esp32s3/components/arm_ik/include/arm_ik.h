// arm_ik.h - 逆运动学，跑在 ESP32-S3 上，STM32 只收最终角度。
//
// 算法是从桌面仿真页 arm5dof_fixed.html 搬过来的，结构没改：
//   - 关节角直接用舵机角（90 = 竖直/伸直），省掉一层 off/sign 换算
//   - 先扫工具绝对倾角 phi，再对腕点做两连杆解析解
//   - 打分 = 越限角度总和 * 100 + 姿态偏差，取最小的那个
//
// 坐标：右手系，单位 cm。原点在底座转轴和桌面的交点，
//       +X 底座正前方（J1 = 90 时爪尖指向），+Y 左手边，+Z 向上。
//
// phi 是夹爪轴线和 +Z 的夹角（内部量）：0 朝上、90 水平朝前、180 朝下。
// 对外还是用 toolDeg，和网页一致：toolDeg = 90 - phi
//       0 = 水平朝前，-90 = 朝下，+90 = 朝上
#pragma once

#include <math.h>
#include <string.h>
#include <stdbool.h>

/* ---------------- 几何参数（cm）----------------
 *  选哪一套由下面的开关决定，两套都保留着方便对照：
 *    预设 1 = 仿真页 arm5dof_fixed.html 的数值（来自 CAD 模型）
 *    预设 0 = 尺子实测值（8 / 10.5 / 14.5 / 7 / 12.5）
 *  实测差一点很正常，哪套跟真机对得上就用哪套。 */
#define ARM_GEOM_PRESET 1

#if ARM_GEOM_PRESET == 1
/* 仿真页 arm5dof_fixed.html 的四个输入框默认值（页面会用它覆盖脚本里的硬编码）：
 *   d1 = 85.98   L1 = 104.91   L2 = 145.0   L3 = 177.70  （mm）
 * 必须与页面一致，否则同样的坐标两边摆出来的姿态不同。 */
#define ARM_D1 8.598f  /* 桌面 → 肩关节转轴 */
#define ARM_A1 10.491f /* 大臂：肩 → 肘   */
#define ARM_A2 14.500f /* 小臂：肘 → 腕   */
#define ARM_A3 17.770f /* 腕 → 爪尖      */
#else
#define ARM_D1 8.0f
#define ARM_A1 10.5f
#define ARM_A2 14.5f
#define ARM_A3 19.5f /* 腕座 7 + 夹爪 12.5 */
#endif

/* 关节绝对能转到的范围（度）——必须和 STM32 的 MPU6050.c 一致 */
#define ARM_JMIN 10.0f
#define ARM_JMAX 170.0f

/* 工具倾角扫描范围（度）。网页扫 ±240°，桌面机械臂 ±180° 就够，
 * 多扫一点不会错，只是多花 CPU。 */
#define ARM_PHI_MIN (-240.0f)
#define ARM_PHI_MAX (240.0f)
#define ARM_PHI_STEP 1.0f

/* 解算安全边距（cm）：贴到完全伸直/完全折叠时舵机会抖 */
#define ARM_REACH_PAD 0.05f

/* 离底座轴太近时方位角没有意义 */
#define ARM_RHO_MIN 1.5f

/* 用解出的角度做正运动学回代，偏差超过这个值（cm）就判这个点到不了 */
#define ARM_REACH_TOL_CM 0.5f

/* 关节越限超过这么多度就认为需要改机械结构，而不是"夹一下凑合" */
#define ARM_CLAMP_TOL_DEG 10.0f

/* ---------------- 伺服换算表（保留原有结构，方便改方向）----------------
 * servo = off + sign * joint，然后夹到 [lo, hi]
 * 本版全部是 sign = +1 / off = 90：joint 就是"相对竖直的倾角"，
 * 也就是仿真页里的 (t - 90)。 */
typedef struct {
    float off;
    float sign;
    float lo;
    float hi;
} ArmAxisMap;

static const ArmAxisMap ARM_AXIS[6] = {
    {90.0f, 1.0f, ARM_JMIN, ARM_JMAX}, /* 0 底座    */
    {90.0f, 1.0f, ARM_JMIN, ARM_JMAX}, /* 1 大臂    */
    {90.0f, 1.0f, ARM_JMIN, ARM_JMAX}, /* 2 小臂    */
    {90.0f, 1.0f, ARM_JMIN, ARM_JMAX}, /* 3 腕俯仰  */
    {90.0f, 1.0f, 10.0f, 170.0f},      /* 4 腕旋转  */
    {0.0f, 1.0f, 0.0f, 180.0f},        /* 5 夹爪    */
};

/* ---------------- 解算结果 ---------------- */
typedef struct {
    bool ok;       /* 几何上可达且没被限位卡死 */
    bool clamped;  /* 有关节被软限位裁剪 */
    bool toolAuto; /* 工具姿态是自动挑的（wrist≈0） */

    const char *err; /* ok=false 时的原因（UTF-8 中文） */

    /* 关节角（度，相对竖直）：t = servo - 90 */
    float base;     /* J1 已减 90，0 = 正前方 */
    float shoulder; /* J2 - 90，0 = 大臂竖直 */
    float elbow;    /* J3 - 90，0 = 大小臂成直线 */
    float wrist;    /* J4 - 90，0 = 夹爪顺着小臂 */

    float toolDeg;    /* 实际工具倾角，0 = 水平朝前，-90 = 朝下 */
    float servo[6];   /* 要下发的舵机角度 0~180 */
    float clampDeg;   /* 单个关节超出限位的最大度数 */
    float reachErrCm; /* 用这组舵机角正算，爪尖偏离目标多少 cm */
} ArmIKResult;

/* ---------------- 小工具 ---------------- */
static inline float ArmIK_Clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float ArmIK_Rad2Deg(float r) {
    return r * 57.29577951f;
}
static inline float ArmIK_Deg2Rad(float d) {
    return d * 0.01745329252f;
}

/* 关节角 → 舵机角度，越界就夹住并记录超了多少度 */
static inline void ArmIK_ToServo(int axis, float jointDeg, ArmIKResult *r) {
    const ArmAxisMap *m = &ARM_AXIS[axis];
    float s = m->off + m->sign * jointDeg;

    if (s < m->lo) {
        float over = m->lo - s;
        if (over > r->clampDeg) {
            r->clampDeg = over;
        }
        s = m->lo;
        r->clamped = true;
    } else if (s > m->hi) {
        float over = s - m->hi;
        if (over > r->clampDeg) {
            r->clampDeg = over;
        }
        s = m->hi;
        r->clamped = true;
    }

    r->servo[axis] = s;
}

/* 舵机角度 → 关节角（ToServo 的逆运算） */
static inline float ArmIK_ServoToJoint(int axis, float servoDeg) {
    const ArmAxisMap *m = &ARM_AXIS[axis];
    return (m->sign != 0.0f) ? ((servoDeg - m->off) / m->sign) : 0.0f;
}

/* 某个关节角需要越限多少度（0 表示在范围内） */
static inline float ArmIK_Over(int axis, float servoDeg) {
    const ArmAxisMap *m = &ARM_AXIS[axis];
    if (servoDeg < m->lo) {
        return m->lo - servoDeg;
    }
    if (servoDeg > m->hi) {
        return servoDeg - m->hi;
    }
    return 0.0f;
}

/* =============================================================================
 *  正运动学：四个舵机角 → 爪尖位置（与仿真页 fk() 完全一致）
 *    R = A1·sin(J2-90) + A2·sin(J2+J3-180) + A3·sin(J2+J3+J4-270)
 *    Z = D1 + A1·cos(J2-90) + A2·cos(J2+J3-180) + A3·cos(J2+J3+J4-270)
 * ========================================================================== */
static inline void ArmIK_FK(float t1, float t2, float t3, float t4, float *x, float *y, float *z) {
    float yaw = ArmIK_Deg2Rad(t1 - 90.0f);
    float d2 = ArmIK_Deg2Rad(t2 - 90.0f);
    float d3 = ArmIK_Deg2Rad(t3 - 90.0f);
    float d4 = ArmIK_Deg2Rad(t4 - 90.0f);
    float a23 = d2 + d3;
    float a234 = a23 + d4;

    float R = ARM_A1 * sinf(d2) + ARM_A2 * sinf(a23) + ARM_A3 * sinf(a234);
    float Z = ARM_D1 + ARM_A1 * cosf(d2) + ARM_A2 * cosf(a23) + ARM_A3 * cosf(a234);

    *x = R * cosf(yaw);
    *y = R * sinf(yaw);
    *z = Z;
}

/* =============================================================================
 *  逆解主函数：目标点 (x,y,z) + 想要的工具姿态
 *
 *  toolAuto = false：尽量让夹爪轴线达到 toolDeg（0=水平，-90=朝下）
 *  toolAuto = true ：不指定姿态，挑"腕关节最不别扭（J4≈90）"的解
 *
 *  算法（同 arm5dof_fixed.html 的 ikScan）：
 *      for φ in [-240, 240] step 1°
 *         腕点 = 目标 - A3·(sinφ, cosφ)      （竖直平面内）
 *         两连杆解析解 → 两组（肘上/肘下）
 *         四个关节角 = [方位, 90+d2, 90+d3, 90+φ-d2-d3]
 *         打分 key = 越限总量×100 + 姿态偏差，取最小
 *  带多解择优：同一个点通常有几十组解（φ 扫 240°、两组肘方向、两种底座朝向）。
 *  打分 = 越限量×1000 + 姿态偏差 + 关节改动量×0.3
 *    - 越限量乘 1000：任何可行解都排在不可行解前面
 *    - 关节改动量按 cur[]（当前舵机角）算，让手臂选"离现在最近"的解，
 *      夹爪在同一个点附近来回移动时不会突然把肘翻到另一边（这就是抖的原因）
 *    - cur 传 NULL 就退化成老行为（只看姿态偏差）
 * ========================================================================== */
static inline bool ArmIK_SolveEx(float x, float y, float z, float toolDeg, bool toolAuto,
                                 const float *cur, ArmIKResult *r) {
    memset(r, 0, sizeof(*r));
    r->toolAuto = toolAuto;

    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
        r->err = "坐标无效";
        return false;
    }
    if (z < 0.0f) {
        r->err = "Z 低于桌面";
        return false;
    }

    float rho = sqrtf(x * x + y * y);
    if (rho < ARM_RHO_MIN) {
        r->err = "太靠近底座中心，方位角算不出来";
        return false;
    }

    float yawServo = ArmIK_Rad2Deg(atan2f(y, x)) + 90.0f;
    while (yawServo > 180.0f) {
        yawServo -= 360.0f;
    }
    while (yawServo < -180.0f) {
        yawServo += 360.0f;
    }

    /* 想要 φ（工具绝对倾角）：toolDeg = 90 - φ */
    float phiWant = 90.0f - toolDeg;

    float bestKey = 0.0f;
    float bT1 = 0.0f, bT2 = 0.0f, bT3 = 0.0f, bT4 = 0.0f, bPhi = 0.0f;
    bool found = false;

    float yawFlip = (yawServo > 0.0f) ? (yawServo - 180.0f) : (yawServo + 180.0f);

    /* 两种底座朝向：
     *   R = +rho ：手臂朝前伸（常见）
     *   R = -rho ：大臂/小臂向后折，末端落在底座"背后" 180° 的方向上。
     *              仿真页没有这一支，所以它会把"向后折"的姿势误判成不可达。 */
    int branch;
    for (branch = 0; branch < 2; branch++) {
        float bServo = (branch == 0) ? yawServo : yawFlip;
        float Rsign = (branch == 0) ? rho : -rho;

        float phi;
        for (phi = ARM_PHI_MIN; phi <= ARM_PHI_MAX + 0.001f; phi += ARM_PHI_STEP) {
            float p = ArmIK_Deg2Rad(phi);

            /* 腕关节该在的位置（竖直平面 R-Z 内） */
            float wr = Rsign - ARM_A3 * sinf(p);
            float h = (z - ARM_A3 * cosf(p)) - ARM_D1;
            float D = sqrtf(wr * wr + h * h);

            if (D > (ARM_A1 + ARM_A2 - ARM_REACH_PAD)) {
                continue;
            }
            if (D < (fabsf(ARM_A1 - ARM_A2) + ARM_REACH_PAD)) {
                continue;
            }

            float D2 = D * D;
            float cBend = (D2 - ARM_A1 * ARM_A1 - ARM_A2 * ARM_A2) / (2.0f * ARM_A1 * ARM_A2);
            float bend = ArmIK_Rad2Deg(acosf(ArmIK_Clamp(cBend, -1.0f, 1.0f)));

            float cBeta = (ARM_A1 * ARM_A1 + D2 - ARM_A2 * ARM_A2) / (2.0f * ARM_A1 * D);
            float beta = ArmIK_Rad2Deg(acosf(ArmIK_Clamp(cBeta, -1.0f, 1.0f)));

            float alpha = ArmIK_Rad2Deg(atan2f(wr, h));

            int k;
            for (k = 0; k < 2; k++) {
                float d2 = (k == 0) ? (alpha - beta) : (alpha + beta);
                float d3 = (k == 0) ? bend : -bend;

                float t1 = bServo;
                float t2 = 90.0f + d2;
                float t3 = 90.0f + d3;
                float t4 = 90.0f + phi - d2 - d3;

                /* 越限总量 */
                float viol =
                    ArmIK_Over(0, t1) + ArmIK_Over(1, t2) + ArmIK_Over(2, t3) + ArmIK_Over(3, t4);

                /* 姿态偏差：自动模式完全照仿真页的规矩（|φ| 最小 = 工具尽量竖直）；
                 * 指定模式则让 φ 尽量贴近要的姿态（权重放大，免得被"少动关节"抢走） */
                float dev = toolAuto ? fabsf(phi) : (3.0f * fabsf(phi - phiWant));

                /* 关节改动量：选离当前姿态最近的解，避免肘部翻来翻去 */
                float cont = 0.0f;
                if (cur != 0) {
                    cont = fabsf(t1 - cur[0]) + fabsf(t2 - cur[1]) + fabsf(t3 - cur[2]) +
                           fabsf(t4 - cur[3]);
                }

                float key = viol * 1000.0f + dev + cont * 0.3f;

                if (!found || key < bestKey) {
                    found = true;
                    bestKey = key;
                    bT1 = t1;
                    bT2 = t2;
                    bT3 = t3;
                    bT4 = t4;
                    bPhi = phi;
                }
            }
        }
    }

    if (!found) {
        r->err = "超出可达空间：腕点离肩太远或太近";
        return false;
    }

    /* 记录关节角（减 90 存成"相对竖直"的量，方便上层显示） */
    r->base = bT1 - 90.0f;
    r->shoulder = bT2 - 90.0f;
    r->elbow = bT3 - 90.0f;
    r->wrist = bT4 - 90.0f;
    r->toolDeg = 90.0f - bPhi;

    /* 换成舵机角（会夹限位并累计 clampDeg） */
    ArmIK_ToServo(0, r->base, r);
    ArmIK_ToServo(1, r->shoulder, r);
    ArmIK_ToServo(2, r->elbow, r);
    ArmIK_ToServo(3, r->wrist, r);
    r->servo[4] = ARM_AXIS[4].off; /* 腕旋转保持中位，由调用者覆盖 */
    r->servo[5] = ARM_AXIS[5].off; /* 夹爪保持张开，由调用者覆盖 */

    /* 越限太多：说明这台机器结构上就够不到，别硬掰 */
    if (r->clampDeg > ARM_CLAMP_TOL_DEG) {
        r->err = "超出关节限位（该点需要更大幅度的关节角）";
        return false;
    }

    /* 正运动学回代：偏太多说明是"夹一下凑出来的假解" */
    float fx, fy, fz;
    ArmIK_FK(r->servo[0], r->servo[1], r->servo[2], r->servo[3], &fx, &fy, &fz);

    float dx = fx - x, dy = fy - y, dz = fz - z;
    r->reachErrCm = sqrtf(dx * dx + dy * dy + dz * dz);

    if (r->reachErrCm > ARM_REACH_TOL_CM) {
        r->err = "受关节限位限制，这个点到不了（最近的解也会偏几厘米）";
        return false;
    }

    r->ok = true;
    return true;
}

/* 老接口：不带"当前姿态"，就近择优失效，只有姿态偏差起作用 */
static inline bool ArmIK_Solve(float x, float y, float z, float toolDeg, bool toolAuto,
                               ArmIKResult *r) {
    return ArmIK_SolveEx(x, y, z, toolDeg, toolAuto, 0, r);
}
