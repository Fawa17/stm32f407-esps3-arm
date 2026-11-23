"""float_check.py — 用 Python 复刻固件里的浮点 S 曲线，验证「更顺、更准」。

对应 C 代码：C:/STM32/smart hand/BSP_Drivers/RobotArm.c 的 RobotArm_Update()
参数与固件一致（RobotArm.h / MPU6050.c）：
    ROBOTARM_TICK_MS   10       -> dt = 0.01 s
    ROBOTARM_JERK_STEPS 6
    JOINT_SPEED_DPS    180      度/秒
    JOINT_ACCEL_DPS2   900      度/秒²
    GRIP_SPEED_DPS     600
    GRIP_ACCEL_DPS2   4000
    ROBOTARM_SOFT_ZONE_DEG 12 / ROBOTARM_SOFT_PCT 30

输出：耗时、峰值速度、过冲、稳态误差。
"""
import sys

TICK_MS = 10
DT = TICK_MS / 1000.0
JERK_STEPS = 6
SOFT_ZONE = 12.0
SOFT_PCT = 30.0


def simulate(pos, target, vmax, amax, lo=0.0, hi=180.0, max_ms=20000):
    v = 0.0
    acc = 0.0
    tms = 0
    peak = 0.0
    hist = [pos]

    while tms < max_ms:
        dist = target - pos
        if abs(dist) < 1e-4:
            pos = target
            v = 0.0
            acc = 0.0
            break

        rem = abs(dist)
        vm = vmax
        if vm < 1.0:
            vm = 1.0

        edge = (hi - target) if target > pos else (target - lo)
        if edge < 0.0:
            edge = 0.0
        if edge < SOFT_ZONE:
            pct = SOFT_PCT + (100.0 - SOFT_PCT) * edge / SOFT_ZONE
            vm *= pct * 0.01
            if vm < 0.5:
                vm = 0.5

        a = amax if amax >= 1.0 else 1.0

        stopd = (v * v) / (2.0 * a)
        acc_tgt = -a if rem <= stopd else a

        da = a / JERK_STEPS
        if acc < acc_tgt:
            acc = min(acc + da, acc_tgt)
        elif acc > acc_tgt:
            acc = max(acc - da, acc_tgt)

        v += acc * DT
        v = max(0.0, min(v, vm))
        peak = max(peak, v)

        step = v * DT
        if step >= rem:
            pos = target
            v = 0.0
            acc = 0.0
        else:
            pos += step if dist > 0 else -step

        hist.append(pos)
        tms += TICK_MS

    overshoot = 0.0
    if target != hist[0]:
        sign = 1.0 if target > hist[0] else -1.0
        overshoot = max(0.0, (max(hist) - target) if sign > 0 else (target - min(hist)))

    return {
        "ms": tms,
        "peak": peak,
        "overshoot": overshoot,
        "err": abs(pos - target),
        "final": pos,
    }


def row(name, a, b, vmax, amax):
    r = simulate(a, b, vmax, amax)
    print("%-18s %6.1f->%6.1f  %5d ms  peak %6.1f 过冲 %5.3f 误差 %6.4f  末值 %7.2f"
          % (name, a, b, r["ms"], r["peak"], r["overshoot"], r["err"], r["final"]))
    return r


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    print("每拍 %d ms，jerk 限幅 %d 拍，边界降速 %.0f°/底速 %.0f%%\n"
          % (TICK_MS, JERK_STEPS, SOFT_ZONE, SOFT_PCT))

    print("== 关节：180°/s，900°/s² ==")
    row("大臂 90->130", 90, 130, 180, 900)
    row("大臂 90->170", 90, 170, 180, 900)
    row("底座 90->150", 90, 150, 180, 900)
    row("微调 90->91", 90, 91, 180, 900)
    row("微调 90->90.5", 90, 90.5, 180, 900)
    row("贴限位 90->178", 90, 178, 180, 900)

    print("\n== 夹爪：600°/s，4000°/s² ==")
    row("夹爪 25->135", 25, 135, 600, 4000)
    row("夹爪 135->25", 135, 25, 600, 4000)
    row("夹爪 90->92", 90, 92, 600, 4000)

    print("\n== 对照：老版整数核心（0.01°/拍 截断）==")
    # 老版：vmax = speed*2 = 360 (0.01°/拍)，a = 400/25 = 16，jerk = 16/6 = 2
    vmax_i, a_i, jerk_i = 360, 16, 2
    pos_i, v_i, acc_i, t, peak = 9000, 0, 0, 0, 0
    tgt_i = 13000
    while t < 20000:
        rem = tgt_i - pos_i
        if rem == 0:
            break
        acc_tgt = -a_i if rem * 2 * a_i <= v_i * v_i else a_i
        if acc_i < acc_tgt:
            acc_i = min(acc_i + jerk_i, acc_tgt)
        elif acc_i > acc_tgt:
            acc_i = max(acc_i - jerk_i, acc_tgt)
        v_i += acc_i
        v_i = max(0, min(v_i, vmax_i))
        if v_i > rem:
            v_i = rem
        if v_i == 0:
            v_i = 1
        pos_i += v_i
        peak = max(peak, v_i)
        t += TICK_MS
    print("大臂 90->130        %5d ms  peak %6.1f (°/s)  (整数版：加速度只有 5 拍爬升，收尾粒度 1 格)"
          % (t, peak * 0.01 * (1000.0 / TICK_MS)))


if __name__ == "__main__":
    main()