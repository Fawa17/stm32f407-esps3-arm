"""S 曲线速度规划离线校验：把 RobotArm.c 的整数运算原样搬到 Python 跑一遍。

目的：
  1) 确认不会过冲（位置永远不越过目标）；
  2) 确认能停住（速度归零、位置等于目标）；
  3) 给出各典型动作的实际耗时，方便对照"丝滑但不拖沓"。

用法：python scurve_check.py
"""

import sys
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

TICK_MS = 10
VMAX_K = TICK_MS // 10
ACCEL_DIV = 1000000 // (TICK_MS * TICK_MS * 100)
JERK_STEPS = 6


def run(dist_deg, speed_dps, accel_dps2, tick_ms=TICK_MS, jerk_steps=JERK_STEPS):
    target = int(round(dist_deg * 100))          # 0.01°
    pos = 0
    vel = 0
    acc = 0
    vmax = max(1, int(speed_dps) * VMAX_K)
    accel = max(1, int(accel_dps2) // ACCEL_DIV)
    jerk = max(1, accel // jerk_steps)

    peak_v = 0
    max_pos = 0
    ticks = 0
    ticks_at_target = None

    while ticks < 20000:
        ticks += 1
        rem_signed = target - pos
        if rem_signed == 0:
            vel = 0
            acc = 0
            if ticks_at_target is None:
                ticks_at_target = ticks
                break
            continue

        rem = abs(rem_signed)
        acc_tgt = -accel if (rem * 2 * accel <= vel * vel) else accel

        if acc < acc_tgt:
            acc = min(acc + jerk, acc_tgt)
        elif acc > acc_tgt:
            acc = max(acc - jerk, acc_tgt)

        vel = vel + acc
        if vel < 0:
            vel = 0
        if vel > vmax:
            vel = vmax

        if vel > rem:
            vel = rem
        if vel == 0:
            vel = 1

        pos += vel if rem_signed > 0 else -vel
        peak_v = max(peak_v, vel)
        max_pos = max(max_pos, pos)

    return {
        "ticks": ticks,
        "ms": ticks * tick_ms,
        "pos_deg": pos / 100.0,
        "target_deg": target / 100.0,
        "overshoot_deg": max(0.0, (max_pos - target) / 100.0) if target > 0 else 0.0,
        "peak_dps": peak_v * 0.01 * (1000.0 / tick_ms),   # 0.01°/拍 → °/s
    }


CASES = [
    ("大臂 90→130 (180°/s)",   40,  180,  900),
    ("大臂 90→170 (180°/s)",   80,  180,  900),
    ("底座 90→150 (180°/s)",   60,  180,  900),
    ("夹爪 25→135 (600°/s)",  110,  600, 4000),
    ("微调 1°",                 1,  180,  900),
    ("微调 0.5°",             0.5,  180,  900),
]

print(f"TICK={TICK_MS}ms  VMAX_K={VMAX_K}  ACCEL_DIV={ACCEL_DIV}  JERK_STEPS={JERK_STEPS}\n")
print(f"{'动作':<26}{'耗时ms':>8}{'到位°':>9}{'过冲°':>9}{'峰值°/s':>10}")
print("-" * 64)
worst_overshoot = 0.0
bad = []
for name, d, sp, ac in CASES:
    r = run(d, sp, ac)
    worst_overshoot = max(worst_overshoot, r["overshoot_deg"])
    ok = (abs(r["pos_deg"] - r["target_deg"]) < 0.005) and r["overshoot_deg"] < 0.01
    if not ok:
        bad.append(name)
    print(f"{name:<26}{r['ms']:>8}{r['pos_deg']:>9.2f}{r['overshoot_deg']:>9.3f}{r['peak_dps']:>10.0f}")

print("-" * 64)
print(f"最大过冲: {worst_overshoot:.3f}°")
print("结论: " + ("全部通过 —— 无过冲、能准确停在目标" if not bad else f"异常: {bad}"))