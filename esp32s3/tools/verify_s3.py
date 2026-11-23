#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_s3.py —— ESP32-S3 上位机 + STM32 下位机 全链路自检

用法：
    python verify_s3.py                       # 默认 http://172.19.62.150
    python verify_s3.py --esp 192.168.4.150
    python verify_s3.py --no-motion           # 只读检查，不发会让机械臂动的命令

检查项（共 13 项）：
    1. 网页可访问且是机械臂控制台
    2. /api/state 返回合法 JSON，字段齐全
    3. 固件指纹 fw 是 s3-*（确认烧的是 S3 版）
    4. S3 ↔ STM32 链路在线（link=1）
    5. 切换远程模式生效（mode=1）
    6. 姿态命令被接受（/api/pose）
    7. 姿态回读与目标一致（±3°）
    8. 夹爪闭合/张开（angles[5]）
    9. /api/query 正常
   10. /api/stop 正常
   11. 坐标控制可达点返回 ok=1 且偏差 < 0.5cm
   12. 坐标控制不可达点返回 ok=0 且带 err
   13. cmd 计数单调递增（命令确实被 STM32 收到）

退出码：0 全过，1 有失败。
"""

import argparse
import json
import sys
import time
import urllib.request
import urllib.error

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

OK = "PASS"
NG = "FAIL"
SKIP = "SKIP"

results = []


def get(base, path, timeout=3.0):
    """GET → (status, text)；网络异常时抛出，由调用方判定失败。"""
    url = base.rstrip("/") + path
    req = urllib.request.Request(url, headers={"User-Agent": "verify_s3/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode("utf-8", "replace")


def get_json(base, path, timeout=3.0):
    code, text = get(base, path, timeout)
    try:
        return code, json.loads(text)
    except Exception:
        return code, None


def record(name, ok, detail=""):
    results.append((name, OK if ok else NG, detail))
    print("  [%s] %s%s" % (OK if ok else NG, name, ("  — " + detail) if detail else ""))


def state(base):
    code, js = get_json(base, "/api/state")
    if code != 200 or not isinstance(js, dict):
        return None
    return js


def wait_angles(base, target, tol=3, timeout=4.0):
    """等机械臂走到目标角度（轮询 state）。返回 (ok, 实际角度, 用时)。"""
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        js = state(base)
        if js:
            last = js.get("angles")
            if isinstance(last, list) and len(last) == 6:
                if all(abs(last[i] - target[i]) <= tol for i in range(6)):
                    return True, last, time.time() - t0
        time.sleep(0.15)
    return False, last, time.time() - t0


def main():
    ap = argparse.ArgumentParser(description="ESP32-S3 + STM32 全链路自检")
    ap.add_argument("--esp", default="172.19.62.150", help="S3 的 IP（默认 172.19.62.150）")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--no-motion", action="store_true", help="只做只读检查，不发动作命令")
    args = ap.parse_args()

    base = "http://%s:%d" % (args.esp, args.port)
    print("目标: %s" % base)
    print("模式: %s\n" % ("只读" if args.no_motion else "完整（会让机械臂动作）"))

    # ---------- 1. 网页 ----------
    print("[1] 网页与静态资源")
    try:
        code, html = get(base, "/", args.timeout)
    except Exception as e:
        record("网页可访问", False, "连不上：%s" % e)
        print("\n无法连接，后续检查跳过。请确认：①手机/电脑连的是同一个热点 "
              "②S3 已烧录并上电 ③IP 是 %s" % args.esp)
        return finish()
    record("网页可访问", code == 200, "HTTP %d，%d 字节" % (code, len(html)))
    record("是机械臂控制台", ("机械臂" in html) or ("robot" in html.lower()),
           "含标题" if "机械臂" in html else "未找到标题")

    # ---------- 2. state 结构 ----------
    print("\n[2] 状态接口")
    js = state(base)
    ok = isinstance(js, dict)
    record("/api/state 返回 JSON", ok)
    if ok:
        need = ["mode", "angles", "link"]
        miss = [k for k in need if k not in js]
        record("字段齐全", not miss, "缺 %s" % miss if miss else "mode/angles/link 都在")
        ang = js.get("angles")
        record("angles 是 6 个整数", isinstance(ang, list) and len(ang) == 6,
               str(ang) if ang else "无")
        fw = js.get("fw", "")
        record("固件指纹是 S3 版", fw.startswith("s3-"), "fw=%r" % fw)
        record("S3 ↔ STM32 链路在线", js.get("link") == 1,
               "link=%s（0 表示 STM32 没回状态）" % js.get("link"))

    # ---------- 3. 只读到此为止 ----------
    if args.no_motion:
        print("\n[--no-motion] 跳过动作类检查")
        return finish()

    # ---------- 4. 远程模式 ----------
    print("\n[3] 模式切换")
    code, _ = get(base, "/api/mode?remote=1")
    time.sleep(0.4)
    js = state(base) or {}
    record("切到远程模式", js.get("mode") == 1, "mode=%s" % js.get("mode"))

    # ---------- 5. 姿态命令 ----------
    print("\n[4] 姿态控制")
    target = [90, 90, 90, 90, 90, 25]
    code, txt = get(base, "/api/pose?angles=%s&ms=0" % ",".join(str(v) for v in target))
    record("接受姿态命令", code == 200, "HTTP %d %s" % (code, txt[:40]))
    ok, ang, dt = wait_angles(base, target, tol=3)
    record("机械臂走到目标（±3°）", ok, "实际 %s，用时 %.1fs" % (ang, dt))

    # ---------- 6. 夹爪 ----------
    print("\n[5] 夹爪")
    get(base, "/api/gripper?open=0")
    ok, ang, _ = wait_angles(base, [90, 90, 90, 90, 90, 135], tol=4)
    record("夹爪闭合到 135°", ok, "angles[5]=%s" % (ang[5] if ang else "?"))
    get(base, "/api/gripper?open=1")
    ok, ang, _ = wait_angles(base, [90, 90, 90, 90, 90, 25], tol=4)
    record("夹爪张开到 25°", ok, "angles[5]=%s" % (ang[5] if ang else "?"))

    # ---------- 7. 查询 / 急停 ----------
    print("\n[6] 查询与急停")
    code, txt = get(base, "/api/query")
    record("/api/query", code == 200, "HTTP %d %s" % (code, txt[:30]))
    time.sleep(0.3)
    before = (state(base) or {}).get("cmd", 0)
    code, txt = get(base, "/api/stop")
    record("/api/stop", code == 200, "HTTP %d %s" % (code, txt[:30]))

    # ---------- 8. 坐标控制 ----------
    print("\n[7] 坐标控制（IK）")
    code, js2 = get_json(base, "/api/cart?x=35&y=0&z=20&pitch=0&ms=600")
    ok = code == 200 and isinstance(js2, dict) and js2.get("ok") == 1
    dev = js2.get("dev") if isinstance(js2, dict) else None
    record("可达点 (35,0,20) 求解成功", ok, "angles=%s dev=%s" %
           (js2.get("angles") if isinstance(js2, dict) else "?", dev))
    record("回代偏差 < 0.5cm", isinstance(dev, (int, float)) and dev < 0.5,
           "dev=%.3f cm" % dev if isinstance(dev, (int, float)) else "无 dev 字段")

    code, js3 = get_json(base, "/api/cart?x=0&y=0&z=20&pitch=0")
    ok = isinstance(js3, dict) and js3.get("ok") == 0
    record("不可达点正确拒绝", ok,
           "err=%r" % (js3.get("err") if isinstance(js3, dict) else "?"))
    if isinstance(js3, dict):
        record("拒绝时给出中文原因", bool(js3.get("err")), "")

    # ---------- 9. 计数器 ----------
    print("\n[8] 命令计数")
    time.sleep(0.3)
    after = (state(base) or {}).get("cmd", 0)
    record("cmd 计数在增长（命令确实到了 STM32）", after > before,
           "%s → %s" % (before, after))

    # ---------- 收尾：回到中位、交还体感 ----------
    print("\n[9] 复位")
    get(base, "/api/pose?angles=90,90,90,90,90,25&ms=800")
    get(base, "/api/mode?remote=0")
    print("  已回中位并把模式交还体感（remote=0）")

    return finish()


def finish():
    print("\n" + "=" * 56)
    ng = [r for r in results if r[1] == NG]
    print("合计 %d 项，通过 %d，失败 %d" % (len(results), len(results) - len(ng), len(ng)))
    if ng:
        print("\n失败项：")
        for name, _st, detail in ng:
            print("  - %s %s" % (name, ("(%s)" % detail) if detail else ""))
        print("\n排查顺序：链路 link → 串口接线/共地 → STM32 是否烧录 → 固件 fw 指纹")
    else:
        print("全部通过 ✅")
    print("=" * 56)
    return 0 if not ng else 1


if __name__ == "__main__":
    sys.exit(main())