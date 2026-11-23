#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""find_s3.py —— 一秒钟找出 ESP32-S3 的 IP（不依赖 mDNS）

原理：S3 固件每 3 秒往 255.255.255.255:9999 广播一行
      STM32ARM;ip=192.168.1.23;fw=s3-2-mode;link=1
本脚本在电脑上监听这个端口，把收到的地址打印出来。

用法：
    python find_s3.py            # 听 6 秒
    python find_s3.py -t 15      # 听 15 秒（广播有丢包时多听一会）
"""
import argparse
import socket
import sys
import time

PORT = 9999
MAGIC = "STM32ARM"


def listen(seconds: float):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except OSError:
        pass
    s.bind(("", PORT))
    s.settimeout(0.5)

    found = {}
    deadline = time.time() + seconds
    print("正在监听 UDP %d 端口（%.0f 秒）..." % (PORT, seconds))

    while time.time() < deadline:
        try:
            data, addr = s.recvfrom(512)
        except socket.timeout:
            continue
        except OSError:
            break
        text = data.decode("utf-8", "replace").strip()
        if not text.startswith(MAGIC):
            continue
        found[addr[0]] = text

    s.close()
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--time", type=float, default=6.0, help="监听秒数，默认 6")
    args = ap.parse_args()

    found = listen(args.time)
    print()

    if not found:
        print("没收到广播。可能的原因：")
        print("  1) ESP32-S3 没上电 / 没烧新固件（旧固件没有广播功能）")
        print("  2) 电脑没连到同一个热点（现在连的是别的 WiFi）")
        print("  3) 热点隔离了广播（少数手机热点会这样）→ 去  设置→个人热点→已连接的设备  里看 IP")
        return 1

    for src_ip, line in sorted(found.items()):
        parts = dict(kv.split("=", 1) for kv in line.split(";")[1:] if "=" in kv)
        ip = parts.get("ip", src_ip)
        link = parts.get("link", "?")
        state = "STM32 已连上" if link == "1" else "STM32 未回状态"
        print("  找到机械臂: http://%s/" % ip)
        print("              fw=%s  %s" % (parts.get("fw", "?"), state))
    print()
    print("直接把上面的地址贴进浏览器即可。")
    return 0


if __name__ == "__main__":
    sys.exit(main())