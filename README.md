# 双芯六轴机械臂（STM32F407 + ESP32-S3）

手机浏览器里输坐标或者拖滑块就能控制这台六轴舵机机械臂，也能用 MPU6050 体感跟着手动；
STM32 的固件可以在网页上直接升级，不用拆下来插 ST-Link。

- 下位机（`f407/`）：STM32F407VET6，跑 FreeRTOS，管六路 PWM、S 曲线插值、串口协议、
  看门狗、Bootloader 和 IAP。应用 41 KB，引导程序 2.8 KB。
- 上位机（`esp32s3/`）：ESP32-S3，跑 ESP-IDF 5.5，管 WiFi、内置网页、逆运动学解算和
  固件转发，固件 882 KB。
- 两块板之间是一行一条的文本协议（115200 8N1），带头心跳、逐帧 ACK 和超时重发。

## 目录

```
f407/       STM32F407VET6 工程（Keil MDK）
  BSP_Drivers/  舵机轨迹、MPU6050、OLED、串口协议、Bootloader/IAP、备份回滚、看门狗
  Core/         CubeMX 生成的外设初始化
  bootloader/   引导程序（独立工程，先跑它再跳应用）
  MDK-ARM/      smart hand.uvprojx（应用）、bootloader.uvprojx（引导程序）
  docs/         接线指南、接线图
esp32s3/    ESP32-S3 工程（ESP-IDF 5.5）
  main/         网页服务、串口协议、逆运动学 arm_ik.h、内嵌 index.html
  tools/        verify_s3.py（19 项端到端检查）、find_s3.py（UDP 广播找 IP）
  build.bat     编译 / build.bat flash COM3 烧录
```

## 编译与烧录

STM32（Keil）：打开 `f407/MDK-ARM/smart hand.uvprojx`，F7 编译、F8 下载。
引导程序同理用 `bootloader.uvprojx`。**第一次**必须依次烧两份，顺序不能反：

1. `bootloader.hex` → `0x08000000`
2. 应用 → `0x08010000`（编译后自动生成 `hand_app.bin`）

之后升级就不用线了：网页底部选 `hand_app.bin` 上传，或者

```
python tools/iap_upload.py --http http://<ESP32 的 IP>/
```

ESP32-S3：`cd esp32s3 && build.bat`，插上 USB 后 `build.bat flash COM3`。

接线见 `f407/docs/接线指南.md`：舵机 PA0~PA3、PA6、PA7，MPU6050 走 PB10/PB11，
OLED 走 PB6/PB7，ESP32-S3 的 GPIO17→PA10、GPIO18←PA9，必须共地。

## 串口协议

| 命令 | 含义 |
|---|---|
| `P,<6 个角度>[,ms]` | 六轴目标角，可带到位时长 |
| `M,0/1` | 切体感 / 远程 |
| `G,0/1` | 夹爪开 / 合 |
| `S` | 急停（目标设成当前位置） |
| `Q` | 查询状态 |
| `H,0/1` | 周期心跳开关 |
| `C,<轴>,<最小计数>,<最大计数>` | 单轴两点标定，存 W25Q16 |
| `U,<长度>,<CRC>` / `W,<偏移>,<hex>` / `E,<CRC>` | IAP 三段式 |

回包：`OK`、`ERR`、`ST,...`（整数度）、`ST2,...`（十分之一度）、`VR,f4-v7`、`HB,...`。

## 已知问题

- 舵机是开环的，上电第一件事就是回中位；断电时手臂停得离 90° 越远，上电那一下越明显。
- 手机热点每次开出来的网段不一定一样，所以 S3 走 DHCP，IP 用 UDP 9999 广播查：
  `python esp32s3/tools/find_s3.py`。
- 逆运动学的四个几何参数（底座高度、三段连杆）是从实物上量的，换一台臂要改 `arm_ik.h`
  里的 `ARM_D1 / ARM_A1 / ARM_A2 / ARM_A3`，否则算出来的角度不准。
- 舵机电源一定要独立（5 V / ≥5 A，加 1000 µF），从开发板取电会掉压复位。