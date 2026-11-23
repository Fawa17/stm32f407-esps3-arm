# 双芯六轴机械臂 —— 下位机（STM32F407VET6 / FreeRTOS / HAL）

## 项目介绍

实现了网页与体感双模式操控、支持网页一键升级固件的六轴舵机机械臂。

- **STM32F407 端**：六路舵机 PWM 输出、100 Hz 轨迹插值、体感姿态解算、多任务调度与看门狗守护。
- **ESP32-S3 端**：WiFi 通信、网页控制台与接口服务、六轴逆运动学解算、固件升级包接收与转发。

## 技术栈

C / FreeRTOS / STM32F407 / ESP32-S3 / ESP-IDF / UART + DMA / I2C / SPI / TIM PWM / Bootloader / IAP / CRC32 / IWDG / Keil MDK

## 主要工作与成果

1. **实时任务划分**：控制、体感、命令解析、显示、看门狗拆为 5 个 FreeRTOS 任务，按实时性分配优先级与栈深（控制任务 AboveNormal、显示与看门狗 Low，栈 192/384/384/320/256 字），共享关节角用互斥量保护；控制任务只做寄存器级输出，100 Hz 节拍不会被 I2C 读传感器（单次约 1.6 ms）与串口解析打断；另配 6 s 独立看门狗 + 四任务心跳，单任务卡死自动复位。

2. **PWM 驱动与运动平滑**：TIM2/TIM3 六通道输出 50 Hz PWM，定时器 84 MHz 下计数分辨率做到 0.5 µs（PSC=41、ARR=39999）；2 µs 写死区消除浮点尾数引起的 100 Hz 微抖，S 曲线 jerk 限幅实现六轴同步平滑启停，贴限位自动降速至 30%。调试中修掉定时器预装载导致首帧脉宽 1500→3000 µs 的上电冲击（改为先写满六路 CCR、再置 EGR.UG 一次生效）。

3. **通信协议与串口驱动**：自定义 UART 文本协议承载姿态、模式、夹爪、心跳、标定、IAP 六类指令；USART1 + DMA 环形缓冲收包，按行解析、超长帧丢弃与重同步；逐条 ACK + 300 ms 超时重发 + 500 ms 心跳 + 3 s 断链判定，两端带固件版本握手（`VR,<ver>`），丢帧可定位、断链自恢复。

4. **引导程序与 IAP 升级**：手写 2.8 KB 引导程序（业务逻辑不依赖 HAL，Flash 操作走自写的 `FlashDrv` 层），Flash 四区隔离（引导区 64 KB / 应用区 192 KB / 暂存区 128 KB / 元数据区）；新固件先落暂存区，CRC32 与向量表 SP/PC 合法性校验通过才搬运，"待安装"标记最后写入，掉电可重搬；应用健康启动后盖章并把当前镜像备份到板载 W25Q16，连续 3 次启动未盖章即自动回滚。STM32 侧是 IAP（应用在运行时写 Flash），数据由 ESP32-S3 经 WiFi 收下后转发过来；41 KB 应用从网页发起到升级完成约 20 s。

## 硬件

- 主控：STM32F407VET6（512K Flash / 192K RAM）
- 网络：ESP32-S3（N16R8，16 MB Flash + 8 MB PSRAM），跑 ESP-IDF v5.5
- 姿态：MPU6050（I2C2，PB10/PB11，100 kHz）
- 显示：0.96" SSD1306 OLED（I2C1，PB6/PB7，400 kHz），显示链路、模式、六路实际下发脉宽、姿态角
- 存储：板载 W25Q16（SPI2，PB12~PB15），存标定参数与固件备份
- 执行：6 × MG996R，PA0/PA1/PA2/PA3（TIM2 CH1~4）+ PA6/PA7（TIM3 CH1/CH2）
- 串口：S3 `GPIO17→PA10`、`GPIO18←PA9`，115200 8N1，必须共地
- 供电：舵机独立 5V/5A 以上 + 1000 µF，与主控单点共地

接线细节见 `docs/接线指南.md`，图见 `docs/接线图.png`。

## 目录

```
BSP_Drivers/   自己写的驱动：RobotArm（轨迹/PWM）、MPU6050、NetLink（串口协议）、OLED、
               W25Qxx、Param（参数持久化）、Backup（回滚）、IAP、FlashDrv、Watchdog
Core/          CubeMX 生成的初始化代码 + main.c
bootloader/    引导程序（0x08000000 ~ 0x0800FFFF）
MDK-ARM/       Keil 工程、flash_first.bat（首次烧录）、dump_meta.bat（读升级元数据）
docs/          接线指南、接线图、自查记录
tools/         构建/自检/离线验算脚本
```

上位机是旁边的 `C:\STM32\hand_s3`（ESP-IDF 工程），网页在 `main/www/index.html`。

## 编译

Keil 打开 `MDK-ARM/smart hand.uvprojx` 按 F7；引导程序是 `MDK-ARM/bootloader.uvprojx`。
命令行编译（必须把 armcc 加进 PATH，否则 After-Make 的 fromelf 会失败）：

```bat
set PATH=C:\Keil_v5\ARM\ARMCC\bin;%PATH%
"C:\Keil_v5\UV4\UV4.exe" -b "MDK-ARM\smart hand.uvprojx" -j0 -o build.log
```

ESP32-S3 这边：

```bat
cd C:\STM32\hand_s3
build.bat            :: 编译
build.bat flash 3    :: 烧到 COM3
```

## 烧录

第一次必须用 ST-Link 烧两份，顺序不能反：

1. `MDK-ARM\bootloader\bootloader.hex` → `0x08000000`（只需烧这一次）
2. `MDK-ARM\smart hand\smart hand.hex` → `0x08010000`

之后不用再插调试器。浏览器打开 `http://<S3 的 IP>/` → 页面底部「固件升级」→ 选 `hand_app.bin` 上传，
或者 `python tools\iap_upload.py --http http://<S3 的 IP>/`。
数据走向是：浏览器 →（WiFi）→ ESP32-S3 →（UART）→ STM32 的 IAP。也就是说 **STM32 侧是 IAP，
"空中"那一段发生在 ESP32-S3 上**；S3 自己的固件更新才是真正的 OTA（esp_ota 双分区）。
`MDK-ARM\flash_first.bat` 是一键烧两份的脚本。

## 串口协议

每行以 `\n` 结尾，命令字母大小写都认：

| 命令 | 含义 | 回包 |
|---|---|---|
| `P,<a0>..<a5>[,<ms>]` | 六轴目标角（度，可带一位小数），可选完成时间 | `OK` / `ERR` |
| `M,<0\|1>` | 模式：0 体感 / 1 远程 | `OK` |
| `G,<0\|1>` | 夹爪 开 / 合 | `OK` |
| `S` | 急停（目标=当前位置） | `OK` |
| `Q` | 查询 | `VR,<ver>` + `ST2,...` |
| `H,<0\|1>` | 500 ms 周期心跳开关 | `OK` |
| `C,<axis>,<tmin>,<tmax>` | 单轴两点标定，落 W25Q16 | `OK` / `ERR` |
| `U,<size>,<crc>` / `W,<off>,<hex>` / `E,<crc>` | IAP 升级三段 | `OK` / `ERR` |

状态行：`ST,<mode>,<a0..a5>[,<alive>]`、`ST2,...`（0.1° 精度，页面回显用）、`HB,...`、`VR,<ver>`。

## 目前的样子

- 上电：六路按 200 ms 间隔逐路启动到 1500 µs 中位，避免六轴同时起冲
- 体感：MPU6050 姿态映射到六轴（摇一摇切夹爪默认关闭，容易误触发）
- 网页：六个滑块 + 坐标输入 + 预设动作 + 急停 + 模式切换，手机连热点直接用
- 坐标控制：IK 在 ESP32-S3 上算（工具姿态 1° 扫描 + 底座正反两支 + 正解回代校验，超差 0.5 cm 拒绝）
- 轨迹：100 Hz 浮点 S 曲线，过冲 0.000°，180° 行程 620 ms；末端走直线用笛卡尔插补，偏差 0.16~0.79 cm
- 升级：暂存区 + CRC32 + 元数据标记，装失败连续 3 次起不来自动从 W25Q16 回滚

固件版本：STM32 `f4-v7`，ESP32-S3 `s3-15-rxfast`。

## 排障记录（几个真实踩过的坑）

- **点一下夹爪，整条手臂甩过去**：ESP32-S3 侧把"镜像后的输出角"存进了"逻辑角"变量，下发时又镜像一次，J1~J3 变成 `180 - x`。统一只保存逻辑角，镜像只在发送函数里做一次。
- **引导程序上电即 HardFault**：自写的 `SystemInit()` 留空导致 FPU 未使能，armcc 启动代码执行 `VMSR FPSCR` 触发 UsageFault（`ICSR=0x0803`，栈帧里 PC 指向 `_fp_init`）。补 `SCB->CPACR |= (0xFUL << 20)` 后正常启动。
- **W25Q16 恒判不在位**：JEDEC ID 判据写成 `(id & 0xFF00FF) == 0x00EF00`，对真芯片 `0xEF4015` 永远为假，参数持久化与备份静默失效；改成只比对厂商字节 `(id >> 16) & 0xFF == 0xEF`。
- **网页升级每帧白等 100 ms**：`uart_read_bytes(len=256, timeout=100ms)` 是"读满或超时才返回"，而 STM32 每帧只回 3 字节，840 帧累计约 84 s，正好撞上 90 s 上限；改为 `uart_get_buffered_data_len()` 只读实际到达的字节，单帧降到 19 ms。

## 已知问题 / 还没做

- 舵机是开环的，没有位置反馈；堵转只能靠电流采样（板子上还没加采样电阻）
- 上电仍会有一次跳到中位的小动作，彻底解决要把上次姿态存起来复用（`ROBOTARM_BOOT_FROM_FLASH`，还没接）
- 可达域内圈（靠近底座 16~18 cm 以内）因为爪子长、肘只能折 80° 够不到
- 没有 `/api/latency` 这类诊断接口，延迟目前只能靠抓包估
- CubeMX 生成的注释与手写注释混在一起，没整理

## 自检

不需要人肉点的几个脚本，改完代码跑一遍就知道有没有回归：

```bat
C:\Ana\python.exe C:\STM32\hand\sim\verify.py   :: 30 项：URDF / 限位 / IK 三方一致（在 hand 工程里）
python tools\boot_state_check.py                 :: 5 个回滚场景的状态机重放
python tools\scurve_check.py                     :: S 曲线 1:1 复刻，看时间与过冲
```

上位机的端到端检查在 `C:\STM32\hand_s3\tools\verify_s3.py`（需板子上电联网）。