# 双芯六轴机械臂 —— 上位机（ESP32-S3 / ESP-IDF 5.5.5 / 纯 C）

STM32 负责硬实时（PWM / 体感 / 网络协议解析），本工程负责网络层与 IK 解算。
**接口与 ESP8266 版完全相同**，网页、`server/robot_server.py`、`sim/` 都不用改。

## 代码结构

按 ESP-IDF 的组件方式分层，`main/` 只留入口，其余各层是独立组件（依赖单向，不互相包含）：

| 位置 | 作用 | 依赖 |
|---|---|---|
| `main/app_main.c` | 启动编排：NVS → 镜像恢复 → 串口 → WiFi → HTTP → 发现广播 → 开机握手 | 下面全部 |
| `components/arm_config` | 全部可调参数（WiFi、串口引脚、端口、速度、夹爪端点、IAP 分帧） | — |
| `components/arm_ik` | 逆运动学，纯头文件，和 STM32 侧同一套公式 | — |
| `components/arm_bus` | 串口 API 的单薄声明层，让 state 和 link 不互相依赖 | — |
| `components/arm_state` | 关节状态、镜像换算、轨迹下发、笛卡尔直线插补 | config / ik / bus |
| `components/arm_link` | 与 STM32 的串口协议：发送、逐行解析、ACK 重发、心跳 | config / bus / state |
| `components/arm_net` | WiFi 连接、AP 兜底、UDP 设备发现、IP 查询 | config / link |
| `components/arm_web` | 13 个 HTTP 接口、OTA、STM32 固件上传（内含 `www/index.html`） | config / ik / state / link / net |

依赖关系：`config / ik / bus` ← `state` ← `link` ← `net` ← `web` ← `main`。

## 硬件

| 项 | 说明 |
|---|---|
| 模组 | ESP32-S3-WROOM-1 **N16R8**（16MB Flash / 8MB Octal PSRAM） |
| 串口链路 | **默认** `GPIO17 (U1TXD)` → STM32 `PA10`，`GPIO18 (U1RXD)` ← STM32 `PA9`，两端 **GND 必须相连** |
| 串口链路（可选） | 板载 `TXD0/RXD0`（`GPIO43`/`GPIO44`）：把 `app_config.h` 里 `LINK_ON_UART0` 改成 `1` 重编。见下方"用板载 TXD0/RXD0" |
| 供电 | 5V/1A 给开发板（S3 峰值电流比 ESP8266 大），不要从 STM32 板取 |
| 烧录 | 板载 USB（USB-Serial-JTAG，GPIO19/20）插上即可，无需 USB-TTL |
| 网络 | WiFi STA 连手机热点 `ESP`，IP 由热点 DHCP 分配；每 3 秒 UDP 广播自己（`tools\find_s3.bat` 可直接查出 IP） |

## 编译 / 烧录

```bat
build.bat                    :: 只编译
build.bat flash 3            :: 编译 + 烧到 COM3 + 打开监视器
build.bat menuconfig         :: 改配置
```

产物：`build\hand_s3.bin`（约 845KB，4MB 分区用了 21%）。
分区表 `partitions.csv`：`ota_0` / `ota_1` 各 4MB，最后 `storage` 约 7.9MB（留着放动作组）。

## HTTP 接口（与旧版一致）

| 接口 | 说明 |
|---|---|
| `GET /` | 控制台网页（内嵌在 `main/www/index.html`） |
| `GET /api/state` | `{"mode":0/1,"angles":[6],"link":0/1,"cmd":N,"err":N,"fw":"s3-2-mode"}` |
| `GET /api/pose?angles=90,90,90,90,90,25&ms=800` | 六轴角度（ms=0 走默认速度） |
| `GET /api/cart?x=35&y=0&z=20&pitch=auto&ms=0` | 坐标 → 本机 IK → 角度 |
| `GET /api/line?x=40&y=0&z=16&pitch=0&ms=800` | **笛卡尔直线插补**：从上次 `/api/cart` 的目标点沿直线走到这里（每 50ms 解一次 IK） |
| `GET /api/jog?axis=0&deg=120` | 单轴点动（标定方向用） |
| `GET /api/gripper?open=1` | 夹爪开/合 |
| `GET /api/mode?remote=1` | 远程 / 体感模式（**上电默认 remote=1（网络控制）**，切换后会被记住，STM32 重启也会恢复成这个模式） |
| `GET /api/stop` | 急停 |
| `GET /api/query` | 让 STM32 回一次状态 |
| `POST /update` | OTA：把 `hand_s3.bin` 直接 POST 上来 |

OTA 示例：

```bat
curl --data-binary "@build\hand_s3.bin" http://172.19.62.150/update
```

### 直线插补有多直（`tools/line_check.py` 实测）

关节空间直接插值时末端走的是弧线；`/api/line` 每 50ms 解一次 IK，末端才真的走直线。
偏差 = 实际末端轨迹到理想直线的最大垂直距离（同样 1° 下发分辨率）：

| 直线（cm） | 直线插补 | 关节空间插值 |
|---|---|---|
| (28,0,10) → (38,0,26) | **0.33 cm** | 2.19 cm |
| (30,-12,12) → (30,12,20) | **0.64 cm** | 2.62 cm |
| (33,0,6) → (33,0,28) | **0.16 cm** | 2.10 cm |
| (25,0,25) → (35,0,25) | **0.79 cm** | 1.15 cm |

剩余的零点几厘米主要来自"角度按 1° 下发"（40cm 处 1° ≈ 0.7cm），不是算法误差。

## 串口协议（不变）

```
下发  P,a0,a1,a2,a3,a4,a5[,ms] | M,0|1 | G,0|1 | S | Q | H,0|1
上报  OK | ERR | ST,mode,a0..a5,alive | HB,mode,a0..a5
115200 8N1，一行一条，`\n` 结尾
```

## 用板载 TXD0/RXD0 连 STM32（可选）

开发板上那组 `TXD0/RXD0`（ESP32-S3 默认 = `GPIO43`/`GPIO44`）也能当链路，但要知道代价：

| 项 | 说明 |
|---|---|
| 接线 | `TX0`(GPIO43) → STM32 `PA10`；`RX0`(GPIO44) ← STM32 `PA9`；GND 相连（和 UART1 一样是交叉） |
| 好处 | 开发板 UART 排针通常只引出这四个脚，接线最省事 |
| 代价 1 | UART0 是 IDF 的 console：日志会混进协议流 → 代码里已自动 `esp_log_level_set("*", ESP_LOG_NONE)` 关掉 |
| 代价 2 | 每次复位，ROM 启动日志会从 TXD0 打出来，STM32 会收到一行乱码并回一次 `ERR`（无害） |
| 代价 3 | 若这块板的 USB 口经过 CH343/CP2102 桥接到 GPIO43/44，插上电脑时 PC 和 STM32 会抢同一对线，**不要同时连** |
| 代价 4 | `idf.py monitor` 看到的是协议流量，不再是可读日志 |

打开方式：把 `app_main.cpp` 里 `#define LINK_ON_UART0 0` 改成 `1`，然后 `build.bat flash <COM号>`。
用 UART0 时驱动由 IDF 的 console 装好，代码不会重复 `uart_driver_install`，直接复用它的缓冲读。

## 与 ESP8266 版的差异

| 项 | ESP8266 版 | 本版 |
|---|---|---|
| SDK | Arduino | ESP-IDF 5.5.5（CMake + idf.py），**纯 C（`app_main.c`）** |
| IK | `arm_ik.h` | **同一份文件**，已改成 C 兼容（struct 加 typedef、引用改指针） |
| mDNS | 有（`stm32-arm.local`） | **去掉**：IDF 5.5 把 mdns 移出主仓库，要先联网装组件；反正你的热点屏蔽组播 |
| MQTT | 可选（`MQTT_HOST` 留空即关） | **去掉**：没有 broker，走 HTTP 就够（要就加 `esp-mqtt` 组件） |
| AP 回退 | `ENABLE_AP` 开关 | **去掉**：改成纯 STA + 固定 IP，你说过"别自己连自己" |
| OTA | espota（UDP 8266，`ota_esp.bat`） | **HTTP POST /update**，浏览器/curl 都行 |
| 调试口 | `Serial1`（GPIO2） | 板载 USB 串口（`idf.py monitor`） |
| 双核 | 单核 | 协议栈 core0，HTTP/IK core1（网页轮询不再拖慢 OTA） |

## 功能对照（ESP8266 → S3）

| 功能 | ESP8266 版 | S3 版 | 说明 |
|---|---|---|---|
| 网页控制台 | ✅ | ✅ | 同一份 `index.html`（含 6 滑块、坐标控制、预设、急停、体感/远程按钮） |
| `/api/state` | ✅ | ✅ | S3 多返回 `cmd`/`err` 计数 |
| `/api/pose` | ✅ | ✅ | `angles=` + 可选 `ms` |
| `/api/cart` | ✅ | ✅ | 坐标 → IK → `P,...`，返回 `dev` 偏差 |
| `/api/jog` | ✅ | ✅ | 单轴点动（标定用） |
| `/api/gripper` `/api/mode` `/api/stop` `/api/query` | ✅ | ✅ | 语义一致 |
| 串口协议 `P/M/G/S/Q/H` | ✅ | ✅ | STM32 侧不用改 |
| 心跳补发（断链自动重连） | ✅ | ✅ | 1s 周期，6s 无状态补发 `H,1`/`Q` |
| OTA | espota UDP | HTTP `/update` | S3 版更简单，`curl --data-binary @bin` 即可 |
| MQTT | 可选 | ❌ | 需要再加（`idf.py add-dependency espressif/esp-mqtt`） |
| AP 自建热点 | 有（默认关） | ❌ | 已删；要的话可加回 |
| mDNS | 有 | ❌ | 你手机热点屏蔽组播，用 IP 直连 |

## 主线状态

- [x] 工程可编译（纯 C，`build.bat` 通过，`hand_s3.bin` 0xD3510 字节）
- [ ] 上板烧录 + 点亮（等板子接上 USB）
- [ ] 与 STM32F407 联调（F407 侧固件还没生成）
- [ ] 网页加"固件升级"入口（`/update` 已可用，页面按钮待加）

## 升级 STM32（F407）固件：`POST /update_stm32`

ESP32-S3 当"转接头"：网页/PC 把 `hand_app.bin` POST 上来，S3 拆成 40 字节一帧，
按 `U,<size>,0` → `W,<offset>,<hex>` ×N → `E,<crc32>` 发给 STM32，STM32 收完自己重启，
由引导程序把暂存区搬到应用区。

```bash
# 电脑
python C:\STM32\hand\tools\iap_upload.py hand_app.bin --http http://<ESP地址>/

# 或者直接 curl
curl.exe -X POST --data-binary "@C:\STM32\smart hand\MDK-ARM\hand_app.bin" `
  -H "Content-Type: application/octet-stream" http://<ESP地址>/update_stm32
```

返回：`{"ok":1,"bytes":35320,"crc":"0x6C030F1C","note":"STM32 正在重启并安装"}`

注意：
- 上传期间心跳会暂停（`s_busy_iap`），STM32 重启后 S3 会自动补 `H,1` / `M,<期望模式>`；
- `U` 命令要擦 128KB 扇区，**8 秒超时**是故意的，别以为卡死了；
- 第一次仍必须用 ST-Link 烧 `bootloader.hex` + 应用，之后才能纯 IAP。


## 本轮优化（firmware s3-12-opt）

1. **多解择优**：同一点通常有几十组关节解（φ 扫 240° × 两种肘向 × 两种底座朝向）。
   现在按 `越限量×1000 + 姿态偏差 + 关节改动量×0.3` 打分，第三项让手臂选"离当前姿态最近"的解 ——
   在同一个高度附近来回移动时不会再突然翻肘。
2. **边界降速**：STM32 侧目标角离关节限位小于 12° 时，速度按比例降到三成
   （`ROBOTARM_SOFT_ZONE_DEG` / `ROBOTARM_SOFT_PCT` 可调）。中间段不受影响，只有贴边的收尾变温柔。
3. **网页显示末端坐标**：`/api/state` 新增 `xyz`（用当前模型姿态正解算出的末端位置，单位 cm），
   页面状态栏多一行"末端：X Y Z"。
