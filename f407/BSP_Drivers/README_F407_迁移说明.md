# F407 迁移说明（STM32F407VET6 + FreeRTOS）

从 `C:\STM32\hand`（F103 + FreeRTOS）迁移到 `C:\STM32\smart hand`（F407 + FreeRTOS/CMSIS-RTOS2）。
本工程由 CubeMX 生成，业务代码全部放在 `BSP_Drivers\`，`Core\` 里只改了 `main.c` 的 USER CODE 区和 `freertos.c` 的四个任务。

---

## 1. 任务划分（`Core/Src/freertos.c`）

| 任务 | 周期 | 优先级 | 栈 | 干什么 |
|---|---|---|---|---|
| `armTask` | **10ms**（100Hz） | AboveNormal | 192 字 | `RobotArm_Update()`：S 曲线加减速 + 写 CCR（只做寄存器操作） |
| `gestureTask` | 20ms | Normal | 384 字 | `MPU6050_GestureStep()`：读姿态 → 映射关节 → 写目标角度（远程模式自动跳过） |
| `netTask` | 5ms | Normal | 384 字 | `NetLink_Poll()`：解析 USART1(DMA) 收到的 ESP32-S3 命令 |
| `oledTask` | 100ms | Low | 320 字 | `OLED_StatusTask()`：刷状态屏 |
| `wdTask` | 200ms | Low | 256 字 | `Watchdog_AllAlive()` → 喂 IWDG |

- 目标角度由 `gestureTask` / `netTask` 写、`armTask` 读，用 `RobotArm_CreateLock()` 创建的互斥量保护（在 `MX_FREERTOS_Init()` 的 `RTOS_MUTEX` 区调用）。
- `NextWake()` 处理“上一拍超时”的情况：落后就重新对齐，不做追赶式连甩。
- `FreeRTOSConfig.h`：`configTOTAL_HEAP_SIZE = 15360`、`configCHECK_FOR_STACK_OVERFLOW = 2`、`configUSE_MALLOC_FAILED_HOOK = 1`（两个钩子在 `freertos.c` 末尾实现，出问题就停住不再乱动舵机）。
- HAL 时基 = **TIM4**（`stm32f4xx_hal_timebase_tim.c`），SysTick 留给 FreeRTOS（`cmsis_os2.c` 里定义 `SysTick_Handler`）。

### 1.1 看门狗（`BSP_Drivers/Watchdog.c`）

- 直接写 IWDG 寄存器（`PR=4` 即 64 分频、`RLR=1500`），**不依赖 CubeMX 生成 `hiwdg`**，重新生成工程也不会丢；标称 3s（LSI 17~47kHz，实际 2~5.6s）。
- 四个业务任务每轮循环各调一次 `Watchdog_KeepAlive(WD_TASK_xxx)`；`wdTask` 每 200ms 检查一次，**全部按时报到才喂狗**，任何一个卡死就不再喂，让 IWDG 复位整机。
- 期限：arm/gesture/net 各 200ms，oled 800ms（屏幕慢一点不算故障）。
- 调试时把 `Watchdog.h` 的 `WATCHDOG_ENABLE` 改 0，全部退化为空实现（断点停住不会被复位）。

### 1.2 S 曲线加减速（`RobotArm.c` 的 `RobotArm_Update()`）

- 位置单位 0.01°，10ms 一拍。加速度每拍最多变化 `jerk = a / ROBOTARM_JERK_STEPS`，再积分成速度 —— 即**加加速度（jerk）受限**，起停不再「咯噔」。
- 刹车判据仍是 `rem·2a ≤ v²`，另有 `if (v > rem) v = rem;` 兜底，**不会过冲**。
- 离线校验脚本 `tools/scurve_check.py`（与固件同一套整数运算）：40° 用时 370ms、80° 用 590ms、夹爪 110° 用 300ms，**过冲 0.000°**、全部准确停在目标。
- 想更柔：`ROBOTARM_JERK_STEPS` 加大或 `JOINT_ACCEL_DPS2` 减小；想更快：加大 `JOINT_SPEED_DPS`/加速度。

初始化（`main.c` 的 USER CODE 2，在 `osKernelStart()` 之前）：

```
NetLink_Init()          串口先开，回中位那 2 秒里上位机命令也不会丢
OLED_Init()             屏没接就跳过，不影响机械臂
RobotArm_Init()         六路 PWM 分时软启动（每路间隔 200ms）+ 1.5s 线性回中位
MPU6050_GestureInit()   陀螺校准 + 记录体感中位（失败就每秒重试，期间照常收网络命令）
NetLink_Flush()         丢掉启动期间积压的串口数据
```

---

## 2. 引脚表

| 功能 | 引脚 | 外设 |
|---|---|---|
| 底座旋转（轴0） | PA0 | TIM2_CH1 AF1 |
| 大臂俯仰（轴1） | **PA1** | TIM2_CH2 AF1 |
| 小臂俯仰（轴2） | **PA2** | TIM2_CH3 AF1 |
| 腕部俯仰（轴3） | **PA3** | TIM2_CH4 AF1 |
| 腕部旋转（轴4） | **PA6** | TIM3_CH1 AF2 |
| 夹爪开合（轴5） | **PA7** | TIM3_CH2 AF2 |
| MPU6050（I2C2） | PB10=SCL / PB11=SDA | 100kHz |
| OLED（I2C1） | **PB6=SCL / PB7=SDA** | 400kHz |
| ESP32-S3 串口 | PA9=TX / PA10=RX | USART1 115200 8N1 |
| 调试串口 | PD8=TX / PD9=RX | USART3 |
| SWD | PA13 / PA14 | — |

定时器时基：APB1 = 42MHz → 定时器 84MHz；`Prescaler = 83` → 1MHz 计数，`Period = 19999` → 20ms(50Hz)，
所以 **CCR 值就是脉宽微秒数**，500～2500us 对应 0～180°。

> 六路舵机全部落在 GPIOA（PA0~PA3 + PA6/PA7），接线一排到底；
> 不再占用 PB3(JTDO) 这类有调试功能的引脚，SWD 下载/调试不受任何影响。

---

## 3. 与 F103 版的差异

| 项 | F103（`C:\STM32\hand`） | F407（本工程） |
|---|---|---|
| 舵机通道 | TIM2_CH1~4 + TIM3_CH1/2（含 PB0 补丁） | TIM2_CH1/CH2/CH3/CH4 + TIM3_CH1/CH2 = PA0/PA1/PA2/PA3/PA6/PA7，**不再需要 PB0/PB3 打补丁代码** |
| 串口 DMA | `DMA1_Channel5` 通道4 | `DMA2_Stream2` 通道4（寄存器直配，不依赖 CubeMX 勾选） |
| USART1 | CubeMX 已配异步 | CubeMX 已配异步（`huart1`），`NetLink.c` 直接用它发数据，只自己补 DMA 接收 |
| I2C1（OLED） | 驱动自己初始化 | 直接用 CubeMX 的 `hi2c1`（PB6/PB7，400kHz） |
| TIM2 | PWM Generation | 已改成 PWM Generation（CubeMX 重新配过） |
| 时基 | TIM4 | TIM4 |
| 调度 | FreeRTOS 四任务 | FreeRTOS 四任务（同一套） |

---

## 4. CubeMX 里建议顺手改的（不改也能跑）

1. **USART1_RX 的 DMA**（DMA2_Stream2 / Channel4 / Circular）。现在没勾，`NetLink.c` 用寄存器直配补上了；在 CubeMX 里勾了之后这段也可以删。
2. LSE（PC14/PC15）如果不用 RTC 可以关掉，否则启动会等它起振。

---

## 5. 编译与烧录

- 编译：Keil 打开 `MDK-ARM\smart hand.uvprojx`，F7；命令行 `UV4 -b "smart hand.uvprojx" -j0 -o build.log`。
  当前结果：`Code=31876 RO-data=1780 RW-data=312 ZI-data=22520`，0 Error 0 Warning（插值节拍已提到 100Hz）。
- 烧录：F8（ST-Link，SWD：PA13/PA14）。
- 产物：`MDK-ARM\smart hand\smart hand.hex`。

## 6. 上电流程

1. 六路 PWM **分时**启动（每路间隔 200ms）——避免 6 个舵机同时上电把 5V 拉垮。
2. 1.5 秒内六轴线性插值回中位 90°。
3. MPU6050 复位 + 陀螺零偏校准（约 0.4s，**此时别动板子**）。
4. 记录当前姿势为体感中位（约 0.2s）。
5. 进入 100Hz 插值循环（`armTask`）：体感控制 / 远程控制（网页）；夹爪上电默认**闭合**。

## 7. 串口协议（ESP32-S3 → STM32，每行 `\n` 结尾）

| 命令 | 含义 |
|---|---|
| `P,<a0..a5>[,<ms>]` | 设置 6 轴目标角度（0~180），可带完成时间 |
| `M,<0\|1>` | 0=体感模式，1=远程模式 |
| `G,<0\|1>` | 夹爪 0=张开 / 1=闭合 |
| `S` | 急停（停在当前角度） |
| `Q` | 查询状态 |
| `H,<0\|1>` | 500ms 周期心跳/状态回传开关 |

回复：`OK`、`ERR`、`ST,<mode>,<a0..a5>,<alive>`、`VR,<ver>`、`HB,<mode>,<a0..a5>`。

- `Q` 现在先回 `VR,f4-v1`（版本握手）再回 `ST,...`；ESP32-S3 把它放进 `/api/state` 的 `"stm32"` 字段，两端固件对不上时一眼能看出来。
- S3 侧对每条命令做**确认 + 重发**：发出 300ms 没收到 `OK`/`ERR` 就重发，最多 3 次，计数进 `/api/state` 的 `"retry"` / `"retry_fail"`。

## 10. IAP：不用 ST-Link 升级应用固件

### 10.1 Flash 分区（F407VE，512KB）

| 区域 | 地址范围 | 大小 | Keil 工程 | 说明 |
|---|---|---|---|---|
| 引导程序 | 0x08000000 ~ 0x0800FFFF | 64KB | `bootloader.uvprojx` | **只需 ST-Link 烧一次** |
| 应用 | 0x08010000 ~ 0x0803FFFF | 192KB | `smart hand.uvprojx` | IROM 起点就是 0x08010000 |
| 暂存区 | 0x08040000 ~ 0x0805FFFF | 128KB | — | 新固件先放这里 |
| 元数据 | 0x08060000 ~ 0x0807FFFF | 128KB | — | 只用前 16 字节 |

### 10.2 为什么不会刷成砖

1. 新固件先写**暂存区**，运行中的代码一个字节都不碰；
2. 元数据里的"待安装"标记只有**整段 CRC32 校验通过**才写；
3. 重启后引导程序校验暂存区 → 拷贝到应用区 → 再校验一次 → 才标记完成；
4. 中途掉电：标记还在，下次上电重拷一遍（暂存区没被擦）；
5. 暂存区坏了或没标记：直接启动老应用。

真出问题（应用区被擦、引导程序被误擦）也能救：**ST-Link 重新烧 `bootloader.hex` + `smart hand.hex`**。

### 10.3 三条命令

| 命令 | 作用 | 应答 |
|---|---|---|
| `U,<size>,<crc>` | 擦除暂存区与元数据区（1~2 秒） | `OK` / `ERR` |
| `W,<offset>,<hex>` | 写入一段数据，offset 必须严格等于已收字节数 | `OK` / `ERR` |
| `E,<crc32>` | 校验整段 CRC，通过则写元数据并重启 | `OK` 后重启 |

一帧最多 40 字节（hex 80 字符 + `W,<off>,` ≤ 88 字符，STM32 行缓冲 96）。
CRC32 是标准反射算法（多项式 0xEDB88320），与 `zlib.crc32` 一致。

### 10.4 怎么用

```bat
:: 1) 编译应用（多出一条 fromelf，产出 hand_app.bin）
cd "C:\STM32\smart hand\MDK-ARM"
"C:\Keil_v5\UV4\UV4.exe" -b "smart hand.uvprojx" -j0 -o build.log

:: 2) 通过 ESP32-S3 上传（手机/电脑都行）
python C:\STM32\hand\tools\iap_upload.py hand_app.bin --http http://<ESP地址>/

:: 或者 USB-TTL 直连 STM32 的 PA9/PA10
python C:\STM32\hand\tools\iap_upload.py hand_app.bin --serial COM8
```

网页上也有的选：ESP32-S3 控制台底部「固件升级」里第二个文件框选 `hand_app.bin`，点「烧到 STM32」。

### 10.5 第一次烧录（必须按这个顺序）

1. ST-Link 烧 `MDK-ARM\bootloader\bootloader.hex`（0x08000000）；
2. ST-Link 烧 `MDK-ARM\smart hand\smart hand.hex`（0x08010000）；
3. 之后改应用就只用 IAP / 网页，不用再插 SWD。

### 10.6 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 上电完全没反应，OLED 不亮 | 引导程序没烧、或应用没烧到 0x08010000 | 按 10.5 顺序重烧 |
| `U` 命令超时 | 应用版本太旧（没有 IAP 代码） | 先用 ST-Link 烧一次新应用 |
| 传到一半 `ERR` | 串口丢字节 / 帧太长 | 检查波特率 115200、共地；帧长保持 40 字节 |
| 升级后又是老程序 | CRC 校验没过，元数据没写 | 重传一次；看串口日志里的 `E` 是否回 `OK` |
| 想放弃升级 | — | 断电重启即可，暂存区不影响启动 |
---

## 11. 浮点轨迹核心与逐轴标定（f4-v4）

### 11.1 为什么改成 float

F407 有单精度 FPU（FPv4-SP），本工程已经启用，代价接近零：

| 位置 | 证据 |
|---|---|
| Keil 目标 | `<Cpu> ... FPU2 CPUTYPE("Cortex-M4")` |
| 启动代码 | `system_stm32f4xx.c` 里 `SCB->CPACR |= (3UL<<20)|(3UL<<22)` |
| FreeRTOS | 用的 `portable/RVDS/ARM_CM4F/port.c`，切任务保存 FPU 上下文 |

老版本用 0.01° 整数，两个地方被截断：

- 加速度 `400 / 25 = 16` 扣掉 jerk 之后只剩 2~3 格，慢速收尾时"某几拍不动、下一拍跳一格"；
- `Output_Pos()` 先把 0.01° 四舍五入成**整度**再写 CCR（1° ≈ 11.1µs），0.5µs 分辨率全被吃掉。

现在 `s_pos / s_target / s_vel / s_acc` 全是 float（度、度/秒、度/秒²），整条链路只在最后写
CCR 时量化一次。

### 11.2 实测（tools/float_check.py，1:1 复刻固件运算）

| 动作 | 耗时 | 峰值速度 | 过冲 | 稳态误差 |
|---|---|---|---|---|
| 大臂 90→130 | 350 ms | 180°/s | 0.000° | 0.0000° |
| 大臂 90→170 | 620 ms | 159°/s | 0.000° | 0.0000° |
| 底座 90→150 | 460 ms | 180°/s | 0.000° | 0.0000° |
| 1° 微调 | 70 ms | 39°/s | 0.000° | 0.0000° |
| 0.5° 微调 | 50 ms | 22.5°/s | 0.000° | 0.0000° |
| 贴限位 90→178 | 1240 ms | 75°/s | 0.000° | 0.0000° |
| 夹爪 25→135 | 280 ms | 600°/s | 0.000° | 0.0000° |

"贴限位"变慢是**故意的**：目标进入限位前 12° 的软区，速度按 30%~100% 线性下降。

### 11.3 两点标定：真正减小误差的抓手

浮点只让**命令**更细，舵机的真实角度由它自己的行程决定（不同舵机 500~2500µs 对应的
机械角可能差 3~8°，舵盘齿位还有 ±5°）。所以每个轴做两点标定：

1. 发 `P,90,90,90,90,90,25` 回中位，确认机械零位；
2. 串口发 `C,<轴>,<tmin>,<tmax>`，单位是 **0.5µs 计数**：
   - `tmin` = 命令 0°  时要发的计数（标准舵机 500µs → **1000**）
   - `tmax` = 命令 180° 时要发的计数（标准舵机 2500µs → **5000**）
3. 若某个关节实际只能转 0~160°（顶到结构），把它设成 `tmax = 1000 + 4000*160/180 ≈ 4555`，
   这样满行程命令也不会撞限位；
4. 回 `OK` 即生效（立即按新标定重写当前角度）。

### 11.4 与 PWM 分辨率的关系

| 项目 | 值 |
|---|---|
| 定时器时钟 | 84 MHz（APB1×2） |
| 预分频 | 41 → 2 MHz，即 **0.5 µs / 格** |
| 周期 ARR | 39999 → 20 ms（50 Hz） |
| 全行程格数 | 4000 格 / 180° ≈ **0.045°/格** |
| 舵机本身死区 | 2~10 µs（≈0.2~1°） |

即：分辨率已经不是瓶颈，**标定和死区才是**。

### 11.5 串口新增命令

| 命令 | 含义 |
|---|---|
| `C,<axis>,<tmin>,<tmax>` | 单轴两点标定（axis 0~5，计数 0.5µs） |

固件版本随之升到 `VR,f4-v3`。

---

## 12. 板载 W25Q16（2MB SPI NOR Flash）怎么用

### 12.1 接线（默认 SPI2，全在 GPIOB，与现有外设零冲突）

| W25Q16 | STM32F407 | 说明 |
|---|---|---|
| CS   | **PB12** | 软件片选，内部开漏上拉 + 程序里默认拉高 |
| CLK  | **PB13** | AF5 = SPI2_SCK |
| DO(MISO) | **PB14** | AF5 = SPI2_MISO |
| DI(MOSI) | **PB15** | AF5 = SPI2_MOSI |
| VCC  | 3.3V | **不要接 5V** |
| GND  | GND | |

不用 SPI1 的原因：SPI1 的默认脚 PA4/PA5/PA6/PA7 里，**PA6/PA7 已经给舵机 5/6**（TIM3_CH1/CH2），
PA4/PA5 也容易和别的东西撞。SPI2 这组脚在本工程里是空的。

速率：APB1 42MHz ÷ 4 = **10.5MHz**（W25Q16 标称 104MHz，这里很保守，飞线也稳）。

### 12.2 已经用起来的：参数持久化（`BSP_Drivers/W25Qxx.c` + `Param.c`）

存的是"标定表 + 速度 + 加速度"，共约 80 字节，放在第 0 个扇区（0x000000）：

| 字段 | 说明 |
|---|---|
| `magic` / `version` / `count` | `PARM` + 版本 + 轴数，格式不符直接丢弃 |
| `tmin[6]` / `tmax[6]` | 每个关节 0°/180° 对应的 CCR 计数（0.5µs 单位） |
| `speed[6]` / `accel[6]` | 度/秒、度/秒² |
| `crc` | 前面全部字节的 CRC32（与 IAP 同一套算法） |

行为：上电 `Param_Init()` 读一次 → CRC 通过就批量应用；串口发 `C,<axis>,<tmin>,<tmax>`
改了标定后**自动落盘**（擦 4KB 扇区 + 写 80 字节，约 50ms）。没插芯片时 `Param_Init()`
返回 2，其余功能完全不受影响。

**验证方法**：发 `C,1,1000,4600` → 断电 → 上电 → 发 `P,90,120,90,90,90,25`，
大臂的落点应该和改动前不同（因为 180° 对应的计数变了），说明参数确实存住了。

### 12.3 还能拿它做什么（按性价比排序）

| 用途 | 占用 | 价值 |
|---|---|---|
| **固件仓库 + 回滚** | 36KB × 2~3 = 10 个扇区 | 升级前把当前 `hand_app.bin` 备份到 W25Q16，新固件跑飞了能一键回退（比现在"只能重烧"强一档） |
| **示教动作组** | 每个点 6 字节 + 时间戳 | 2000 个点只占 12KB；配合 `/api/play` 做"录一遍、循环放"演示 |
| **黑匣子日志** | 环形 128KB | 记录 IAP 历史、WiFi 断开原因、急停次数，出问题能复盘 |
| **网页资源** | 几十 KB | 目前页面在 S3 上（4KB），没必要搬 |
| **XIP 直接执行** | — | **不行**：F407 没有 SPI NOR 的地址映射，只能拷到 RAM 跑，而 192KB 应用放不进 128KB RAM |

### 12.4 容量规划建议（2MB = 512 个 4KB 扇区）

| 地址 | 扇区 | 用途 |
|---|---|---|
| 0x000000 | 0 | 参数（当前在用） |
| 0x001000 ~ 0x00A000 | 1~10 | 固件备份 A（36KB） |
| 0x00B000 ~ 0x014000 | 11~20 | 固件备份 B / 上一版 |
| 0x015000 ~ 0x0FFFFF | 21~255 | 示教动作组 |
| 0x100000 ~ 0x1FFFFF | 256~511 | 日志环形缓冲 |

### 12.5 注意

- NOR 只能 **1 → 0**，写之前必须先按扇区擦（`W25Q_EraseRange()` 会把你给的区间覆盖到的扇区全擦掉，
  同扇区的其他数据一起没了）。
- 扇区擦除典型 45~400ms，`Wait_Ready(1000)` 兜底；这段时间**不要**放在 `armTask` 里。
- 若你板子上的 W25Q16 不是接 PB12~PB15，改 `W25Qxx.h` 顶部那 4 组引脚宏即可，驱动不用动。
- SPI2 的时钟要 `__HAL_RCC_SPI2_CLK_ENABLE()`（已在 `W25Q_Init()` 里），并确认
  `stm32f4xx_hal_conf.h` 里 `HAL_SPI_MODULE_ENABLED` 是打开的（本工程已打开，CubeMX 重新
  生成后可能被关掉，`tools/fix_cubemx.bat` 会检查这一项）。

---

## 13. 坏固件自动回滚（启动健康印章 + W25Q16 备份）

### 13.1 它解决什么问题

IAP 只能保证"新固件**完整送到**暂存区"，保证不了"新固件**跑得起来**"。
如果新固件一上电就 HardFault / 卡死，设备会无限重启，以前只能拆板子接 ST-Link 重烧。
现在引导程序会自己发现"连着 3 次没跑起来"，然后用 W25Q16 里的上一版好固件覆盖回去。

### 13.2 判定规则（写在 `bootloader/bootloader_main.c` 的 `main()` 里）

| 时机 | 引导程序做什么 |
|---|---|
| 元数据 state = PENDING 且暂存区 CRC 对 | 拷到应用区、校验、**擦掉 sector 7**（计数归零） |
| sector 7 里有 `"SEAL"` 印章 | 说明上次应用跑到了 3 秒 → **擦掉 sector 7**，计数归零 |
| 连续 3 个 `"BOOK"` 槽位被占（= 3 次没盖章） | 从 W25Q16 备份恢复应用区，然后写 `state=RESTORED` |
| 每次上电 | 占一个 `"BOOK"` 槽位（最多 8 个，够 8 次判断） |

`RESTORED` 是**截止标记**：回滚过一次后就不再回滚，避免"备份也坏"时来回刷写。

应用侧（`BSP_Drivers/Backup.c` → `Backup_Healthy()`，由 `freertos.c` 的 `WdTask` 每 200ms 调）：

1. 跑满 3 秒 → 往 sector 7 的 `0x30` 写 `"SEAL"`（一次编程，很便宜）；
2. 等机械臂停稳（`RobotArm_TargetReached()`）→ 把当前镜像（约 40KB）连同 CRC 存进 W25Q16；
3. 如果 W25Q16 里已经是同一份（size + CRC 一致），**什么都不做**（正常启动零开销）。

### 13.3 存储布局（两边必须一致）

内部 Flash sector 7（`0x08060000`）：

| 偏移 | 内容 |
|---|---|
| `+0x00` | `meta_t {magic "EMAT", size, crc, state}`（IAP 用） |
| `+0x10` | `uint32_t boot_slots[8]`（`"BOOK"`） |
| `+0x30` | `uint32_t seal`（`"SEAL"`） |
| `+0x34` | `uint32_t seal_ver`（固件标识哈希，仅供显示） |

W25Q16：

| 地址 | 内容 |
|---|---|
| `0x001000` | 备份头 `backup_hdr_t {magic "BAK1", size, crc, tag, ...}` |
| `0x002000` | 镜像本体（最多 192KB，与内部应用区等大） |

> 先写镜像、最后写头：中途掉电时旧头仍指向旧镜像，不会出现"头新身旧"。
> 参数区（`0x000000`，见第 12 节）与备份区（`0x001000` 起）互不干扰。

### 13.4 怎么验证（关键：制造一个"坏固件"）

正常固件永远不会触发回滚，所以要主动造一个坏的应用：

1. 先烧 `bootloader.hex`（0x08000000）+ 当前 `smart hand.hex`（0x08010000），上电等 5 秒
   —— 这时应用已经盖了章、并把镜像存进了 W25Q16。
2. 做一份"坏应用"：在 `Core/Src/main.c` 的 `USER CODE 2` 最前面加一行
   ```c
   volatile uint32_t boom = *(volatile uint32_t *)0x1FFFFFF0UL;  /* 立刻 HardFault */
   (void)boom;
   ```
   编译得到 `hand_app.bin`，用网页/`iap_upload.py` 走 IAP 刷进去。
3. 断电上电 **4 次**（每次都会复位重来）。第 4 次上电时串口/OLED 会看到老固件又回来了
   （`VR,f4-v4`），机械臂恢复正常 —— 说明回滚成功。
4. 把那一行注释掉，重新编译烧回去。

### 13.5 失败与边界

| 现象 | 原因 | 处理 |
|---|---|---|
| 刷了坏固件但没回滚 | W25Q16 没焊/没检测到（`Backup_Init()` 返回 1） | 回滚依赖备份，没芯片就只能靠 `RESTORED` 兜底不刷写，最终仍需 SWD 重烧 |
| 回滚后还是不起来 | 备份本身也坏 / 应用区硬件问题 | 只会回滚一次（`RESTORED` 截止），接 ST-Link 烧 `bootloader.hex` + 应用 |
| 正常固件被误判回滚 | 应用在 3 秒内没跑完初始化（例如 MPU6050 一直不通，重试循环超过 3 秒） | `Backup_Healthy()` 是在 WdTask 里跑的，只要任务在跑就会盖章；若初始化阶段就卡死，属于"真起不来" |
| 上电慢了一点 | 每次上电多写一个 `"BOOK"` 槽位，几十毫秒 | 无感 |

### 13.6 状态机自检

`tools/boot_state_check.py` 用 Python 重放了这套判定（5 个场景：一直正常 / 一直崩 / 回滚后恢复 /
备份不可用 / 断续健康），断言"回滚只在连续 3 次未盖章且没回滚过时发生一次"。
改判定逻辑后先跑它：

```
C:\Ana\python.exe "C:\STM32\smart hand\tools\boot_state_check.py"
```
