@echo off
rem ============================================================================
rem  flash_first.bat  —— 第一次烧录：引导程序 + 应用（只需做一次）
rem
rem  用法：
rem    flash_first.bat            自动尝试：ST-Link(SWD)  ->  USB DFU
rem    flash_first.bat swd        只用 ST-Link
rem    flash_first.bat dfu        USB DFU（BOOT0 拉高 + 复位，走板载 USB）
rem    flash_first.bat uart COMx  UART 引导（BOOT0 拉高，走 PA9/PA10 那对脚）
rem
rem  烧写内容：
rem    0x08000000  bootloader.hex   （引导程序，64KB 区）
rem    0x08010000  smart hand.hex   （应用，192KB 区，hex 里自带地址）
rem ============================================================================
setlocal enabledelayedexpansion
chcp 936 >nul

set "CLI=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "BASE=C:\STM32\smart hand\MDK-ARM"
set "BOOT=%BASE%\bootloader\bootloader.hex"
set "APP=%BASE%\smart hand\smart hand.hex"

if not exist "%CLI%" (
  echo [X] 没找到 STM32CubeProgrammer：
  echo     %CLI%
  echo     装好后再跑，或者直接用 Keil 分别烧两个工程（F8）。
  goto :end
)
if not exist "%BOOT%" (
  echo [X] 没找到 %BOOT%
  echo     先在 Keil 里编译 bootloader 工程。
  goto :end
)
if not exist "%APP%" (
  echo [X] 没找到 %APP%
  echo     先在 Keil 里编译应用工程。
  goto :end
)

set "MODE=%~1"
if "%MODE%"=="" set "MODE=auto"

echo.
echo === 要烧的文件 ===
echo   引导程序 : %BOOT%
echo   应用     : %APP%
echo   模式     : %MODE%
echo.

if /i "%MODE%"=="dfu"  goto :dfu
if /i "%MODE%"=="uart" goto :uart
if /i "%MODE%"=="swd"  goto :swd

:auto
echo [1/2] 试 ST-Link ...
"%CLI%" -c port=SWD mode=UR reset=HWrst -w "%BOOT%" -w "%APP%" -rst -v
if not errorlevel 1 goto :ok
echo.
echo [!] ST-Link 没连上，改试 USB DFU ...
:dfu
echo.
echo 请把 BOOT0 拨到 1（3.3V），按一下复位，确认板子的 USB 口插在电脑上。
pause
"%CLI%" -c port=USB1 -w "%BOOT%" -w "%APP%" -rst -v
if not errorlevel 1 goto :ok
echo.
echo [X] DFU 也失败。检查：BOOT0 是否=1 / USB 线是否数据线 / 驱动是否装（STM32 DFU）
goto :end

:swd
"%CLI%" -c port=SWD mode=UR reset=HWrst -w "%BOOT%" -w "%APP%" -rst -v
if not errorlevel 1 goto :ok
echo.
echo [X] SWD 失败。检查：SWDIO=PA13 / SWCLK=PA14 / GND / 3.3V，
echo     以及 Keil 里 Debug 用 ST-Link、Port=SW、Connect=under Reset。
goto :end

:uart
set "COMPORT=%~2"
if "%COMPORT%"=="" (
  echo [X] 用法：flash_first.bat uart COM8
  goto :end
)
echo 请把 BOOT0 拨到 1 并复位，确保 S3 的 TX/RX 没有抢 PA9/PA10（最好拔掉 S3 或按住它的 EN）。
pause
"%CLI%" -c port=%COMPORT% br=115200 -w "%BOOT%" -w "%APP%" -rst -v
if not errorlevel 1 goto :ok
echo [X] UART 引导失败：检查 BOOT0 / 波特率 / PA9-PA10 是否被 S3 占用
goto :end

:ok
echo.
echo === 烧写完成 ===
echo 1) BOOT0 拨回 0，按复位
echo 2) 上电后 OLED 应显示 ARM LINK:... ，串口发 Q 应回 VR,f4-v4
echo 3) 等 5 秒（盖章 + 备份），再断电上电一次，确认还能正常起来
echo.
echo 之后升级应用就不用 SWD 了：网页 / iap_upload.py 走 IAP 即可。

:end
pause
endlocal