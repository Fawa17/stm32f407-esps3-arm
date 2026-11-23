@echo off
:: Build hand_s3 (ESP32-S3 / ESP-IDF 5.5.5)
:: Usage:  build.bat            -> build
::         build.bat flash      -> build + flash (needs board on USB)
::         build.bat menuconfig -> configure
setlocal
call "C:\STM32\hand\tools\idf_env.bat"
cd /d "%~dp0"

if not exist sdkconfig (
    echo [1/2] set-target esp32s3
    idf.py set-target esp32s3
)

if /i "%1"=="menuconfig" (
    idf.py menuconfig
    goto :done
)

echo [2/2] build
idf.py build
if errorlevel 1 goto :fail

if /i "%1"=="flash" (
    echo [3/3] flash
    idf.py -p COM%2 flash monitor
)

:done
goto :eof

:fail
echo BUILD FAILED
exit /b 1