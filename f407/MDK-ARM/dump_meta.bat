@echo off
rem ============================================================================
rem  dump_meta.bat  —— 读出"元数据区" sector 7 的前 64 字节，验证启动计数/印章
rem
rem  期望看到（小端，每 4 字节一个字）：
rem    0x08060000: magic=45 4D 41 54 ("EMAT") 或 FF FF FF FF（已擦）
rem    0x08060004: size
rem    0x08060008: crc
rem    0x0806000C: state（01 00 00 00 = 待安装 / 02 00 00 00 = 已回滚过 / FF.. = 空闲）
rem    0x08060010: boot_slots[0]  42 4F 4F 4B = "BOOK"（占用了 1 次启动）
rem    0x08060030: seal            53 45 41 4C = "SEAL"（应用已盖章 => 健康）
rem ============================================================================
setlocal
chcp 936 >nul

set "CLI=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

if not exist "%CLI%" (
  echo [X] 没找到 STM32CubeProgrammer CLI
  goto :end
)

echo === sector 7 (0x08060000) 前 64 字节 ===
"%CLI%" -c port=SWD -u 0x08060000 0x40
echo.
echo === 解读 ===
echo   0C 处 = 01 00 00 00  -^> 有 PENDING 新固件待安装
echo   0C 处 = 02 00 00 00  -^> 已经回滚过一次（防重复回滚的截止标记）
echo   10 起出现 42 4F 4F 4B -^> 占了 1 个启动槽；连着 3 个就该回滚了
echo   30 处 = 53 45 41 4C  -^> 应用盖了健康印章（正常状态）

:end
pause
endlocal