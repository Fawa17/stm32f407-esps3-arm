@echo off
chcp 65001 >nul
setlocal
set "PY=C:\Ana\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" "%~dp0verify_s3.py" %*
echo.
pause