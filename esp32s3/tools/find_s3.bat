@echo off
chcp 65001 >nul
setlocal
set "PY=C:\Ana\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" "%~dp0find_s3.py" %*
echo.
pause