@echo off
title PPG Monitor

REM ========================================
REM   PPG Monitor 启动脚本
REM   用法:
REM     start_monitor.bat                - 串口模式
REM     start_monitor.bat --simulate     - HR 模拟数据模式
REM     start_monitor.bat --raw-simulate - 原始数据模拟模式 (100Hz)
REM ========================================

echo [PPG Monitor] 正在启动...

cd /d "%~dp0"

REM 检查 Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未找到 Python, 请检查 PATH 环境变量
    goto :end
)

REM 激活 conda 环境
echo [PPG Monitor] 正在激活 conda 环境: ppg_prj
call conda activate ppg_prj
if %errorlevel% neq 0 (
    echo [警告] 未找到 conda 环境 ppg_prj, 使用系统 Python
)

REM 检查依赖
python -c "import PyQt5; import pyqtgraph; import serial" 2>nul
if %errorlevel% neq 0 (
    echo [错误] 缺少依赖库, 请执行: pip install -r requirements.txt
    goto :end
)

REM 显示运行模式
if "%~1"=="" (
    echo [PPG Monitor] 运行模式: 串口模式
) else if "%~1"=="--simulate" (
    echo [PPG Monitor] 运行模式: 心率模拟
) else if "%~1"=="--raw-simulate" (
    echo [PPG Monitor] 运行模式: 原始数据模拟
) else (
    echo [PPG Monitor] 运行模式: %*
)
echo [PPG Monitor] 正在启动程序...
echo.

REM 启动上位机
python main.py %*
set EXIT_CODE=%errorlevel%

echo.
if %EXIT_CODE% equ 0 (
    echo [PPG Monitor] 程序已正常退出
) else (
    echo [PPG Monitor] 程序异常退出, 错误码: %EXIT_CODE%
)

REM 退出 conda 环境
call conda deactivate

:end
echo.
pause
