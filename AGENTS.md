# Repository Guidelines

## 项目结构与模块组织

本仓库包含 STM32L452CEU6 固件和 Python 上位机监控工具，用于 PPG 心率采集与验证。固件源码位于 `Core/Src/`，头文件位于 `Core/Inc/`，重点模块包括 `hr_algorithm`、`hr_dsp`、`hr_lms`、传感器驱动、UART、SPI、TIM、DMA 和 GPIO 初始化。`Drivers/` 存放 STM32 HAL 与 CMSIS 第三方代码，除非硬件集成需要，避免直接改动。当前上位机在 `tools/monitor/`，包含 PyQt5 仪表盘、协议解析、原始数据面板、实时 FFT 心率估计和链路质量统计。测试位于 `tests/`，文档位于 `docs/`，构建产物放在 `build/`。

## 构建、测试与运行命令

- `cmake -S . -B build`：配置 ARM GCC 固件构建。
- `cmake --build build -j4`：生成 `.elf`、`.hex` 和 `.bin`。
- `make`：使用 STM32CubeMX 生成的 Makefile 构建。
- `openocd -f openocd.cfg -c "program build/L452CEU6.elf verify reset exit"`：通过 ST-Link 烧录固件。
- `conda activate ppg-prj`：进入本项目 Python 运行环境。
- `pip install -r tools/monitor/requirements.txt`：安装上位机依赖。
- `python tools/monitor/main.py --simulate`：运行 1Hz 心率模拟。
- `python tools/monitor/main.py --raw-simulate`：运行 100Hz 原始数据模拟。
- `python -m pytest -q tests`：运行仓库正式 pytest 测试。

## 代码风格与命名约定

C 代码使用 C11、STM32 HAL 和 CMSIS-DSP。文件内私有函数使用 `static`，宏使用 `ALL_CAPS`，模块接口优先使用清晰前缀，例如 `HR_`、`MIMU_`、`MAX30101_`。Python 代码优先使用类型提示、dataclass、PyQt signals/slots 和 f-string。中文文档、脚本提示和既有中文注释保持 UTF-8 编码。

## 测试规范

主测试框架为 pytest。测试文件命名为 `test_*.py`，优先放在 `tests/` 下。修改协议布局、Raw 链路质量、串口读取、CSV 录制、实时心率估计或固件常量时，应补充或更新对应测试。提交前至少运行 `python -m pytest -q tests`。

## 提交与 Pull Request 规范

历史提交使用简洁前缀，例如 `feat:`、`fix:`、`docs:`、`perf:`、`revert:`，中英文均可。提交应聚焦单一主题，并说明影响范围，例如 firmware、protocol、UI 或 docs。PR 需包含修改目的、关键文件、测试/构建结果、关联 issue；涉及界面变化时附截图或录屏。

## 文档与配置说明

修改固件或算法文件时，同步更新 `docs/在线心率算法实施文档.md`。修改上位机 UI、协议、串口读取或 Raw 面板行为时，同步更新 `docs/上位机UI说明文档.md`。不要提交 `build/`、CSV 录制文件、缓存目录或本地 IDE 配置。
