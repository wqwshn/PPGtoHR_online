# CLAUDE.md - PPG Heart Rate Monitor Project

## Project Overview
STM32L452CEU6 embedded PPG heart rate monitor with PyQt5 PC dashboard.
- **MC firmware**: C + CMSIS-DSP, 125Hz sampling, 1Hz HR output via UART
- **PC dashboard**: Python/PyQt5 real-time monitor, serial/Bluetooth, dark theme

## Key Directories
- `Core/Inc/` / `Core/Src/` - STM32 HAL + HR algorithm (hr_algorithm, hr_dsp, hr_lms)
- `tools/hr_monitor/` - Python upper computer (dashboard, protocol, serial_reader)
- `docs/` - Implementation documentation
- `outline-HRsolver/` - MATLAB offline algorithm reference

## Build & Run
- **MCU**: CMake + arm-none-eabi-gcc, flash via OpenOCD
- **PC**: `python tools/hr_monitor/main.py` or `--simulate` mode

## Documentation Update Rule (MANDATORY)
After ANY modification to:
- **Algorithm/firmware** (hr_algorithm, hr_dsp, hr_lms, main.c, CMakeLists.txt):
  Update `docs/在线心率算法实施文档.md` - add brief note in Section 9 (commit history) or Section 10 (known issues).
- **UI/upper computer** (dashboard, protocol, serial_reader, main.py):
  Update `docs/上位机UI说明文档.md` - add entry to the change log table.

Include: date, what changed, why (if non-obvious), affected files.

## Coding Conventions
- C: CMSIS-DSP APIs, float32, static functions for internal use, ALL_CAPS macros
- Python: type hints, dataclasses, PyQt5 signals/slots, f-strings
- Commits: `feat:/fix:/refactor:` prefix, concise Chinese or English

## Language
- Communicate with user in Chinese
- Code comments in Chinese, encoding UTF-8
- No emojis in responses or generated files

## Architecture Notes
- Algorithm: 3-path fusion (LMS-HF, LMS-ACC, pure FFT), motion detection, 5-point median + slew rate
- Protocol: 20-byte HR result packet @ 1Hz, header 0xAA 0xCC, XOR checksum
- RAM budget: ~89KB/160KB used, Flash ~27%
- Dashboard supports i18n (zh/en), data save to CSV, screen clear
