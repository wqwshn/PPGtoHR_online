# PPG 心率监测系统

基于 STM32L452CEU6 的实时 PPG 光电心率监测系统，配套 PyQt5 上位机可视化软件。

## 项目架构

```
L452CEU6_ALL_Cmake/
├── Core/
│   ├── Inc/                          # 头文件
│   │   ├── main.h                    # MCU 引脚定义, 工作模式
│   │   ├── hr_algorithm.h            # 心率算法 API (配置/状态结构体)
│   │   ├── hr_dsp.h                  # DSP 原语 (FFT峰值, 延迟搜索, 频谱惩罚)
│   │   ├── hr_lms.h                  # LMS 自适应滤波器封装
│   │   ├── MAX30101.h                # PPG 传感器驱动
│   │   ├── MIMU.h                    # IMU 驱动 (LSM9DS1)
│   │   └── ADC_ADS124.h              # ADC 驱动 (ADS124S06)
│   └── Src/
│       ├── main.c                    # 主循环 (传感器采集 + 算法调度)
│       ├── hr_algorithm.c            # 心率算法编排器 (14步流水线)
│       ├── hr_dsp.c                  # DSP 实现
│       ├── hr_lms.c                  # LMS 滤波器实现
│       └── [传感器驱动]
├── tools/hr_monitor/                 # Python 上位机
│   ├── main.py                       # 入口
│   ├── dashboard.py                  # PyQt5 主界面
│   ├── protocol.py                   # HR 协议解析
│   ├── serial_reader.py              # 串口通信线程
│   ├── requirements.txt              # Python 依赖
│   └── start_monitor.bat             # Windows 快速启动
├── docs/
│   ├── 在线心率算法实施文档.md        # 算法设计与实施细节
│   └── 上位机UI说明文档.md            # 上位机使用说明
├── outline-HRsolver/                 # MATLAB 离线算法参考
├── CMakeLists.txt                    # CMake 构建配置
├── STM32L452CEUx_FLASH.ld            # 链接脚本
└── .gitignore
```

## 硬件平台

| 组件 | 型号 | 接口 | 用途 |
|------|------|------|------|
| MCU | STM32L452CEU6 | - | Cortex-M4F @ 80MHz, 160KB RAM, 512KB Flash |
| PPG 传感器 | MAX30101 | I2C (PA0/PA1) | 绿光 LED, 18-bit ADC, 心率采集 |
| IMU | LSM9DS1 | SPI2 (PB7/PB8) | 三轴加速度, 运动伪影参考 |
| ADC | ADS124S06 | SPI1 (PA4) | 24-bit 4通道, HF 参考信号 |
| 通信 | Bluetooth/UART | UART2 @ 115200 | 双协议数据输出 |

### GPIO 引脚映射

| 功能 | 引脚 | 描述 |
|------|------|------|
| I2C1_SCL | PA0 | MAX30101 时钟线 |
| I2C1_SDA | PA1 | MAX30101 数据线 |
| SPI1_NSS | PA4 | ADS124 片选 |
| V5_0_CE | PA8 | 5.0V 电源使能 |
| V1_8_CE | PA9 | 1.8V 电源使能 |
| START_CONV | PA10 | ADC 启动转换 |
| DRDY | PA11 | ADC 数据就绪 (EXTI 中断) |
| AD_RESET | PA12 | ADC 复位 |
| CS_A_G | PB7 | 加速度计/陀螺仪片选 |
| CS_M | PB8 | 磁力计片选 |
| BLE_STATE | PB12 | 蓝牙状态指示 |
| BLE_RST | PB13 | 蓝牙复位信号 |

## 工作模式

通过 `main.h` 中的宏定义切换：

```c
#define CURRENT_WORK_MODE MODE_HEART_RATE   // 心率模式 (绿光 LED)
#define CURRENT_WORK_MODE MODE_SPO2         // 血氧模式 (红光+红外 LED)
```

**心率模式**: 绿光 LED (660mA), Multi-LED 模式, 4x 硬件平均, 1000 SPS
**血氧模式**: 红光 + 红外 LED (各 29mA), 400 SPS, 18-bit ADC, 温度补偿

## 在线心率算法

### 三路融合心率估计

算法采用 8 秒滑动窗口、1 秒步长，通过三条独立路径估计心率：

1. **LMS-HF 路径**: 2 级级联 NLMS 滤波器，以 HF 参考信号消除运动伪影
2. **LMS-ACC 路径**: 3 级级联 NLMS 滤波器，以最优 ACC 轴消除运动伪影
3. **Pure-FFT 路径**: 直接 FFT，无自适应滤波 (适用于静止状态)

融合策略: 运动状态 -> LMS-ACC，静止状态 -> Pure-FFT，后接 5 点中值平滑 + 变化率限制。

### 关键参数

| 参数 | 值 | 说明 |
|------|----|------|
| 采样率 | 125 Hz | TIM16 硬件定时器 |
| 窗口长度 | 8 秒 (1000 点) | 滑动窗口 |
| 步长 | 1 秒 (125 点) | 1Hz 输出 |
| FFT 点数 | 4096 | 实数 RFFT, 0.031Hz 分辨率 |
| 心率范围 | 60-240 BPM | 1.0-4.0 Hz |
| LMS 阶数 | 最大 16 | 根据延迟搜索自适应调整 |

### 14 步算法流水线

```
数据准备: 窗口滑动 -> IIR 带通滤波 -> 滤波窗口滑动
运动检测: ACC 幅度标准差 vs 自适应阈值
时间对齐: 最优延迟搜索 (+-5 采样), 最优 ACC 轴选择
三路估计: LMS-HF(2级) / LMS-ACC(3级) / Pure-FFT
融合输出: 运动/静止决策 -> 中值平滑 -> 变化率限制 -> BPM
```

## 通信协议

### 原始数据包 (21 字节 @ 125Hz)

| 偏移 | 字段 | 类型 | 描述 |
|------|------|------|------|
| 0-1 | Header | uint8 | 0xAA, 0xBB |
| 2-9 | ADC Data | 4x uint16 | 4 通道桥路数据 |
| 10-12 | ACC High | 3x uint8 | ACC X,Y,Z 高字节 |
| 13-16 | PPG Data | 4 bytes | 绿光 PPG (24-bit + count) |
| 17-18 | Extension | 2 bytes | 温度 (SpO2) 或标志 (HR) |
| 19 | XOR Checksum | uint8 | 字节 2-18 的异或校验 |
| 20 | Footer | uint8 | 0xCC |

### 心率结果包 (21 字节 @ 1Hz)

| 偏移 | 字段 | 类型 | 描述 |
|------|------|------|------|
| 0-1 | Header | uint8 | 0xAA, 0xCC |
| 2-3 | Fused HR | uint16 | BPM x10 (72.5 = 725) |
| 4 | Motion Flag | uint8 | 0=静止, 1=运动 |
| 5 | Window Filled | uint8 | 0=未满, 1=已满 |
| 6-7 | LMS-HF BPM | uint16 | BPM x10 |
| 8-9 | LMS-ACC BPM | uint16 | BPM x10 |
| 10-11 | FFT BPM | uint16 | BPM x10 |
| 12-13 | PPG Mean | uint16 | 信号强度 |
| 14 | Calibrated | uint8 | 校准状态 |
| 15-16 | Timestamp | uint16 | 秒计数器 |
| 17 | Cal Progress | uint8 | 校准进度 (0-8) |
| 18 | Sampling Rate | uint8 | 采样率 (固定 125) |
| 19 | XOR Checksum | uint8 | 字节 2-18 的异或校验 |
| 20 | Footer | uint8 | 0xCC |

## 资源占用

| 资源 | 使用量 | 总量 | 占比 |
|------|--------|------|------|
| RAM | ~89 KB | 160 KB | 56% |
| Flash | ~140 KB | 512 KB | 27% |

RAM 主要消耗: 原始窗口 (20KB) + 滤波窗口 (20KB) + FFT 缓冲 (32KB) + LMS 状态 + 辅助缓冲。

## 构建与烧录

### 前置条件

- `arm-none-eabi-gcc` 交叉编译工具链
- CMake >= 3.20
- OpenOCD (烧录)
- ST-Link V2 调试器

### 构建

```bash
mkdir build && cd build
cmake ..
cmake --build . -j4
```

### 烧录

```bash
openocd -f openocd.cfg -c "program build/L452CEU6_ALL_Cmake.elf verify reset exit"
```

### 调试

```bash
# 终端 1: 启动 OpenOCD 服务器
openocd -f openocd.cfg

# 终端 2: GDB 调试
arm-none-eabi-gdb build/L452CEU6_ALL_Cmake.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) continue
```

## 上位机使用

### 安装依赖

```bash
cd tools/hr_monitor
pip install -r requirements.txt
```

依赖: PyQt5, pyqtgraph, pyserial

### 运行

```bash
# 连接硬件串口
python main.py

# 模拟模式 (无需硬件)
python main.py --simulate
```

### 功能

- 实时心率大字显示 (颜色编码: <60 蓝, 60-100 绿, 100-140 橙, >140 红)
- 三路算法路径 BPM 对比 (LMS-HF, LMS-ACC, FFT)
- 60 秒趋势曲线
- 运动状态指示
- 采样率显示 (125Hz)
- 数据导出 CSV (含采样率字段)
- 中/英文切换
- 模拟模式 (调试用)

## 系统时序

```
125Hz TIM16 中断
    |
    +-- START_CONV 脉冲 -> 触发 ADC 转换
    |
    +-- DRDY EXTI 中断 (GPIO)
        |
        +-- 读取 ADS124 4 通道数据
        +-- 读取 LSM9DS1 ACC 数据
        +-- 读取 MAX30101 FIFO (批量)
        +-- 设置 ADC_1to4Voltage_flag = 4

主循环 (while 1):
    |
    +-- 125Hz 数据就绪?
    |   +-- 推送采样到算法缓冲区
    |   +-- DMA 发送 21 字节原始数据包
    |
    +-- 1Hz 算法就绪?
        +-- HR_RunSolver() 14 步流水线
        +-- 构建 21 字节 HR 结果包
        +-- DMA 发送 HR 结果包
```

## 蓝牙配置

蓝牙模块初始化流程 (代码中已注释，需要时取消注释):

1. IO 引脚复位蓝牙模块 (PB13)
2. 以默认 19200 波特率发送配置命令
3. 设置常开模式, 发射功率 +2.5dBm, 蓝牙名称, 配对密码
4. 切换波特率至 115200
5. MCU 同步切换波特率

## 文档

- `docs/在线心率算法实施文档.md` - 算法设计、实施细节、已知问题
- `docs/上位机UI说明文档.md` - 上位机功能说明和操作指南
- `outline-HRsolver/` - MATLAB 离线算法参考代码

## 注意事项

1. **电源时序**: 必须在 SPI 初始化之后开启传感器电源，避免引脚干扰
2. **FIFO 管理**: MAX30101 的 FIFO 需要在每次读取后排空
3. **模式切换**: 修改工作模式后需要重新编译烧录
4. **窗口预热**: 算法启动后前 8 秒为窗口填充期，输出 HR 为 0
5. **运动阈值**: 默认阈值 200.0，需根据实际传感器数据校准

## 开发规范

- **C**: CMSIS-DSP API, float32, static 内部函数, ALL_CAPS 宏, 中文注释 UTF-8
- **Python**: type hints, dataclasses, PyQt5 signals/slots, f-strings
- **提交**: `feat:/fix:/refactor:` 前缀, 简洁中/英文

## 更新日志

- **2026-03** 在线心率算法集成: 三路融合 (LMS-HF, LMS-ACC, FFT), 1Hz HR 输出, PyQt5 上位机
- 125Hz 多传感器数据采集 + 蓝牙通讯
- 心率/血氧双模式支持
- 温度补偿功能
