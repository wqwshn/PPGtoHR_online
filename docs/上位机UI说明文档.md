# PPG Monitor 统一上位机 UI 说明文档

> 本文档记录统一上位机 monitor 的设计、功能、协议交互，供开发和调试参考。
> 由原 hr_monitor 和 data_monitor 融合而来。

---

## 1. 概述

基于 PyQt5 + pyqtgraph 构建的暗色主题统一监测上位机，通过串口/蓝牙接收 STM32 单片机数据，支持两种工作面板:
- **在线心率面板**: 1Hz 心率结果包 (31字节, 0xAA 0xCC)，实时展示融合心率、三路径对比、趋势曲线
- **原始数据面板**: 100Hz 原始传感器包 (35字节, 0xAA 0xBB)，实时展示 PPG 波形、热膜桥压、加速度计、陀螺仪和链路质量；同时接收 1Hz Raw 链路诊断 STATUS 包 (53字节, 0xAA 0xDD)

### 1.1 运行方式

```bash
# 实际串口模式 (自动识别两种协议包)
python tools/monitor/main.py

# HR 模拟数据模式 (1Hz)
python tools/monitor/main.py --simulate

# 原始数据模拟模式 (100Hz)
python tools/monitor/main.py --raw-simulate
```

### 1.2 依赖

- Python >= 3.9
- PyQt5
- pyqtgraph
- pyserial

安装: `pip install -r tools/monitor/requirements.txt`

---

## 2. 功能模块

### 2.1 工具栏

| 控件 | 功能 |
|------|------|
| 状态指示灯 | 绿色脉冲=已连接, 红色=未连接 |
| 串口选择 | 自动枚举系统可用串口 |
| 连接/断开 | 管理串口连接 (115200 bps) |
| 刷新 | 重新扫描串口列表 |
| **面板切换** | "在线心率" / "原始数据" 双面板切换按钮 |
| **清屏** | 清除当前面板的所有数据、曲线、记录缓冲区 |
| **保存路径** | 显示当前保存目录名称, 点击可切换目录, 默认桌面 |
| **录制** | 点击即开始录制到默认路径, 无需选择文件; 再次点击停止并保存 |
| **语言切换** | 中文/English 即时切换，默认中文 |
| 连接状态 | 文字显示当前连接状态 |

### 2.2 心率卡片

- 大字显示当前融合心率 BPM
- 颜色阈值: <60 蓝色, 60-100 绿色, 100-140 橙色, >140 红色
- 运动状态标签: 静息(绿色) / 运动(橙色)

### 2.3 算法路径卡片

三路径 BPM 对比进度条:
- **LMS-HF**: LMS 自适应滤波 + HF 桥臂参考 (青色)
- **LMS-ACC**: LMS 自适应滤波 + ACC 参考计 (紫色)
- **FFT**: 纯 FFT + Hamming 窗 (橙色)

附加信息:
- PPG 信号均值 (信号强度参考)
- 校准进度 (0-8, 运动阈值自动标定)
- 数据窗口状态 (填充中/就绪)

### 2.4 趋势图

- 显示最近 60 秒心率曲线
- 绿色参考区域: 60-100 BPM 正常心率范围
- X/Y 轴自动缩放

### 2.5 原始数据面板

顶部信息条显示: 当前模式(HR/SpO2) | 实际采样率 | 丢包率 | 数据包总数

**PPG 波形区** (模式切换):
- HR 模式: 绿光 PPG 波形 (100Hz 实时)
- SpO2 模式: 左侧红光+红外 PPG 上下排列 / 右侧温度曲线 + SpO2 预留

**通用波形** (始终显示):
- 桥顶电压: Ut1 (橙色) + Ut2 (紫色) 水平并排
- 桥中电压: Uc1 (红色) + Uc2 (蓝色) 水平并排
- 三轴加速度: AccX(红) + AccY(绿) + AccZ(蓝) 单图三线

波形以 33ms (30FPS) 定时刷新，缓冲区 2000 点，可见窗口 800 点 (8 秒)。信息条按帧率更新而非逐包刷新。

原始数据面板顶部信息条新增 `实时心率/Realtime HR`: 每 1 秒从绿光 PPG 缓冲区取最近 8 秒数据执行轻量纯 FFT 估计，仅用于静息采集时快速判断佩戴位置与绿光信号是否可解算。该计算不进入串口读取线程，也不在逐包 CSV 写入路径中执行；界面同步显示单次计算耗时 `Calc xx ms`，便于观察算法开销。

原始数据面板顶部信息条新增 `Diag`: 显示固件 1Hz STATUS 帧中的关键链路诊断摘要，格式为 `Busy x | Err y | PCGap z | FIFO empty/overflow`。该信息用于判断缺失更可能来自 UART/DMA busy、发送错误、PC 端未收到或 PPG FIFO 空读/溢出。

### 2.6 状态栏

显示: 数据包计数 | 运行时间 | [录制数据点数(仅录制时)] | PPG 均值 | 校准进度

---

## 3. 数据录制

### 3.1 录制机制

- 工具栏"保存路径"按钮显示当前保存目录，点击可选择其他目录，默认桌面
- 点击"录制"按钮直接开始录制 (自动生成文件名)，无需弹出文件对话框
- 录制期间每收到一个心率结果包 (1Hz)，记录所有解算字段到内存缓冲区
- 时间列采用相对时间 (从录制开始的第一个数据包计，单位秒，精度毫秒)
- 再次点击按钮停止录制，数据自动写入之前选择的 CSV 文件
- 原始数据面板录制: 逐包实时写入 CSV，每 100 包刷盘一次 (避免 GUI 线程阻塞导致丢包)
- 原始数据面板 `Time(s)` 按 100Hz 样本序号生成，不再使用 UI 主线程处理时间，避免串口/绘图批处理导致时间戳跳变
- 原始数据面板 CSV 新增 `Seq` 和 `MissingBefore`: `Seq` 为固件侧 Raw 采样序号, `MissingBefore` 为该包前由序号缺口推断的缺失样本数
- 原始数据面板录制会同时生成同名 `_status.csv`: 记录 1Hz STATUS 计数器快照、PC 端 Raw 接收/缺失统计和 `PCMissingAfterTxDone`，用于采集后诊断链路瓶颈
- 串口读取线程采用 0.01s timeout + 最多 4 个 Raw 包的小块读取，避免 4096 字节读取造成约 124 包批量进入 UI

### 3.2 CSV 格式

默认文件名 `hr_data_YYYYMMDD_HHMMSS.csv`，保存路径在开始录制时由用户选择。

CSV 列定义:

| 列名 | 类型 | 说明 |
|------|------|------|
| time | float | 相对时间 (秒) |
| fused_bpm | float | 融合心率 (BPM) |
| is_motion | int | 运动标志 (0/1) |
| win_filled | int | 窗口填充状态 (0/1) |
| hr_lms_hf | float | LMS-HF 路径 BPM |
| hr_lms_acc | float | LMS-ACC 路径 BPM |
| hr_fft | float | FFT 路径 BPM |
| ppg_mean | int | PPG 信号均值 (缩放值 = 原始均值 / 8) |
| motion_calibrated | int | 运动阈值是否已校准 (0/1) |
| timestamp | int | 单片机秒计数器 |
| calib_progress | int | 校准进度 (0-8) |
| sampling_rate | int | 采样率 (Hz) |

**注意**: `ppg_mean` 为缩放后的值，真实 PPG 均值需乘以 8。正常佩戴时应在 5000-15000 范围，对应原始值 40000-120000。

### 3.3 清屏行为

点击"清屏"按钮会同时清除记录缓冲区并重置录制状态。如需保留数据，请先停止录制再清屏。

---

## 4. 语言切换

支持中文和 English，默认中文。点击语言切换按钮即时切换所有 UI 文本:

- 窗口标题
- 工具栏按钮文字
- 算法路径卡片标题和信息标签
- 运动状态标签 (静息/运动 or REST/MOTION)
- 校准和窗口状态文本
- 状态栏文本

---

## 5. 通信协议

### 5.1 心率结果包 (31 字节, 1Hz, 帧头 0xAA 0xCC)

```
偏移  字段                类型        说明
0-1   帧头                uint8 x2    0xAA, 0xCC
2-3   融合心率 BPM        uint16 BE   x10 精度 (72.5 = 725)
4     运动标志             uint8       0=静息, 1=运动
5     窗口填充状态         uint8       0=未满, 1=已满
6-7   LMS-HF 路径 BPM     uint16 BE   x10 精度
8-9   LMS-ACC 路径 BPM    uint16 BE   x10 精度
10-11 FFT 路径 BPM        uint16 BE   x10 精度
12-13 PPG 信号均值         uint16 BE   信号强度
14    运动校准状态         uint8       0=未校准, 1=已校准
15-16 时间戳              uint16 BE   秒计数器
17    校准窗口进度         uint8       0-8
18    采样率              uint8       固定 100
19    XOR 校验             uint8       bytes[2..18] 异或
20    帧尾                uint8       0xCC
```

### 5.2 原始传感器包 (35 字节, 100Hz, 帧头 0xAA 0xBB)

当前 Raw 包格式如下。该格式自 2026-04-24 起用于链路质量评估，旧 21/33 字节格式仅作为历史记录保留在变更记录中。

```
偏移   字段                类型        说明
0-1    帧头                uint8 x2    0xAA, 0xBB
2-3    桥顶2 (HF2)         uint16 BE   ADS124S06 24bit高16bit
4-5    桥顶1 (HF1)         uint16 BE   ADS124S06 24bit高16bit
6-7    桥中2               uint16 BE   ADS124S06 24bit高16bit
8-9    桥中1               uint16 BE   ADS124S06 24bit高16bit
10-15  ACC X/Y/Z           int16 BE    LSM9DS1完整16bit, X/Y/Z
16-21  GYRO X/Y/Z          int16 BE    LSM9DS1完整16bit, X/Y/Z
22-24  PPG Green           3 bytes     17-bit原始值
25-27  PPG Red             3 bytes     17-bit原始值
28-30  PPG IR              3 bytes     17-bit原始值
31-32  Seq                 uint16 BE   固件侧Raw采样序号, 0xFFFF后回绕
33     XOR 校验             uint8       bytes[2..32] 异或
34     帧尾                uint8       0xCC
```

链路质量评估:
- `RX Hz`: PC 端每秒解析成功的 Raw 包数。
- `DEV Hz`: 按 `Seq` 推断的设备侧 Raw 采样周期数, 正常应接近 100Hz。
- `Loss`: `missing_count / expected_count`, 其中 `missing_count` 来自 `Seq` 缺口。
- `MissingBefore`: CSV 中每个包前的缺失样本数。例如序号 11 后直接收到 15, 则序号 15 行的 `MissingBefore=3`。

### 5.3 Raw 链路诊断 STATUS 包 (53 字节, 1Hz, 帧头 0xAA 0xDD)

STATUS 包用于第一阶段链路诊断，不改变 35 字节 Raw DATA 包格式。

```
偏移   字段                         类型        说明
0-1    帧头                         uint8 x2    0xAA, 0xDD
2      protocol_version             uint8       当前为 1
3-6    mcu_time_ms                  uint32 BE   MCU HAL_GetTick()
7-10   sample_counter               uint32 BE   TIM16 采样 tick 总数
11-14  adc_drdy_counter             uint32 BE   ADC DRDY 中断次数
15-18  frame_counter                uint32 BE   Raw DATA 组帧次数
19-22  tx_start_counter             uint32 BE   Raw DATA UART DMA 启动成功次数
23-26  tx_done_counter              uint32 BE   Raw DATA UART DMA 完成回调次数
27-30  tx_busy_counter              uint32 BE   Raw DATA 发送时 HAL_BUSY 次数
31-34  tx_error_counter             uint32 BE   Raw DATA UART/DMA 错误次数
35-38  adc_error_counter            uint32 BE   ADC 状态异常次数
39-42  imu_error_counter            uint32 BE   IMU 运行期异常次数
43-46  ppg_fifo_empty_counter       uint32 BE   PPG FIFO 空读次数
47-50  ppg_fifo_overflow_counter    uint32 BE   PPG FIFO 接近满/溢出风险次数
51     XOR 校验                      uint8       bytes[2..50] 异或
52     帧尾                         uint8       0xCC
```

录制时 `_status.csv` 的关键派生列:
- `PcReceivedRaw` / `PcExpectedRaw` / `PcMissingRaw`: 上位机按 Raw `Seq` 统计的接收、期望和缺失样本数。
- `PcMissingAfterTxDone`: `tx_done_counter - PcReceivedRaw` 的非负部分，用于估算 MCU 已完成发送但 PC 未解析成功的帧数。
- `TxInflight`: `tx_start_counter - tx_done_counter` 的非负部分，用于观察 DMA 是否长期未完成。

历史旧格式摘录如下，当前版本不再使用:

```
偏移  字段                类型        说明
0-1   帧头                uint8 x2    0xAA, 0xBB
2-3   桥顶2 (HF2)         uint16 BE   24bit 高16bit+8bit
4-5   桥顶1 (HF1)         uint16 BE   24bit 高16bit+8bit
6-7   桥中2               uint16 BE   24bit 高16bit+8bit
8-9   桥中1               uint16 BE   24bit 高16bit+8bit
10    ACC X 高字节         uint8
11    ACC Y 高字节         uint8
12    ACC Z 高字节         uint8
13-16 PPG 数据            4 bytes     模式相关 (见下)
17-18 扩展数据            2 bytes     模式相关 (见下)
19    XOR 校验             uint8       bytes[2..18] 异或
20    帧尾                uint8       0xCC

HR 模式:   bytes[13-15]=24bit绿光累加, byte[16]=采样计数
           bytes[17]=0x00, byte[18]=0xFF (模式标记)
SpO2 模式: bytes[13-14]=16bit红光均值, bytes[15-16]=16bit红外均值
           byte[17]=温度整数(有符号), byte[18]=温度小数
           温度公式: die_temp_int + die_temp_frac * 0.0625 + 2.4 (LED温升补偿)
```

### 5.4 帧解析状态机

串口读取线程使用双协议状态机:
1. 等待帧头 0xAA
2. 等待第二帧头字节区分协议:
   - 0xCC -> 31字节 HR 结果包 -> `parse_hr_packet()` -> `HRPacket`
   - 0xBB -> 35字节 原始传感器包 -> `parse_raw_packet()` -> `RawDataPacket`
   - 0xDD -> 53字节 Raw 链路诊断包 -> `parse_status_packet()` -> `StatusPacket`
3. 收集 payload 直到满对应长度
4. XOR 校验 + 帧尾验证
5. 通过不同的 pyqtSignal 发射给对应面板

---

## 6. 文件结构

```
tools/monitor/
  main.py              # 程序入口, AppController 连接 MonitorWindow 和 SerialReader
  dashboard.py         # MonitorWindow(外壳+工具栏) + HRPanel(在线心率面板) + 翻译表/配色
  raw_data_panel.py    # RawDataPanel(原始数据面板) - PPG/ACC/桥压/SpO2 波形
  realtime_hr.py       # 原始数据面板绿光PPG实时纯FFT心率估计
  protocol.py          # HRPacket(31字节) + RawDataPacket(35字节) + StatusPacket(53字节) 协议定义与解析
  serial_reader.py     # 多协议串口读取线程 (QThread + 状态机)
  requirements.txt     # Python 依赖
  start_monitor.bat    # Windows 一键启动脚本
```

---

## 7. 变更记录

| 日期 | 变更内容 |
|------|----------|
| 2026-03-28 | 初始版本: 实时心率仪表盘, 串口通信, 三路径对比, 趋势图 |
| 2026-03-28 | 新增清屏功能: 清除所有数据和曲线 |
| 2026-03-28 | 新增数据保存: CSV 导出解算后数据 (相对时间, UTF-8-BOM) |
| 2026-03-28 | 新增语言切换: 中/English 双语支持, 默认中文 |
| 2026-03-28 | 新增采样率显示: 算法路径卡片增加采样率字段, HR结果包扩展至21字节 |
| 2026-03-29 | 协议升级至31字节: 新增HF2 AC幅值、HF2-PPG相关系数字段; 上位机新增HF1/HF2信号质量显示 (AC幅值+相关系数) |
| 2026-04-01 | 录制功能重构: "保存"按钮改为"录制"按钮, 点击先选路径(默认桌面)再开始录制, 停止时自动保存, 状态栏显示录制数据点数 |
| 2026-04-05 | 融合 data_monitor 原始数据功能: 新建 tools/monitor/ 统一上位机, 支持面板切换(在线心率/原始数据); 双协议串口状态机(31字节HR+21字节Raw); 原始数据暗色科技风面板(PPG/桥压/ACC/SpO2, 50ms刷新); 扩展 i18n |
| 2026-04-15 | 多光谱原始数据包支持(33字节): 新增 RawDataPacket 解析(Green+Red+IR三通道PPG + 16-bit ACC + 陀螺仪); 串口双包类型自动检测(0xAA0xBB/0xAA0xCC); PPG波形固定三通道布局(左侧绿光, 右侧红光+红外上下); 新增陀螺仪角速度波形; 移除HR/SpO2模式切换; tools/monitor/ 协议/串口/面板同步更新 |
| 2026-04-18 | UI优化: 1)录制按钮旁新增保存路径设置(默认桌面, 点击录制直接开始无需选路径); 2)修复数据录制丢包(移除逐包flush改为每100包刷盘, 串口读取缓冲区从128字节增至4096); 3)绘图性能优化(启用pyqtgraph裁剪视图+自动降采样, 刷新频率从20FPS降至15FPS) |
| 2026-04-18 | 绘图流畅度优化: 缓冲区翻倍至2000点, 可见窗口缩减为800点(8秒), 每帧渲染量-20%; 刷新率提升至30FPS(33ms); 信息条从逐包(100Hz)降频为按帧率(30FPS)更新, 消除冗余QLabel重绘 |
| 2026-04-24 | 原始数据录制缺点排查修复: 串口读取由 4096 字节大块读取改为低延迟小块读取; Raw CSV 时间列改为按 100Hz 样本序号生成，消除批处理造成的时间戳跳变和点击录制尾部时延 |
| 2026-04-24 | Raw链路质量评估: 原始数据包扩展为35字节, 新增固件侧 `Seq`; 上位机显示 `RX Hz/DEV Hz/Loss`, CSV新增 `Seq` 与 `MissingBefore` |
| 2026-04-26 | 原始数据面板新增绿光 PPG 实时纯 FFT 心率估计: 8 秒窗口、1Hz 更新、显示计算耗时; 计算从已有缓冲读取，不影响串口解析和 Raw CSV 逐包保存链路 |
| 2026-04-30 | Raw链路诊断第一阶段: 新增 53 字节 STATUS 包 (`0xAA 0xDD`) 与 `StatusPacket` 解析; Raw 面板显示 `Diag` 摘要; 录制时同步生成 `_status.csv`，用于采集后定位 MCU 采样/组帧/UART DMA/PC 接收解析问题 |

---

**最后更新**: 2026-04-30
**对应分支**: main
