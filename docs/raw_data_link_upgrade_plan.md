# 原始数据链路诊断与时间轴补救升级方案

## 1. 核心问题定义

当前目标不是单纯降低丢包率，而是把连续 Raw 采集链路拆成两类问题：

- **链路诊断**：判断缺失更可能发生在传感器采样、MCU 组帧、UART/DMA/BLE 发送、串口链路、PC 读取解析或文件写入哪一段。
- **数据补救**：即使已经发生缺失，也要让 20 分钟以上连续采集的数据仍保留可信时间轴，便于后续 PPG、IMU、温度和外部设备对齐。

仅统计丢包率和校验错误率不够，因为：

- 丢包率只能说明接收序号存在缺口，不能说明缺口来自 MCU 端、无线链路还是 PC 端。
- XOR 校验错误只能说明一帧内容不可信，不能发现某些偶数位翻转，也不能定位错误发生位置。
- 按接收顺序保存数据会在缺包后压缩时间轴，长时间采集时会逐步偏离真实采样时间。
- 诊断和补救是两件事：诊断回答“哪里坏了”，补救回答“数据文件如何仍可被分析”。

不建议一开始做复杂 ACK/重传：

- PPG/IMU Raw 是固定频率连续流，历史样本过期后价值下降，重传容易造成新数据阻塞。
- 115200 bps UART/BLE 透传带宽有限，ACK/重传会显著增加协议复杂度和实时性风险。
- 本项目更适合先做 **轻量诊断 + 上位机显式缺失标记 + 短缺失插值**。

### 1.1 分阶段实施判断

建议将 **链路诊断** 和 **时间轴补救** 分段实施，而且第一阶段应优先完善链路诊断机制。

理由如下：

- 当前最大不确定性不是“如何插值”，而是“丢失主要发生在哪里”。如果直接上线时间轴补救，虽然 CSV 会更规整，但仍无法判断缺失来自 MCU 采样、DMA busy、UART/BLE 透传、PC 串口读取还是解析写盘。
- 链路诊断改动可以先做得很轻：只增加 MCU 计数器、STATUS 状态帧、上位机日志和 quality summary，不急着改变 Raw DATA payload、采样节奏、插值策略或 ACK/重传。
- 先采集一段带诊断计数的数据，可以用真实证据判断下一步：如果 `tx_busy_counter` 增长明显，优先处理 DMA/缓冲；如果 `sample_counter` 与 `frame_counter` 偏差明显，优先查采样到组帧路径；如果 MCU 计数连续但 PC 缺帧明显，优先查串口链路、解析和写盘。
- 时间轴补救依赖可信的 `sample_idx`、缺失段信息和质量事件。先把诊断字段做清楚，后续重建时间轴时才能明确哪些缺失是短 gap、长 gap、重复帧、回绕还是设备重启。

推荐阶段边界：

1. **阶段 A：链路诊断优先**。保留当前 Raw 数据格式或尽量少改 Raw DATA payload，增加 STATUS 帧和关键计数器，上位机记录诊断快照与质量事件。
2. **阶段 B：采集反馈与定点修复**。用新诊断机制采集 10-20 分钟以上 Raw 数据，分析 busy、error、gap、invalid frame 和计数器差值，先解决主要丢失来源，把丢失情况降到最低。
3. **阶段 C：时间轴补救部署**。在链路主要问题被定位后，再加入规则时间轴导出、显式 NaN 缺失行和短缺失插值，保证长期采集数据可对齐、可回溯。

因此，“先完善链路诊断 -> 采集数据反馈 -> 分析并降低实际丢失 -> 再部署时间轴补救”是更合理的开展顺序。

## 2. 当前链路现状

### 2.1 MCU 端采样、组帧、发送流程

主要代码位置：

| 环节 | 文件/函数 | 当前行为 |
| --- | --- | --- |
| 工作模式和 Raw 协议常量 | `Core/Inc/main.h` | `ENABLE_RAW_DATA_PACKET=1` 时发送 Raw 包；`PACKET_LEN=35`；`RAW_SEQUENCE_START_INDEX=31`；`XOR_CHECK_LEN=31`。 |
| 定时采样触发 | `Core/Src/main.c` `HAL_TIM_PeriodElapsedCallback()` | TIM16 周期中断拉动 `START_CONV`，触发 ADS124S06 ADC 转换。采样率来自 `PPG_SAMPLE_RATE=100`。 |
| ADC 读取 | `Core/Src/main.c` `HAL_GPIO_EXTI_Callback()`；`Core/Src/ADC_ADS124.c` `ADC_RDATA()` | ADC DRDY 外部中断中按状态机读取 4 路桥压，写入 `allData[2]..[9]`。 |
| IMU 读取 | `Core/Src/main.c` `HAL_GPIO_EXTI_Callback()`；`Core/Src/MIMU.c` `ACC_6BytesRead()` / `GYRO_6BytesRead()` | 当 `ADC_1to4Voltage_flag == 4` 时读取 ACC/GYRO 原始 6 字节。 |
| PPG 读取 | `Core/Src/main.c` 主循环；`Core/Src/ppg_channel.c`；`Core/Src/MAX30101.c` / `MAX30101_2.c` | 读取 MAX30101 FIFO 写/读指针，计算 `sample_count=(wr_ptr-rd_ptr)&0x1F`，批量排空 FIFO 后求 Green/Red/IR 均值。 |
| Raw 组帧 | `Core/Src/main.c` 主循环 `ADC_1to4Voltage_flag == 4` 分支 | 复用全局 `allData[50]`，填 ADC、ACC、GYRO、PPG、`raw_packet_seq`、XOR、帧尾。 |
| UART/DMA 发送 | `Core/Src/main.c` `HAL_UART_Transmit_DMA(&huart2, allData, PACKET_LEN)`；`Core/Src/usart.c` | USART2 115200 8N1，TX DMA1 Channel7，RX DMA1 Channel6。当前忽略 HAL 返回值。 |
| BLE | `Core/Src/main.c` `BLE_Init()` | 目前在 `#if 0` 内，保留 AT 初始化代码但未启用。实际链路更像 UART 透传到外部模块。 |

当前发送缓冲与失败处理：

- 存在单个全局发送缓冲 `allData[50]`。
- 未发现发送队列、双缓冲或环形缓冲。
- `HAL_UART_Transmit_DMA()` 返回值被 `(void)` 丢弃。
- 未显式判断 `huart2.gState`、DMA busy、HAL_BUSY、HAL_ERROR。
- 未维护发送成功计数、发送失败计数、DMA busy 计数或队列溢出计数。
- `raw_packet_seq++` 位于 DMA 调用之后，但不依赖返回值；因此它更接近“MCU 生成/尝试发送 Raw 帧计数”，不是“发送成功计数”。

### 2.2 当前 Raw 协议字段

当前 Raw 包是固定 35 字节：

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0-1 | Header | `0xAA 0xBB` | Raw 帧头 |
| 2-3 | Ut2/HF2 | `uint16 BE` | ADS124S06 桥顶2，24bit 高位压缩格式 |
| 4-5 | Ut1/HF1 | `uint16 BE` | ADS124S06 桥顶1 |
| 6-7 | Uc2 | `uint16 BE` | 桥中2 |
| 8-9 | Uc1 | `uint16 BE` | 桥中1 |
| 10-15 | ACC X/Y/Z | `int16 BE * 3` | 加速度 |
| 16-21 | GYRO X/Y/Z | `int16 BE * 3` | 陀螺仪，已扣除零偏 |
| 22-24 | PPG Green | 3 bytes | 17-bit Raw 值 |
| 25-27 | PPG Red | 3 bytes | 17-bit Raw 值 |
| 28-30 | PPG IR | 3 bytes | 17-bit Raw 值 |
| 31-32 | Seq | `uint16 BE` | 当前 Raw 序号，0xFFFF 后回绕 |
| 33 | XOR | `uint8` | `bytes[2..32]` 异或 |
| 34 | Footer | `0xCC` | 帧尾 |

当前校验方式：

- Raw 包使用 1 字节 XOR，不是 CRC16。
- HR 包也使用 XOR。
- SPI 外设层未启用 HAL SPI CRC；ADS124S06 文档中也记录 CRC 未启用。

### 2.3 上位机端接收、解析、统计、保存流程

主要代码位置：

| 环节 | 文件/函数 | 当前行为 |
| --- | --- | --- |
| 串口打开 | `tools/monitor/serial_reader.py` `SerialReader.run()` | PySerial 打开 USART，115200 8N1，timeout 0.01s。 |
| 字节读取 | `tools/monitor/serial_reader.py` `read_serial_chunk()` | 按 `in_waiting` 小块读取，最多 `RAW_PACKET_LEN * 4` 字节，避免 4096 字节批量延迟。 |
| 字节流缓存/帧同步 | `tools/monitor/serial_reader.py` `SerialReader.run()` | 三状态机：等 `0xAA`、识别第二帧头、收满固定长度。支持 ASCII 调试行。 |
| 帧解析 | `tools/monitor/protocol.py` `parse_raw_packet()` | 校验长度、帧头、帧尾、XOR，解析为 `RawDataPacket`。 |
| 无效帧统计 | `tools/monitor/serial_reader.py` | `_raw_total` 统计收满的 Raw 候选帧，`_raw_invalid` 统计解析失败帧。 |
| 丢包统计 | `tools/monitor/raw_quality.py` `RawQualityStats.observe()` | 根据 `sequence` 缺口计算 `missing_count`、`expected_count`、`loss_rate`，支持 uint16 回绕。 |
| UI 质量显示 | `tools/monitor/raw_data_panel.py` | 显示 `RX Hz`、`DEV Hz`、`Loss`、包计数。 |
| CSV 保存 | `tools/monitor/raw_data_panel.py` `raw_packet_to_csv_row()` | 每个收到的有效包写一行，列含 `Time(s)`、`Seq`、`MissingBefore` 和各传感器值。 |
| 当前时间轴 | `tools/monitor/raw_data_panel.py` `sample_index_to_elapsed_seconds()` | 用 `_recorded_sample_count / 100Hz` 生成 `Time(s)`。注意 `_recorded_sample_count` 是已接收并写入的行数，不是 `pkt.sequence`。 |

## 3. 当前系统的链路诊断能力

### 3.1 诊断字段检查

| 字段/计数器 | 当前是否具备 | 说明 |
| --- | --- | --- |
| `frame_seq` | 否 | Raw 只有 `Seq`，没有独立帧序号。 |
| `sample_idx` / `start_sample_idx` | 部分具备 | `Seq` 可近似视为 Raw 生成周期序号，但只有 16 位，语义未明确为全局采样序号。 |
| `payload_len` | 否 | 固定长度协议，帧内没有长度字段。 |
| `sample_count` | 否 | MCU 内部计算了 MAX30101 FIFO `sample_count`，但没有上报。 |
| CRC16 | 否 | 当前是 1 字节 XOR。 |
| `mcu_time_ms` | 否 | 没有 MCU 毫秒时间戳。HR 包有 16 位秒计数，Raw 包没有。 |
| MCU 采样计数器 | 否 | 没有独立上报 ADC/IMU/PPG 采样触发次数。 |
| MCU 组帧计数器 | 否 | 没有独立上报 Raw 组帧次数。 |
| MCU 成功发送计数器 | 否 | `HAL_UART_Transmit_DMA()` 返回值被忽略。 |
| DMA busy 计数器 | 否 | 未判断 HAL_BUSY。 |
| 发送队列溢出计数器 | 不适用/否 | 当前没有发送队列。 |
| 传感器读取异常计数器 | 否 | 初始化阶段有阻塞检查和调试文本，运行期没有结构化异常计数。 |

### 3.2 明确判断

仅靠当前上位机接收到的数据，**不能可靠区分“单片机未发送”和“上位机未收到”**。

当前 `Seq` 能告诉上位机“有序号缺口”，但不能区分：

- MCU 没有触发采样；
- MCU 采样了但未成功组帧；
- MCU 组帧了但 UART DMA busy 或启动失败；
- UART/BLE 发送过程中丢失；
- PC 已收到字节但串口读取、帧同步、校验或写文件阶段丢失。

至少需要 MCU 额外上报以下状态：

- `mcu_time_ms`：MCU 运行时间，辅助判断停顿、重启和时钟漂移。
- `sample_counter`：采样周期计数，最好 32 位。
- `frame_counter`：已组帧计数。
- `tx_start_counter`：调用发送并返回 `HAL_OK` 的次数。
- `tx_done_counter`：DMA 发送完成回调次数。
- `tx_busy_counter`：`HAL_UART_Transmit_DMA()` 返回 `HAL_BUSY` 次数。
- `tx_error_counter`：`HAL_ERROR` 或 UART error 次数。
- `queue_overflow_counter`：如果后续增加发送队列，需要统计队列满丢弃次数。
- `sensor_error_counter`：ADC/IMU/PPG 运行期读取失败或 FIFO 异常次数。
- `ppg_fifo_empty_counter` / `ppg_fifo_overflow_counter`：区分 PPG 没有新样本和 FIFO 溢出。

实现代价判断：

- 增加计数器本身代价小，多数是 `uint32_t` 自增。
- 增加 STATUS 状态帧代价中等，需要协议版本、解析器、UI/CSV/日志展示。
- 把 Raw DATA 帧扩展为可变长统一帧代价中等，需要固件和上位机同步升级。
- 真正复杂的是发送队列、ACK、重传和乱序恢复，不建议作为第一阶段。

ACK/重传必要性：

- 对当前连续 PPG/IMU Raw 流，**不建议优先做 ACK/重传**。
- 推荐先做“DATA 帧带序号 + STATUS 状态帧 + 上位机缺失补救”。
- 如果未来出现控制命令、配置写入、标定结果保存等低频关键消息，可对这些控制类消息单独设计 ACK，不必给高频 Raw 数据流做全量重传。

## 4. 当前系统的缺失补救与时间轴重建能力

### 4.1 当前保存方式检查

| 能力 | 当前是否具备 | 说明 |
| --- | --- | --- |
| 根据采样序号生成 `timestamp_s` | 否/部分 | CSV `Time(s)` 来自 `_recorded_sample_count / 100Hz`，不是 `Seq / fs`。 |
| 使用上位机接收时间作为采样时间戳 | 否 | Raw CSV 未使用 wall-clock 写每行时间，这是正确方向。 |
| 发现缺失 `sample_idx` | 部分具备 | `RawQualityStats` 能根据 `Seq` 缺口得到 `MissingBefore`。 |
| 文件保留缺失点位置 | 否 | 只在后一条有效数据行写 `MissingBefore`，没有为缺失样本生成占位行。 |
| 区分 raw 原始数据和 interp 插值数据 | 否 | 当前只保存收到的原始有效包，不保存插值层。 |
| `valid_flag` / `interp_flag` / `gap_len` / `source_frame_seq` | 否/部分 | 只有 `MissingBefore` 和 `Seq`。 |
| 长时间采集防漂移 | 不充分 | 一旦丢包，`Time(s)` 按接收行数递增，会压缩缺失时间。 |

### 4.2 明确判断

当前保存方式**不适合直接作为 20 分钟以上连续采集的最终对齐数据格式**。

原因：

- 20 分钟 100Hz 约 120000 个样本，16 位 `Seq` 会至少回绕一次；当前统计支持回绕，但 CSV 中单独 `Seq` 不足以形成全局唯一样本号。
- 丢包后 `Time(s)` 仍按已接收行数递增，缺失时间不会在文件中展开，时间轴会向前压缩。
- `MissingBefore` 能提示缺失数量，但后处理必须额外展开时间轴；直接用 CSV 行号或 `Time(s)` 对齐会有风险。
- 与其他传感器或外部设备对齐时，当前格式容易把缺失导致的空洞误当成连续采样。

如果发生丢包，当前文件中的 `Seq` 和 `MissingBefore` 仍有诊断价值，但 `Time(s)` 不应被视为完整规则采样时间轴。

### 4.3 推荐保存方式

推荐输出三类数据，严格分离：

1. `raw_received.csv`
   - 只保存真实收到并通过校验的原始帧。
   - 保留 `frame_seq`、`sample_idx`、`mcu_time_ms`、`rx_wall_time_ns`、`crc_ok`、`source_frame_seq`。
   - 不插入伪造数据。

2. `timeline_aligned.csv`
   - 按全局 `sample_idx` 展开规则时间轴。
   - 每个采样点都有 `timestamp_s = (sample_idx - first_sample_idx) / fs`。
   - 缺失点先填 `NaN`。
   - 原始有效点 `valid_flag=1, interp_flag=0`。
   - 缺失未补点 `valid_flag=0, interp_flag=0`。
   - 短缺失插值点 `valid_flag=0, interp_flag=1`。

3. `quality_events.csv` 或 `quality_summary.json`
   - 记录 gap 起点、gap 长度、涉及帧序号、CRC 错误数、DMA busy、传感器异常、队列溢出等。
   - 用于排查链路，而不是混在信号数据列里。

插值策略：

- 缺失点默认保留 `NaN`。
- 对短缺失，例如 1-3 个样本，可对 PPG/ADC/IMU 连续量线性插值，并标记 `interp_flag=1`。
- 对中长缺失，例如 >3 或 >5 个样本，只标记，不强行伪造。
- 对状态量、计数器、校验字段不插值。
- 插值结果不能覆盖原始数据，必须可回溯到原始帧。

## 5. 推荐目标方案

### 5.1 统一帧协议

建议引入版本化统一帧头，而不是继续无限扩展固定 35 字节包：

| 字段 | 建议类型 | 说明 |
| --- | --- | --- |
| `sof0` / `sof1` | `uint8` | 例如继续使用 `0xAA` + frame_type。 |
| `protocol_version` | `uint8` | 便于兼容旧解析器。 |
| `frame_type` | `uint8` | `DATA_RAW`、`STATUS`、`HR_RESULT` 等。 |
| `header_len` | `uint8` | 便于扩展。 |
| `payload_len` | `uint16` | 支持变长 payload。 |
| `frame_seq` | `uint32` | 每发出一帧递增。 |
| `start_sample_idx` | `uint32` | DATA 帧首样本全局序号。 |
| `sample_count` | `uint16` | DATA 帧内样本数。单样本帧为 1，未来可批量。 |
| `mcu_time_ms` | `uint32` | MCU 时间戳。 |
| `payload` | bytes | 传感器数据。 |
| `crc16` | `uint16` | 覆盖 header 关键字段和 payload。 |

如果短期不想重构为可变长协议，可以先在现有 Raw 包后扩展关键字段，但要同步修改 `PACKET_LEN`、解析器、测试和文档。

### 5.2 STATUS 状态帧

建议 STATUS 帧 1Hz 上报，不混入每个 Raw DATA 帧，避免高频数据过重。

建议计数器：

| 计数器 | 用途 |
| --- | --- |
| `sample_counter` | MCU 采样 tick 总数。 |
| `adc_drdy_counter` | ADC DRDY 中断次数。 |
| `frame_counter` | Raw DATA 组帧次数。 |
| `tx_start_counter` | UART DMA 启动成功次数。 |
| `tx_done_counter` | UART DMA 完成次数。 |
| `tx_busy_counter` | DMA busy 次数。 |
| `tx_error_counter` | UART/DMA 错误次数。 |
| `ppg_fifo_empty_counter` | PPG FIFO 无新样本次数。 |
| `ppg_fifo_overflow_counter` | PPG FIFO 溢出或回滚风险次数。 |
| `adc_error_counter` | ADC 读取异常次数。 |
| `imu_error_counter` | IMU 读取异常次数。 |
| `parser_version` | 便于上位机确认协议匹配。 |

### 5.3 上位机时间轴重建

上位机应以 `sample_idx` 为主键，不以接收顺序为主键：

- `global_sample_idx` 使用 32 位或 64 位展开，处理 16 位兼容序号回绕。
- `timestamp_s = (global_sample_idx - first_global_sample_idx) / fs`。
- 收到样本填入对应索引。
- 未收到样本生成显式 NaN 行。
- 插值只写入派生文件或派生列，不写回原始文件。

## 6. 单片机端建议修改点

### 6.1 文件和函数

| 文件 | 建议修改 |
| --- | --- |
| `Core/Inc/main.h` | 增加协议版本、帧类型、CRC16、STATUS 帧长度和字段偏移定义。 |
| `Core/Src/main.c` | 增加全局诊断计数器；在采样、组帧、发送、DMA 回调路径中维护计数。 |
| `Core/Src/usart.c` | 保持 USART2/DMA 初始化；可按需增加 UART error 处理入口。 |
| `Core/Src/stm32l4xx_it.c` | DMA/USART IRQ 已存在，必要时配合 HAL 回调统计。 |
| `Core/Src/ADC_ADS124.c` | 如有返回值改造空间，增加运行期读取异常反馈。 |
| `Core/Src/MIMU.c` | 如有返回值改造空间，增加 IMU 读取异常反馈。 |
| `Core/Src/MAX30101.c` / `MAX30101_2.c` | 增加 FIFO 异常、空读、溢出相关诊断。 |

关键函数：

- `HAL_TIM_PeriodElapsedCallback()`：增加 `sample_tick_counter`。
- `HAL_GPIO_EXTI_Callback()`：增加 `adc_drdy_counter`，并统计 ADC/IMU 读取阶段异常。
- 主循环 `ADC_1to4Voltage_flag == 4` 分支：增加 `frame_counter`、`start_sample_idx`、`sample_count`、CRC16。
- `HAL_UART_Transmit_DMA()` 调用点：检查返回值，统计 `tx_start_counter`、`tx_busy_counter`、`tx_error_counter`。
- `HAL_UART_TxCpltCallback()`：统计 `tx_done_counter`。
- 可新增 `BuildRawDataFrame()`、`BuildStatusFrame()`、`Crc16Ccitt()`，减少主循环内联组帧复杂度。

### 6.2 DATA 帧建议增加字段

- `frame_seq`：独立帧序号，32 位。
- `start_sample_idx`：DATA 帧首样本全局序号，32 位。
- `sample_count`：帧内样本数量，当前可为 1。
- `payload_len`：payload 长度。
- `mcu_time_ms`：组帧时刻。
- `crc16`：替代 XOR。

### 6.3 DMA busy 和发送队列风险

当前 `HAL_UART_Transmit_DMA()` 可能在上一次 DMA 未完成时返回 `HAL_BUSY`，但代码忽略返回值。风险包括：

- 当前 `allData` 是单缓冲，若 DMA 尚未发送完，下一轮采样覆盖 `allData`，可能导致帧内容被污染。
- `Seq` 会继续递增，但上位机无法知道是 busy、链路丢失还是 PC 丢失。
- 如果 Raw 频率上升或 BLE 透传吞吐不足，busy 会变成主要丢包来源。

建议优先级：

1. 先统计 busy/error，不急着重传。
2. 如 busy 明显存在，再改成双缓冲或小型环形队列。
3. 队列满时丢弃旧帧或新帧要明确策略，并通过 STATUS 上报 `queue_overflow_counter`。

## 7. 上位机端建议修改点

### 7.1 文件和函数

| 文件 | 建议修改 |
| --- | --- |
| `tools/monitor/protocol.py` | 新增统一帧解析器、CRC16 校验、`RawDataPacketV2`、`StatusPacket`。 |
| `tools/monitor/serial_reader.py` | 状态机从固定长度改为读取 header 后按 `payload_len` 收帧；新增 STATUS 信号。 |
| `tools/monitor/raw_quality.py` | 从单纯序号缺口统计升级为 frame gap、sample gap、CRC 错误、STATUS 计数器快照综合统计。 |
| `tools/monitor/raw_data_panel.py` | CSV 保存改为 raw/timeline/quality 三类输出；UI 展示 STATUS 诊断字段。 |
| `tests/test_raw_transport_quality.py` | 增加 V2 协议、CRC、STATUS、缺样本展开测试。 |
| `tests/test_monitor_recording.py` | 增加规则时间轴、NaN 缺失行、插值标记和回绕展开测试。 |
| `docs/上位机UI说明文档.md` | 同步说明新协议、质量指标和导出文件。 |
| `docs/在线心率算法实施文档.md` | 若修改固件协议和采样链路，同步记录。 |

### 7.2 新协议解析器设计

建议分层：

- `FrameDecoder`：只处理字节流同步、长度、CRC，输出通用 `Frame`。
- `parse_raw_data_frame(frame)`：解析 Raw payload。
- `parse_status_frame(frame)`：解析 STATUS payload。
- `RawQualityStats`：只接收结构化事件，不直接依赖串口状态机。
- `TimelineRebuilder`：以 `sample_idx` 为主键生成规则时间轴。

### 7.3 缺帧和缺样本判断逻辑

判断来源：

- `frame_seq` 缺口：说明某些帧没有到达上位机解析成功路径。
- `sample_idx` 缺口：说明采样时间轴存在缺失。
- `STATUS.frame_counter - PC.valid_frame_count`：说明 MCU 组帧多于 PC 收到，缺失在 MCU 组帧之后。
- `STATUS.sample_counter - STATUS.frame_counter`：说明采样多于组帧，缺失更可能在 MCU 采样到组帧之间。
- `STATUS.tx_busy_counter` 增长：说明 UART DMA 发送启动阶段存在拥塞。
- `STATUS.tx_done_counter < tx_start_counter` 长期不追平：说明 DMA 或 UART 发送完成异常。
- PC `_raw_invalid` 增长但 `frame_seq` 不连续：说明帧同步/校验层存在错误，可能是链路噪声或 PC 解析丢同步。

缺样本重建：

1. 首帧建立 `first_sample_idx`。
2. 每个有效 Raw 样本映射到 `global_sample_idx`。
3. 若新样本索引大于期望索引，插入 `[expected, new_idx)` 的 NaN 缺失行。
4. 若新样本索引等于已有索引，标记重复帧，不覆盖原始数据。
5. 若新样本索引小于当前索引，按回绕或乱序规则处理；连续 Raw 流默认不接受乱序。
6. 对短缺失段生成插值列或派生文件，并写 `gap_len`、`interp_method`、`source_left_idx`、`source_right_idx`。

## 8. 建议实施顺序

### 8.1 阶段 A：先建立链路诊断闭环

目标：尽快让系统能回答“缺失发生在哪一段”，而不是一开始就追求完整 V2 协议和时间轴补救。

建议工作：

1. **冻结最小诊断字段**：先确定 STATUS 帧字段，不急着一次性冻结完整 V2 DATA 帧。至少包含 `mcu_time_ms`、`sample_counter`、`frame_counter`、`tx_start_counter`、`tx_done_counter`、`tx_busy_counter`、`tx_error_counter`、`adc_error_counter`、`imu_error_counter`、`ppg_fifo_empty_counter`、`ppg_fifo_overflow_counter`。
2. **固件增加计数器和 STATUS 帧**：采样 tick、ADC DRDY、组帧、发送启动、DMA 完成、HAL_BUSY、HAL_ERROR、传感器异常分别计数。STATUS 建议 1Hz 上报。
3. **上位机兼容 STATUS 解析**：旧 35 字节 Raw 继续可解析；新增 STATUS 帧解析、日志打印、UI/quality summary 展示。若短期协议改动有限，可先用独立帧头或明确 frame type 区分 STATUS。
4. **记录质量事件**：上位机保存 `quality_events.csv` 或 `quality_summary.json`，记录 PC 端 valid frame、invalid frame、sequence gap、STATUS 快照和接收频率。
5. **保持 Raw 原始保存不做插值**：第一阶段不改变原始 CSV 的数据语义，避免把诊断与补救混在一起。

当前阶段 A 的落地实现采用固定长度 STATUS 帧，而不是完整 V2 可变长 DATA 帧：

- STATUS 帧头为 `0xAA 0xDD`，长度 53 字节，1Hz 尝试上报。
- Raw DATA 帧仍保持 35 字节 `0xAA 0xBB` 格式，避免第一阶段改变高频数据语义。
- STATUS payload 包含 `protocol_version` 和 12 个 `uint32 BE` 计数器：`mcu_time_ms`、`sample_counter`、`adc_drdy_counter`、`frame_counter`、`tx_start_counter`、`tx_done_counter`、`tx_busy_counter`、`tx_error_counter`、`adc_error_counter`、`imu_error_counter`、`ppg_fifo_empty_counter`、`ppg_fifo_overflow_counter`。
- 上位机串口状态机新增 STATUS 分支，Raw 面板显示 `Busy`、`Err`、`PCGap`、`FIFO empty/overflow`，录制时同步生成 `raw_data_YYYYMMDD_HHMMSS_status.csv`。
- 第一阶段仍不做 NaN 时间轴展开、插值、ACK 或重传。

阶段 A 的验收标准：

- 运行 10 分钟以上采集后，可以看到 MCU 端采样、组帧、发送启动、发送完成、busy/error 的计数器趋势。
- 上位机能同时给出 PC 端缺帧、无效帧、接收频率和 MCU STATUS 快照。
- 遇到缺口时，至少能初步判断更像 MCU 未组帧、UART/DMA busy、链路/解析丢失，还是 PC 写入侧问题。

### 8.2 阶段 B：采集反馈与定点修复

目标：用阶段 A 的诊断数据定位主要丢失来源，先把真实丢失情况降到最低。

建议采集与反馈内容：

- 采集时长：至少 10-20 分钟；如果目标是验证 20 分钟以上连续采集，建议直接采 25 分钟以上。
- 保存文件：Raw CSV、同名 `_status.csv`、上位机控制台日志；如果 UI 有截图，也保留 RX Hz、DEV Hz、Loss、Diag 和 invalid frame 指标。
- 记录条件：串口波特率、是否经 BLE 透传、是否同时开启 UI 绘图、是否写 CSV、电脑负载、传感器佩戴状态。
- 关键指标：`sample_counter`、`frame_counter`、`tx_start_counter`、`tx_done_counter`、`tx_busy_counter`、`tx_error_counter`、PC valid frame、PC invalid frame、sequence gap、最大连续 gap、gap 分布。

基于反馈的判断规则：

| 现象 | 优先判断 | 优先处理 |
| --- | --- | --- |
| `tx_busy_counter` 持续增长 | UART DMA 发送启动被阻塞 | 先做双缓冲或小环形队列，再评估是否降频或提波特率 |
| `tx_error_counter` 增长 | UART/DMA 错误或 HAL 状态异常 | 查 UART error callback、DMA IRQ、串口硬件链路 |
| `sample_counter` 明显大于 `frame_counter` | 采样到组帧之间丢失 | 查 ADC/IMU/PPG 读取时序、主循环阻塞和状态机 |
| `tx_start_counter` 与 `tx_done_counter` 长期不追平 | DMA 完成回调异常或发送未完成 | 查 `HAL_UART_TxCpltCallback()`、DMA 中断和缓冲生命周期 |
| MCU 计数连续但 PC `sequence gap` 明显 | 链路或 PC 接收解析丢失 | 查串口读取块大小、帧同步状态机、BLE 透传吞吐和 PC 写盘 |
| PC `invalid frame` 增长明显 | 字节损坏、丢同步或解析器误判 | 优先升级 CRC16/帧长度保护，并记录错误上下文 |
| gap 多为 1-3 个样本 | 短暂拥塞或偶发链路抖动 | 可先靠缓冲优化和后续短缺失插值处理 |
| gap 经常成片出现 | 吞吐不足、阻塞或设备重启 | 优先解决链路吞吐和阻塞，不宜先插值掩盖 |

阶段 B 后再决定是否需要：

- 双缓冲；
- 小型发送环形队列；
- 提高 UART 波特率；
- 降低 Raw 输出频率或批量打包；
- 优化上位机读取、解析、绘图和写盘解耦；
- 升级 DATA 帧到 V2 + CRC16。

### 8.3 阶段 C：部署时间轴补救

目标：在主要丢失来源已定位或已缓解后，再把数据保存升级为可长期对齐的格式。

建议工作：

1. **引入全局 `sample_idx`**：优先使用 32 位或上位机展开后的 64 位索引，不再依赖 16 位 `Seq` 直接作为长期唯一主键。
2. **新增 `raw_received.csv`**：只保存真实收到并通过校验的帧，保留 `sample_idx`、`frame_seq`、`mcu_time_ms`、`rx_wall_time_ns`、`crc_ok`、`missing_before`。
3. **新增 `timeline_aligned.csv`**：按 `sample_idx` 展开规则时间轴，缺失样本写显式 NaN 行，使用 `valid_flag`、`interp_flag`、`gap_len` 标记来源。
4. **新增插值派生层**：短 gap 可线性插值，但必须保留原始值、插值标记、左右来源样本；中长 gap 只标记，不伪造。
5. **保留质量摘要**：`quality_summary.json` 记录采集时长、有效样本数、缺失样本数、最大 gap、gap 分布、CRC/invalid frame、STATUS 计数器起止值。

阶段 C 的验收标准：

- 丢包后 `timeline_aligned.csv` 的 `timestamp_s` 不再按接收行数压缩。
- 20 分钟以上采集时，样本序号回绕不会造成全局时间轴混乱。
- 原始数据、缺失占位和插值数据可明确区分，并能追溯到原始帧。

### 8.4 不建议第一阶段投入的内容

以下内容不建议在第一阶段实施，除非阶段 A/B 数据证明确有必要：

- 高频 Raw 全量 ACK/重传；
- 复杂乱序恢复；
- 大型发送缓存；
- 对所有缺失段强制插值；
- 未定位丢失来源前直接重构全部协议和 UI。

这些内容会增加固件、协议和上位机复杂度，也可能掩盖真正瓶颈。第一阶段应优先让链路变得可观测。

## 9. 采集反馈后建议分析流程

当第一阶段诊断机制部署后，建议按以下流程反馈和分析数据：

1. 提供采集文件：Raw CSV、同名 `_status.csv` 和上位机日志。
2. 确认采集条件：采样率、波特率、是否 BLE、采集时长、是否写盘、是否打开实时绘图。
3. 先看计数器守恒关系：
   - `sample_counter` 是否接近理论采样数；
   - `frame_counter` 是否接近 `sample_counter`；
   - `tx_start_counter + tx_busy_counter + tx_error_counter` 是否接近 `frame_counter`；
   - `tx_done_counter` 是否接近 `tx_start_counter`；
   - PC valid frame 是否接近 `tx_done_counter`。
4. 再看 gap 分布：短 gap、长 gap、周期性 gap、集中爆发 gap 分开判断。
5. 最后决定修复路径：固件缓冲、UART/BLE 吞吐、上位机读取解析、写盘解耦或协议校验升级。

这个闭环可以避免凭猜测修改协议或插值策略，也便于逐步把数据丢失降到最低。

## 10. 结论

当前系统已经具备基础链路质量统计：Raw 帧有 16 位 `Seq`，上位机能计算缺口、丢包率、接收频率和设备侧频率，并在 CSV 写入 `MissingBefore`。

但当前系统还不具备完整链路诊断和长期时间轴补救能力：

- 不能定位丢失发生在 MCU 采样、组帧、发送、链路还是 PC 端。
- 没有 STATUS 状态帧和 MCU 端关键计数器。
- Raw 校验仍是 XOR，建议升级 CRC16。
- CSV 没有显式缺失行，`Time(s)` 会因丢包按接收行数压缩。
- 原始数据和插值数据没有分层保存。

推荐方向是：先增加轻量诊断字段和 STATUS 帧，采集一段真实数据后根据计数器和 gap 分布定位主要丢失来源，优先把实际丢失降到最低；随后再让上位机按 `sample_idx` 重建规则时间轴，缺失点先 `NaN`，短缺失可插值，中长缺失只标记。ACK/重传不作为第一阶段目标。

因此，链路诊断和时间轴补救分段实施更合理：第一阶段解决“看清楚哪里丢”，第二阶段解决“尽量少丢”，第三阶段解决“已经丢失的数据如何仍可分析和对齐”。
