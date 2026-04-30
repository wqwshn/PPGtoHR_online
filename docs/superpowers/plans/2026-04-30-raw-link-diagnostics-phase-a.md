# Raw Link Diagnostics Phase A Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a first-stage Raw link diagnostic loop that reports MCU sampling, framing, UART/DMA, and PC parser counters without deploying timeline repair.

**Architecture:** Keep the existing 35-byte Raw DATA frame intact. Add a fixed-length 1Hz STATUS frame (`0xAA 0xDD`) with XOR validation and diagnostic counters. The PC monitor parses STATUS frames alongside HR and Raw frames, stores the latest diagnostic snapshot in `RawQualityStats`, and exposes it through tests and the Raw panel.

**Tech Stack:** STM32 HAL C firmware, Python dataclasses, PyQt5 signals, pytest source/behavior regression tests, Markdown docs.

**Execution Status:** Implemented in this session. Focused pytest and full pytest passed. Full `cmake --build build -j4` and `ninja -C build -v` both timed out in the existing build directory before producing a useful build result; `Core/Src/main.c` was compiled directly with the command from `build/compile_commands.json` and exited 0 with existing warnings only.

---

### Task 1: STATUS Protocol Tests

**Files:**
- Modify: `tests/test_raw_transport_quality.py`
- Modify: `tools/monitor/protocol.py`
- Modify: `tools/monitor/raw_quality.py`

- [ ] **Step 1: Write failing tests for STATUS parsing**

Add tests that construct a fixed STATUS frame with:

```python
def make_status_packet(**values: int) -> bytes:
    fields = [
        values.get("mcu_time_ms", 1234),
        values.get("sample_counter", 100),
        values.get("adc_drdy_counter", 400),
        values.get("frame_counter", 99),
        values.get("tx_start_counter", 98),
        values.get("tx_done_counter", 97),
        values.get("tx_busy_counter", 1),
        values.get("tx_error_counter", 2),
        values.get("adc_error_counter", 3),
        values.get("imu_error_counter", 4),
        values.get("ppg_fifo_empty_counter", 5),
        values.get("ppg_fifo_overflow_counter", 6),
    ]
```

Expected behavior:
- `protocol.STATUS_PACKET_LEN == 53`
- `parse_status_packet()` returns `StatusPacket`
- corrupted XOR returns `None`
- `RawQualityStats.observe_status()` stores the latest packet and computes `pc_missing_after_tx_done`

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
python -m pytest -q tests/test_raw_transport_quality.py
```

Expected: fails because STATUS constants, parser, and quality snapshot do not exist.

- [ ] **Step 3: Implement protocol and quality code**

Add `StatusPacket`, STATUS constants, big-endian uint32 parser helpers, and `RawQualityStats.observe_status(status)`.

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```powershell
python -m pytest -q tests/test_raw_transport_quality.py
```

Expected: all tests in the file pass.

### Task 2: Serial Reader STATUS Dispatch

**Files:**
- Modify: `tools/monitor/serial_reader.py`
- Modify: `tests/test_monitor_recording.py`

- [ ] **Step 1: Write failing tests for packet length and imports**

Assert `SERIAL_READ_CHUNK_BYTES` is sized for the largest monitor frame, and the serial reader imports `STATUS_PACKET_LEN`, `STATUS_HEADER_BYTE_1`, `parse_status_packet`, and `StatusPacket`.

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
python -m pytest -q tests/test_monitor_recording.py
```

Expected: fails until serial reader handles STATUS frames.

- [ ] **Step 3: Add STATUS signal and frame branch**

Add `status_packet_received = pyqtSignal(StatusPacket)`. In state 1, handle second header byte `0xDD`, set `expected_len = STATUS_PACKET_LEN`, and on full frame call `parse_status_packet()`.

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```powershell
python -m pytest -q tests/test_monitor_recording.py
```

Expected: all tests in the file pass.

### Task 3: Raw Panel Diagnostic Snapshot

**Files:**
- Modify: `tools/monitor/raw_data_panel.py`
- Modify: `tools/monitor/main.py`
- Modify: `tests/test_monitor_recording.py`

- [ ] **Step 1: Write failing tests for status summary text**

Create a `StatusPacket` instance and verify `raw_data_panel.status_packet_to_summary()` includes `Busy`, `Err`, `PCGap`, and counter values.

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
python -m pytest -q tests/test_monitor_recording.py
```

Expected: fails because `status_packet_to_summary()` and panel status handling do not exist.

- [ ] **Step 3: Implement panel status handling**

Add a compact diagnostic label to the Raw info bar, implement `handle_status_data(status)`, store the latest status in `RawQualityStats`, and connect `SerialReader.status_packet_received` in `tools/monitor/main.py`.

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```powershell
python -m pytest -q tests/test_monitor_recording.py
```

Expected: all tests in the file pass.

### Task 4: Firmware Diagnostic Counters and STATUS Frame

**Files:**
- Modify: `Core/Inc/main.h`
- Modify: `Core/Src/main.c`
- Modify: `tests/test_raw_transport_quality.py`

- [ ] **Step 1: Write source-inspection tests**

Assert firmware defines:
- `STATUS_HEADER_BYTE_1 0xDD`
- `STATUS_PACKET_LEN 53`
- `RAW_DIAG_PROTOCOL_VERSION`
- `BuildStatusFrame`
- `HAL_UART_TxCpltCallback`
- `raw_diag_tx_busy_counter++`
- `raw_diag_ppg_fifo_empty_counter++`

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
python -m pytest -q tests/test_raw_transport_quality.py
```

Expected: fails until firmware declarations and counters are present.

- [ ] **Step 3: Implement firmware diagnostics**

Add volatile counters, `WriteU32BE()`, `BuildStatusFrame()`, and `TryTransmitStatusFrame()`. Increment sample tick, ADC DRDY, frame, TX OK/busy/error, TX done, and PPG FIFO empty/overflow counters.

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```powershell
python -m pytest -q tests/test_raw_transport_quality.py
```

Expected: all tests in the file pass.

### Task 5: Documentation and Full Verification

**Files:**
- Modify: `docs/raw_data_link_upgrade_plan.md`
- Modify: `docs/上位机UI说明文档.md`
- Modify: `docs/在线心率算法实施文档.md`

- [ ] **Step 1: Update docs**

Document Phase A STATUS frame format, counters, and collection workflow.

- [ ] **Step 2: Run focused tests**

Run:

```powershell
python -m pytest -q tests/test_raw_transport_quality.py tests/test_monitor_recording.py
```

Expected: all focused tests pass.

- [ ] **Step 3: Run full pytest suite**

Run:

```powershell
python -m pytest -q tests
```

Expected: all repository tests pass.

- [ ] **Step 4: Review diff**

Run:

```powershell
git diff -- Core/Inc/main.h Core/Src/main.c tools/monitor/protocol.py tools/monitor/serial_reader.py tools/monitor/raw_quality.py tools/monitor/raw_data_panel.py tools/monitor/main.py tests/test_raw_transport_quality.py tests/test_monitor_recording.py docs/raw_data_link_upgrade_plan.md docs/上位机UI说明文档.md docs/在线心率算法实施文档.md
```

Expected: diff only contains Phase A diagnostic changes.
