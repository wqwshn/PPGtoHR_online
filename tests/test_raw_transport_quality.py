import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_DIR = ROOT / "tools" / "monitor"
sys.path.insert(0, str(MONITOR_DIR))

import protocol
from raw_quality import RawQualityStats


MAIN_H = (ROOT / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")
MAIN_C = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")


def make_raw_packet(sequence: int) -> bytes:
    data = bytearray(protocol.RAW_PACKET_LEN)
    data[0] = protocol.RAW_HEADER_BYTE_0
    data[1] = protocol.RAW_HEADER_BYTE_1
    data[22:25] = bytes([0x00, 0x12, 0x34])
    data[25:28] = bytes([0x00, 0x56, 0x78])
    data[28:31] = bytes([0x00, 0x7A, 0xBC])
    data[31] = (sequence >> 8) & 0xFF
    data[32] = sequence & 0xFF
    xor_val = 0
    for idx in range(protocol.RAW_XOR_START, protocol.RAW_XOR_END + 1):
        xor_val ^= data[idx]
    data[protocol.RAW_XOR_POS] = xor_val
    data[34] = protocol.RAW_FOOTER_BYTE
    return bytes(data)


def make_status_packet(**values: int) -> bytes:
    data = bytearray(protocol.STATUS_PACKET_LEN)
    data[0] = protocol.RAW_HEADER_BYTE_0
    data[1] = protocol.STATUS_HEADER_BYTE_1
    data[2] = protocol.RAW_DIAG_PROTOCOL_VERSION

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
    pos = 3
    for value in fields:
        data[pos:pos + 4] = int(value).to_bytes(4, "big")
        pos += 4

    xor_val = 0
    for idx in range(protocol.STATUS_XOR_START, protocol.STATUS_XOR_END + 1):
        xor_val ^= data[idx]
    data[protocol.STATUS_XOR_POS] = xor_val
    data[protocol.STATUS_FOOTER_POS] = protocol.RAW_FOOTER_BYTE
    return bytes(data)


def test_raw_packet_parses_35_byte_sequence_field():
    assert protocol.RAW_PACKET_LEN == 35
    assert protocol.RAW_XOR_END == 32
    assert protocol.RAW_XOR_POS == 33

    pkt = protocol.parse_raw_packet(make_raw_packet(0x1234))

    assert pkt is not None
    assert pkt.sequence == 0x1234
    assert pkt.ppg_green == 0x1234
    assert pkt.ppg_red == 0x5678
    assert pkt.ppg_ir == 0x7ABC


def test_raw_quality_stats_count_sequence_gaps_and_loss_rate():
    stats = RawQualityStats()

    assert stats.observe(10) == 0
    assert stats.observe(11) == 0
    assert stats.observe(15) == 3

    assert stats.received_count == 3
    assert stats.expected_count == 6
    assert stats.missing_count == 3
    assert math.isclose(stats.loss_rate, 0.5)


def test_raw_quality_stats_handle_uint16_wraparound():
    stats = RawQualityStats()

    assert stats.observe(0xFFFE) == 0
    assert stats.observe(0x0000) == 1

    assert stats.received_count == 2
    assert stats.expected_count == 3
    assert stats.missing_count == 1


def test_status_packet_parses_diagnostic_counters():
    assert protocol.STATUS_HEADER_BYTE_1 == 0xDD
    assert protocol.STATUS_PACKET_LEN == 53
    assert protocol.STATUS_XOR_POS == 51

    pkt = protocol.parse_status_packet(make_status_packet(tx_busy_counter=7))

    assert pkt is not None
    assert pkt.protocol_version == protocol.RAW_DIAG_PROTOCOL_VERSION
    assert pkt.mcu_time_ms == 1234
    assert pkt.sample_counter == 100
    assert pkt.adc_drdy_counter == 400
    assert pkt.frame_counter == 99
    assert pkt.tx_start_counter == 98
    assert pkt.tx_done_counter == 97
    assert pkt.tx_busy_counter == 7
    assert pkt.tx_error_counter == 2
    assert pkt.adc_error_counter == 3
    assert pkt.imu_error_counter == 4
    assert pkt.ppg_fifo_empty_counter == 5
    assert pkt.ppg_fifo_overflow_counter == 6


def test_status_packet_rejects_corrupted_xor():
    data = bytearray(make_status_packet())
    data[10] ^= 0x01

    assert protocol.parse_status_packet(bytes(data)) is None


def test_raw_quality_stats_store_status_snapshot_and_pc_gap():
    stats = RawQualityStats()
    stats.observe(10)
    stats.observe(12)
    status = protocol.parse_status_packet(
        make_status_packet(tx_done_counter=5, tx_start_counter=6)
    )

    assert status is not None
    snapshot = stats.observe_status(status)

    assert stats.latest_status is status
    assert snapshot.pc_missing_after_tx_done == 3
    assert snapshot.tx_inflight == 1


def test_firmware_raw_packet_declares_sequence_extended_layout():
    assert "#define PACKET_LEN 35" in MAIN_H
    assert "#define XOR_CHECK_LEN 31" in MAIN_H
    assert "#define RAW_SEQUENCE_START_INDEX  31" in MAIN_H
    assert "allData[RAW_SEQUENCE_START_INDEX]" in MAIN_C
    assert "allData[RAW_SEQUENCE_START_INDEX + 1]" in MAIN_C
    assert "raw_packet_seq++" in MAIN_C


def test_firmware_declares_phase_a_status_diagnostics():
    assert "#define STATUS_HEADER_BYTE_1 0xDD" in MAIN_H
    assert "#define STATUS_PACKET_LEN 53" in MAIN_H
    assert "#define RAW_DIAG_PROTOCOL_VERSION" in MAIN_H
    assert "BuildStatusFrame" in MAIN_C
    assert "HAL_UART_TxCpltCallback" in MAIN_C
    assert "raw_diag_tx_busy_counter++" in MAIN_C
    assert "raw_diag_ppg_fifo_empty_counter++" in MAIN_C
