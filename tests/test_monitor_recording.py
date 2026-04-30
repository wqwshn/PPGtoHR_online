import sys
from types import SimpleNamespace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_DIR = ROOT / "tools" / "monitor"
sys.path.insert(0, str(MONITOR_DIR))

import raw_data_panel
import serial_reader
import protocol


class FakeSerial:
    def __init__(self, in_waiting):
        self.in_waiting = in_waiting
        self.read_sizes = []

    def read(self, size):
        self.read_sizes.append(size)
        return b"x" * size


def test_serial_reader_does_not_wait_for_a_full_4096_byte_buffer():
    fake = FakeSerial(in_waiting=4096)

    chunk = serial_reader.read_serial_chunk(fake)

    assert len(chunk) == serial_reader.SERIAL_READ_CHUNK_BYTES
    assert fake.read_sizes == [serial_reader.SERIAL_READ_CHUNK_BYTES]
    assert serial_reader.SERIAL_READ_CHUNK_BYTES == max(
        serial_reader.RAW_PACKET_LEN,
        serial_reader.STATUS_PACKET_LEN,
    ) * 4


def test_serial_reader_blocks_for_only_one_byte_when_no_data_is_buffered():
    fake = FakeSerial(in_waiting=0)

    chunk = serial_reader.read_serial_chunk(fake)

    assert len(chunk) == 1
    assert fake.read_sizes == [1]


def test_raw_recording_time_uses_sample_index_not_ui_wall_clock():
    assert raw_data_panel.sample_index_to_elapsed_seconds(0) == 0.0
    assert raw_data_panel.sample_index_to_elapsed_seconds(1) == 0.01
    assert raw_data_panel.sample_index_to_elapsed_seconds(23999) == 239.99


def test_raw_csv_exports_sequence_and_missing_count():
    pkt = SimpleNamespace(
        sequence=42,
        Uc1=1.0,
        Uc2=2.0,
        Ut1=3.0,
        Ut2=4.0,
        acc_x=0.1,
        acc_y=0.2,
        acc_z=0.3,
        gyro_x=1.1,
        gyro_y=1.2,
        gyro_z=1.3,
        ppg_green=100,
        ppg_red=200,
        ppg_ir=300,
    )

    assert raw_data_panel.RAW_CSV_HEADER[:3] == ["Time(s)", "Seq", "MissingBefore"]
    assert raw_data_panel.raw_packet_to_csv_row(pkt, 1.23, 2)[:3] == [1.23, 42, 2]


def test_status_summary_exposes_diagnostic_counters():
    status = protocol.StatusPacket(
        protocol_version=1,
        mcu_time_ms=1234,
        sample_counter=100,
        adc_drdy_counter=400,
        frame_counter=99,
        tx_start_counter=98,
        tx_done_counter=97,
        tx_busy_counter=7,
        tx_error_counter=2,
        adc_error_counter=3,
        imu_error_counter=4,
        ppg_fifo_empty_counter=5,
        ppg_fifo_overflow_counter=6,
    )
    snapshot = raw_data_panel.RawQualityStats().observe_status(status)

    text = raw_data_panel.status_packet_to_summary(status, snapshot)

    assert "Busy 7" in text
    assert "Err 2" in text
    assert "PCGap 97" in text
    assert "FIFO 5/6" in text


def test_status_csv_row_exports_diagnostic_snapshot():
    status = protocol.StatusPacket(
        protocol_version=1,
        mcu_time_ms=1234,
        sample_counter=100,
        adc_drdy_counter=400,
        frame_counter=99,
        tx_start_counter=98,
        tx_done_counter=97,
        tx_busy_counter=7,
        tx_error_counter=2,
        adc_error_counter=3,
        imu_error_counter=4,
        ppg_fifo_empty_counter=5,
        ppg_fifo_overflow_counter=6,
    )
    snapshot = raw_data_panel.RawQualityStats().observe_status(status)

    assert raw_data_panel.STATUS_CSV_HEADER[:4] == [
        "RxTime(s)",
        "McuTime(ms)",
        "SampleCounter",
        "FrameCounter",
    ]
    assert raw_data_panel.status_packet_to_csv_row(status, snapshot, 1.25)[:4] == [
        1.25,
        1234,
        100,
        99,
    ]
