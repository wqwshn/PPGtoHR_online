"""
PPG Monitor - 原始传感器数据可视化面板

暗色主题实时波形面板, 接收 35 字节多光谱原始传感器数据包 (100Hz),
展示 PPG 三通道波形 (Green/Red/IR) / 热膜桥压 / 三轴加速度 / 陀螺仪角速度.
"""
from __future__ import annotations

import csv
import math
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
)
import pyqtgraph as pg

from protocol import RawDataPacket, StatusPacket
from raw_quality import DiagnosticSnapshot, RawQualityStats
from realtime_hr import timed_estimate_green_fft_hr

# 复用 dashboard 的配色常量
from dashboard import (
    COLOR_BG, COLOR_CARD, COLOR_CARD_BORDER,
    COLOR_PRIMARY, COLOR_TEXT, COLOR_TEXT_DIM,
    COLOR_GREEN, COLOR_ORANGE, COLOR_RED,
    TRANSLATIONS,
)

# 数据缓冲区大小 (存储量, 大于可见窗口以平滑过渡)
PLOT_BUFFER = 2000
# 可见窗口大小 (实际绘制点数)
VISIBLE_POINTS = 800
RAW_RECORD_SAMPLE_RATE_HZ = 100.0
RAW_CSV_HEADER = [
    "Time(s)", "Seq", "MissingBefore",
    "Uc1(mV)", "Uc2(mV)", "Ut1(mV)", "Ut2(mV)",
    "AccX(g)", "AccY(g)", "AccZ(g)",
    "GyroX(dps)", "GyroY(dps)", "GyroZ(dps)",
    "PPG_Green", "PPG_Red", "PPG_IR",
]
STATUS_CSV_HEADER = [
    "RxTime(s)", "McuTime(ms)", "SampleCounter", "FrameCounter",
    "TxStartCounter", "TxDoneCounter", "TxBusyCounter", "TxErrorCounter",
    "AdcDrdyCounter", "AdcErrorCounter", "ImuErrorCounter",
    "PpgFifoEmptyCounter", "PpgFifoOverflowCounter",
    "PcReceivedRaw", "PcExpectedRaw", "PcMissingRaw",
    "PcMissingAfterTxDone", "TxInflight",
]


def sample_index_to_elapsed_seconds(
    sample_index: int,
    sample_rate_hz: float = RAW_RECORD_SAMPLE_RATE_HZ,
) -> float:
    """Return deterministic sample time for fixed-rate raw recording."""
    return round(sample_index / sample_rate_hz, 3)


def raw_packet_to_csv_row(pkt: RawDataPacket, elapsed: float, missing_before: int) -> list:
    return [
        elapsed,
        pkt.sequence,
        missing_before,
        round(pkt.Uc1, 5), round(pkt.Uc2, 5),
        round(pkt.Ut1, 5), round(pkt.Ut2, 5),
        round(pkt.acc_x, 5), round(pkt.acc_y, 5), round(pkt.acc_z, 5),
        round(pkt.gyro_x, 3), round(pkt.gyro_y, 3), round(pkt.gyro_z, 3),
        pkt.ppg_green, pkt.ppg_red, pkt.ppg_ir,
    ]


def status_packet_to_summary(
    status: StatusPacket,
    snapshot: DiagnosticSnapshot,
) -> str:
    return (
        f"Diag: Busy {status.tx_busy_counter} | Err {status.tx_error_counter} | "
        f"PCGap {snapshot.pc_missing_after_tx_done} | "
        f"FIFO {status.ppg_fifo_empty_counter}/{status.ppg_fifo_overflow_counter}"
    )


def status_packet_to_csv_row(
    status: StatusPacket,
    snapshot: DiagnosticSnapshot,
    rx_elapsed: float,
) -> list:
    return [
        round(rx_elapsed, 3),
        status.mcu_time_ms,
        status.sample_counter,
        status.frame_counter,
        status.tx_start_counter,
        status.tx_done_counter,
        status.tx_busy_counter,
        status.tx_error_counter,
        status.adc_drdy_counter,
        status.adc_error_counter,
        status.imu_error_counter,
        status.ppg_fifo_empty_counter,
        status.ppg_fifo_overflow_counter,
        snapshot.pc_received_raw,
        snapshot.pc_expected_raw,
        snapshot.pc_missing_raw,
        snapshot.pc_missing_after_tx_done,
        snapshot.tx_inflight,
    ]


class RawDataPanel(QWidget):
    """原始传感器数据可视化面板"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lang = "zh"

        # 数据缓冲区
        self._data_Uc1 = deque(maxlen=PLOT_BUFFER)
        self._data_Uc2 = deque(maxlen=PLOT_BUFFER)
        self._data_Ut1 = deque(maxlen=PLOT_BUFFER)
        self._data_Ut2 = deque(maxlen=PLOT_BUFFER)
        self._data_Accx = deque(maxlen=PLOT_BUFFER)
        self._data_Accy = deque(maxlen=PLOT_BUFFER)
        self._data_Accz = deque(maxlen=PLOT_BUFFER)
        self._data_Gyrox = deque(maxlen=PLOT_BUFFER)
        self._data_Gyroy = deque(maxlen=PLOT_BUFFER)
        self._data_Gyroz = deque(maxlen=PLOT_BUFFER)
        self._data_ppg_g = deque(maxlen=PLOT_BUFFER)
        self._data_ppg_r = deque(maxlen=PLOT_BUFFER)
        self._data_ppg_ir = deque(maxlen=PLOT_BUFFER)

        # 状态
        self._packet_count = 0
        self._sample_count = 0  # 每秒重置, 用于采样率计算
        self._start_time = time.time()
        self._quality = RawQualityStats()
        self._last_missing_before = 0
        self._last_rx_hz = 0
        self._last_device_hz = 0
        self._last_expected_count = 0
        self._last_hr_calc_ms: Optional[float] = None
        self._last_status: Optional[StatusPacket] = None

        # 信息条最新包缓存 (按帧率更新, 不逐包刷新)
        self._last_pkt: Optional[RawDataPacket] = None

        # 录制
        self._is_recording = False
        self._csv_file = None
        self._csv_writer = None
        self._status_csv_file = None
        self._status_csv_writer = None
        self._recording_start_time: Optional[float] = None
        self._recorded_sample_count = 0
        self._flush_counter = 0

        self._init_ui()

        # 波形刷新定时器 (33ms = 30FPS)
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._update_plots)
        self._plot_timer.start(33)

        # 采样率计算定时器 (1s)
        self._rate_timer = QTimer(self)
        self._rate_timer.timeout.connect(self._update_sample_rate)
        self._rate_timer.start(1000)

        # 1Hz 轻量绿光 PPG FFT 估计。只读取现有显示缓冲区，
        # 不进入串口解析和 CSV 逐包写入路径。
        self._hr_timer = QTimer(self)
        self._hr_timer.timeout.connect(self._update_realtime_hr)
        self._hr_timer.start(1000)

        # 模拟定时器
        self._sim_timer = QTimer(self)
        self._sim_timer.timeout.connect(self._sim_tick)
        self._sim_step = 0

    # ── UI 构建 ──────────────────────────────────────────

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        layout.setContentsMargins(8, 8, 8, 8)

        # 顶部信息条
        layout.addWidget(self._build_info_bar())

        # PPG 三通道波形区: 左=Green, 右上=Red, 右下=IR
        ppg_widget = QWidget()
        ppg_layout = QHBoxLayout(ppg_widget)
        ppg_layout.setContentsMargins(0, 0, 0, 0)
        ppg_layout.setSpacing(6)

        # 左侧: 绿光
        self._plot_ppg_g, self._curve_ppg_g = self._make_plot(
            "PPG Green", COLOR_GREEN
        )
        ppg_layout.addWidget(self._plot_ppg_g, 1)

        # 右侧: 红光 + 红外 上下排列
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)
        self._plot_ppg_r, self._curve_ppg_r = self._make_plot("PPG Red", COLOR_RED)
        self._plot_ppg_ir, self._curve_ppg_ir = self._make_plot("PPG IR", "#3B82F6")
        right_layout.addWidget(self._plot_ppg_r)
        right_layout.addWidget(self._plot_ppg_ir)
        ppg_layout.addWidget(right, 1)

        layout.addWidget(ppg_widget, 3)

        # 桥顶电压 Ut1, Ut2
        ut_widget = QWidget()
        ut_layout = QHBoxLayout(ut_widget)
        ut_layout.setContentsMargins(0, 0, 0, 0)
        ut_layout.setSpacing(6)
        self._plot_Ut1, self._curve_Ut1 = self._make_plot("Ut1 (mV)", COLOR_ORANGE)
        self._plot_Ut2, self._curve_Ut2 = self._make_plot("Ut2 (mV)", "#D946EF")
        ut_layout.addWidget(self._plot_Ut1)
        ut_layout.addWidget(self._plot_Ut2)
        layout.addWidget(ut_widget, 2)

        # 桥中电压 Uc1, Uc2
        uc_widget = QWidget()
        uc_layout = QHBoxLayout(uc_widget)
        uc_layout.setContentsMargins(0, 0, 0, 0)
        uc_layout.setSpacing(6)
        self._plot_Uc1, self._curve_Uc1 = self._make_plot("Uc1 (mV)", COLOR_RED)
        self._plot_Uc2, self._curve_Uc2 = self._make_plot("Uc2 (mV)", "#3B82F6")
        uc_layout.addWidget(self._plot_Uc1)
        uc_layout.addWidget(self._plot_Uc2)
        layout.addWidget(uc_widget, 1)

        # MIMU 区域: 左=ACC, 右=GYRO (水平布局)
        imu_widget = QWidget()
        imu_layout = QHBoxLayout(imu_widget)
        imu_layout.setContentsMargins(0, 0, 0, 0)
        imu_layout.setSpacing(6)

        # 三轴加速度
        self._plot_acc, self._curve_accx = self._make_plot("ACC (g)", COLOR_RED)
        self._curve_accy = self._plot_acc.plot(pen=pg.mkPen(COLOR_GREEN, width=1.5))
        self._curve_accz = self._plot_acc.plot(pen=pg.mkPen("#3B82F6", width=1.5))
        self._plot_acc.addLegend(offset=(60, 10))
        imu_layout.addWidget(self._plot_acc)

        # 三轴陀螺仪
        self._plot_gyro, self._curve_gyrox = self._make_plot("GYRO (dps)", COLOR_RED)
        self._curve_gyroy = self._plot_gyro.plot(pen=pg.mkPen(COLOR_GREEN, width=1.5))
        self._curve_gyroz = self._plot_gyro.plot(pen=pg.mkPen("#3B82F6", width=1.5))
        self._plot_gyro.addLegend(offset=(60, 10))
        imu_layout.addWidget(self._plot_gyro)

        layout.addWidget(imu_widget, 2)

    def _build_info_bar(self) -> QFrame:
        """顶部状态信息条"""
        frame = QFrame()
        frame.setObjectName("card")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 6, 12, 6)

        self._lbl_mode = QLabel("Mode: --")
        self._lbl_mode.setStyleSheet(
            f"color: {COLOR_PRIMARY}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_mode)

        self._lbl_rate = QLabel("Rate: RX 0 Hz | DEV 0 Hz")
        self._lbl_rate.setStyleSheet(
            f"color: {COLOR_ORANGE}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_rate)

        self._lbl_loss = QLabel("Loss: 0.00% (0/0)")
        self._lbl_loss.setStyleSheet(
            f"color: {COLOR_RED}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_loss)

        self._lbl_realtime_hr = QLabel("实时心率: --")
        self._lbl_realtime_hr.setStyleSheet(
            f"color: {COLOR_TEXT_DIM}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_realtime_hr)

        self._lbl_diag = QLabel("Diag: --")
        self._lbl_diag.setStyleSheet(
            f"color: {COLOR_TEXT_DIM}; font-size: 12px; font-weight: bold;"
        )
        layout.addWidget(self._lbl_diag)

        layout.addStretch()

        self._lbl_calib = QLabel("")
        self._lbl_calib.setStyleSheet(
            f"color: {COLOR_GREEN}; font-size: 12px; font-weight: bold;"
        )
        self._lbl_calib.setVisible(False)
        layout.addWidget(self._lbl_calib)

        self._lbl_count = QLabel("Packets: 0")
        self._lbl_count.setStyleSheet(
            f"color: {COLOR_TEXT_DIM}; font-size: 12px;"
        )
        layout.addWidget(self._lbl_count)

        return frame

    def _make_plot(self, title: str, color: str) -> tuple[pg.PlotWidget, pg.PlotDataItem]:
        """创建一个暗色风格的 pyqtgraph 绘图组件"""
        pw = pg.PlotWidget()
        pw.setTitle(title, color=COLOR_TEXT_DIM, size="10pt")
        pw.showGrid(x=True, y=True, alpha=0.12)
        pw.getAxis("left").setPen(pg.mkPen(COLOR_TEXT_DIM))
        pw.getAxis("bottom").setPen(pg.mkPen(COLOR_TEXT_DIM))
        pw.setBackground(COLOR_CARD)
        curve = pw.plot(pen=pg.mkPen(color, width=1.5))
        return pw, curve

    # ── 数据处理 ─────────────────────────────────────────

    def _reset_quality_stats(self):
        self._quality.reset()
        self._last_missing_before = 0
        self._last_rx_hz = 0
        self._last_device_hz = 0
        self._last_expected_count = 0
        self._sample_count = 0

    def handle_raw_data(self, pkt: RawDataPacket):
        """接收并缓存一个多光谱原始数据包"""
        self._sample_count += 1
        self._packet_count += 1
        missing_before = self._quality.observe(pkt.sequence)
        self._last_missing_before = missing_before

        # 追加数据缓冲区
        self._data_Uc1.append(pkt.Uc1)
        self._data_Uc2.append(pkt.Uc2)
        self._data_Ut1.append(pkt.Ut1)
        self._data_Ut2.append(pkt.Ut2)
        self._data_Accx.append(pkt.acc_x)
        self._data_Accy.append(pkt.acc_y)
        self._data_Accz.append(pkt.acc_z)
        self._data_Gyrox.append(pkt.gyro_x)
        self._data_Gyroy.append(pkt.gyro_y)
        self._data_Gyroz.append(pkt.gyro_z)
        self._data_ppg_g.append(pkt.ppg_green)
        self._data_ppg_r.append(pkt.ppg_red)
        self._data_ppg_ir.append(pkt.ppg_ir)

        # CSV 实时写入
        if self._is_recording and self._csv_writer:
            elapsed = sample_index_to_elapsed_seconds(self._recorded_sample_count)
            self._csv_writer.writerow(raw_packet_to_csv_row(pkt, elapsed, missing_before))
            self._recorded_sample_count += 1
            self._flush_counter += 1
            if self._flush_counter >= 100:
                self._csv_file.flush()
                self._flush_counter = 0

        # 缓存最新包, 信息条由 _update_plots 按帧率更新
        self._last_pkt = pkt

    def _update_info_bar(self, pkt: RawDataPacket):
        t = TRANSLATIONS[self._lang]
        self._lbl_mode.setText(
            f"{t.get('mode', 'Mode')}: Multi-LED (G+R+IR)"
        )
        self._lbl_loss.setText(
            f"{t.get('packet_loss', 'Loss')}: {self._quality.loss_rate * 100:.2f}% "
            f"({self._quality.missing_count}/{self._quality.expected_count})"
        )
        self._lbl_count.setText(
            f"{t.get('pkt_count', 'Packets')}: {self._quality.received_count}"
        )

    def handle_calib_status(self, text: str):
        """处理固件发送的标定状态文本"""
        self._lbl_calib.setVisible(True)
        self._lbl_calib.setText(text)
        # 根据内容切换颜色
        if "OK" in text or "saved" in text:
            self._lbl_calib.setStyleSheet(
                f"color: {COLOR_GREEN}; font-size: 12px; font-weight: bold;"
            )
        elif "REJECTED" in text:
            self._lbl_calib.setStyleSheet(
                f"color: {COLOR_RED}; font-size: 12px; font-weight: bold;"
            )
        else:
            self._lbl_calib.setStyleSheet(
                f"color: {COLOR_ORANGE}; font-size: 12px; font-weight: bold;"
            )

    def handle_status_data(self, status: StatusPacket):
        """处理固件 1Hz Raw 链路诊断状态帧。"""
        snapshot = self._quality.observe_status(status)
        self._last_status = status
        self._lbl_diag.setText(status_packet_to_summary(status, snapshot))
        if self._is_recording and self._status_csv_writer:
            start = self._recording_start_time or time.time()
            elapsed = time.time() - start
            self._status_csv_writer.writerow(status_packet_to_csv_row(status, snapshot, elapsed))
            self._status_csv_file.flush()

    def _update_plots(self):
        """33ms 定时刷新波形, 仅绘制最近 VISIBLE_POINTS 个点"""
        if len(self._data_Uc1) == 0:
            return

        # PPG 三通道波形 (裁剪至可见窗口)
        self._curve_ppg_g.setData(list(self._data_ppg_g)[-VISIBLE_POINTS:])
        self._curve_ppg_r.setData(list(self._data_ppg_r)[-VISIBLE_POINTS:])
        self._curve_ppg_ir.setData(list(self._data_ppg_ir)[-VISIBLE_POINTS:])

        # 桥压波形
        self._curve_Ut1.setData(list(self._data_Ut1)[-VISIBLE_POINTS:])
        self._curve_Ut2.setData(list(self._data_Ut2)[-VISIBLE_POINTS:])
        self._curve_Uc1.setData(list(self._data_Uc1)[-VISIBLE_POINTS:])
        self._curve_Uc2.setData(list(self._data_Uc2)[-VISIBLE_POINTS:])

        # 加速度计波形
        self._curve_accx.setData(list(self._data_Accx)[-VISIBLE_POINTS:])
        self._curve_accy.setData(list(self._data_Accy)[-VISIBLE_POINTS:])
        self._curve_accz.setData(list(self._data_Accz)[-VISIBLE_POINTS:])

        # 陀螺仪波形
        self._curve_gyrox.setData(list(self._data_Gyrox)[-VISIBLE_POINTS:])
        self._curve_gyroy.setData(list(self._data_Gyroy)[-VISIBLE_POINTS:])
        self._curve_gyroz.setData(list(self._data_Gyroz)[-VISIBLE_POINTS:])

        # 信息条按帧率更新
        if self._last_pkt is not None:
            self._update_info_bar(self._last_pkt)

    def _update_sample_rate(self):
        """1秒定时器: 刷新采样率"""
        t = TRANSLATIONS[self._lang]
        current_expected = self._quality.expected_count
        self._last_rx_hz = self._sample_count
        self._last_device_hz = max(0, current_expected - self._last_expected_count)
        self._last_expected_count = current_expected
        self._lbl_rate.setText(
            f"{t.get('sample_rate', 'Rate')}: "
            f"RX {self._last_rx_hz} Hz | DEV {self._last_device_hz} Hz"
        )
        self._sample_count = 0

    # ── 录制 ─────────────────────────────────────────────

    def _update_realtime_hr(self):
        """每秒从绿光 PPG 估计一次心率，不触碰 IO 路径。"""
        t = TRANSLATIONS[self._lang]
        estimate = timed_estimate_green_fft_hr(list(self._data_ppg_g))
        self._last_hr_calc_ms = estimate.elapsed_ms
        calc_text = f"{t.get('hr_calc', 'Calc')} {estimate.elapsed_ms:.3f} ms"
        if estimate.ready and estimate.bpm is not None:
            self._lbl_realtime_hr.setText(
                f"{t.get('realtime_hr', 'Realtime HR')}: {estimate.bpm:.1f} BPM | "
                f"{estimate.window_seconds:.0f}s | {calc_text}"
            )
            self._lbl_realtime_hr.setStyleSheet(
                f"color: {COLOR_GREEN}; font-size: 13px; font-weight: bold;"
            )
            return

        if estimate.status == "filling":
            status = f"{t.get('hr_filling', 'Filling')} {estimate.window_seconds:.0f}s"
            color = COLOR_TEXT_DIM
        else:
            status = t.get("hr_weak", "Weak signal")
            color = COLOR_ORANGE
        self._lbl_realtime_hr.setText(
            f"{t.get('realtime_hr', 'Realtime HR')}: -- ({status}) | {calc_text}"
        )
        self._lbl_realtime_hr.setStyleSheet(
            f"color: {color}; font-size: 13px; font-weight: bold;"
        )

    def _toggle_record(self, save_dir: Path = None) -> bool:
        """
        切换录制状态.
        Returns: True=正在录制, False=停止录制
        """
        if not self._is_recording:
            if save_dir is None:
                save_dir = Path.home() / "Desktop"
            save_dir.mkdir(parents=True, exist_ok=True)
            path = str(
                save_dir / f"raw_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            status_path = path.replace(".csv", "_status.csv")
            self._csv_file = open(path, "w", newline="", encoding="utf-8-sig")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(RAW_CSV_HEADER)
            self._status_csv_file = open(status_path, "w", newline="", encoding="utf-8-sig")
            self._status_csv_writer = csv.writer(self._status_csv_file)
            self._status_csv_writer.writerow(STATUS_CSV_HEADER)
            self._recording_start_time = time.time()
            self._recorded_sample_count = 0
            self._reset_quality_stats()
            self._is_recording = True
            self._flush_counter = 0
            return True
        else:
            self._stop_recording()
            return False

    def _stop_recording(self):
        """停止录制并关闭文件"""
        self._is_recording = False
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        if self._status_csv_file:
            self._status_csv_file.flush()
            self._status_csv_file.close()
            self._status_csv_file = None
            self._status_csv_writer = None
        self._recording_start_time = None
        self._recorded_sample_count = 0

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    # ── 清屏 ─────────────────────────────────────────────

    def _clear_screen(self):
        """清除所有显示数据和曲线"""
        for d in (
            self._data_Uc1, self._data_Uc2, self._data_Ut1, self._data_Ut2,
            self._data_Accx, self._data_Accy, self._data_Accz,
            self._data_Gyrox, self._data_Gyroy, self._data_Gyroz,
            self._data_ppg_g, self._data_ppg_r, self._data_ppg_ir,
        ):
            d.clear()

        self._packet_count = 0
        self._start_time = time.time()
        self._reset_quality_stats()

        # 停止录制
        if self._is_recording:
            self._stop_recording()

        # 清空曲线
        for curve in (
            self._curve_ppg_g, self._curve_ppg_r, self._curve_ppg_ir,
            self._curve_Ut1, self._curve_Ut2,
            self._curve_Uc1, self._curve_Uc2,
            self._curve_accx, self._curve_accy, self._curve_accz,
            self._curve_gyrox, self._curve_gyroy, self._curve_gyroz,
        ):
            curve.setData([])

        # 重置信息条 (含标定标签)
        t = TRANSLATIONS[self._lang]
        self._lbl_mode.setText(f"{t.get('mode', 'Mode')}: --")
        self._lbl_rate.setText(f"{t.get('sample_rate', 'Rate')}: RX 0 Hz | DEV 0 Hz")
        self._lbl_loss.setText(f"{t.get('packet_loss', 'Loss')}: 0.00% (0/0)")
        self._lbl_realtime_hr.setText(f"{t.get('realtime_hr', 'Realtime HR')}: --")
        self._lbl_realtime_hr.setStyleSheet(
            f"color: {COLOR_TEXT_DIM}; font-size: 13px; font-weight: bold;"
        )
        self._lbl_count.setText(f"{t.get('pkt_count', 'Packets')}: 0")
        self._lbl_calib.setVisible(False)

    # ── 语言切换 ─────────────────────────────────────────

    def _apply_language(self, lang: str):
        """应用语言到所有 UI 元素"""
        self._lang = lang
        t = TRANSLATIONS[lang]
        self._lbl_mode.setText(f"{t.get('mode', 'Mode')}: --")
        self._lbl_rate.setText(
            f"{t.get('sample_rate', 'Rate')}: "
            f"RX {self._last_rx_hz} Hz | DEV {self._last_device_hz} Hz"
        )
        self._lbl_loss.setText(
            f"{t.get('packet_loss', 'Loss')}: {self._quality.loss_rate * 100:.2f}% "
            f"({self._quality.missing_count}/{self._quality.expected_count})"
        )
        self._lbl_realtime_hr.setText(f"{t.get('realtime_hr', 'Realtime HR')}: --")
        self._lbl_count.setText(f"{t.get('pkt_count', 'Packets')}: 0")

        # 更新图表标题
        self._plot_ppg_g.setTitle(
            t.get("ppg_green", "PPG Green"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_ppg_r.setTitle(
            t.get("ppg_red", "PPG Red"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_ppg_ir.setTitle(
            t.get("ppg_ir", "PPG IR"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Ut1.setTitle(
            t.get("bridge_top", "Ut1 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Ut2.setTitle(
            t.get("bridge_top", "Ut2 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Uc1.setTitle(
            t.get("bridge_mid", "Uc1 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_Uc2.setTitle(
            t.get("bridge_mid", "Uc2 (mV)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_acc.setTitle(
            t.get("acceleration", "ACC (g)"), color=COLOR_TEXT_DIM, size="10pt"
        )
        self._plot_gyro.setTitle(
            "GYRO (dps)", color=COLOR_TEXT_DIM, size="10pt"
        )

    # ── 模拟模式 ─────────────────────────────────────────

    def start_simulation(self):
        """启动 100Hz 模拟数据"""
        self._sim_step = 0
        self._sim_timer.start(10)  # 100Hz

    def stop_simulation(self):
        """停止模拟"""
        self._sim_timer.stop()

    def _sim_tick(self):
        """生成一个模拟多光谱原始数据包"""
        self._sim_step += 1
        t = self._sim_step / 100.0

        # 合成 PPG 信号 (~1.2Hz 心率, 17-bit 量级)
        base = 50000
        pulse = 8000 * math.sin(2 * math.pi * 1.2 * t)
        noise = (hash(int(t * 1000)) % 1000 - 500)

        pkt = RawDataPacket(
            Ut2=1200.0 + 5 * math.sin(0.1 * t),
            Ut1=1198.0 + 5 * math.sin(0.1 * t + 0.5),
            Uc2=600.0 + 2 * math.sin(0.2 * t),
            Uc1=601.0 + 2 * math.cos(0.15 * t),
            acc_x=0.01 * math.sin(2 * t),
            acc_y=0.01 * math.cos(3 * t),
            acc_z=1.0 + 0.005 * math.sin(t),
            gyro_x=2.0 * math.sin(0.5 * t),
            gyro_y=-1.5 * math.cos(0.7 * t),
            gyro_z=0.8 * math.sin(0.3 * t + 1.0),
            ppg_green=int(base + pulse + noise) & 0x01FFFF,
            ppg_red=int(base * 0.8 + pulse * 0.6 + noise * 0.8) & 0x01FFFF,
            ppg_ir=int(base * 0.7 + pulse * 0.5 + noise * 0.7) & 0x01FFFF,
            sequence=self._sim_step & 0xFFFF,
        )
        self.handle_raw_data(pkt)
