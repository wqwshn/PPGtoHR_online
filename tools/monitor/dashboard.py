"""
PPG Monitor - 主窗口与心率面板

暗色主题实时监测仪表盘, 包含:
  - MonitorWindow: 顶层窗口, 管理工具栏/面板切换/状态栏
  - HRPanel: 在线心率展示面板
  - StatusDot: 连接状态指示灯

设计系统 (UI/UX Pro Max):
  风格: Dark Mode + Real-Time Monitoring
  配色: Primary #0891B2, Accent #059669
  背景: #0F1923, 卡片: #1A2332
"""

import csv
import os
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QTimer, pyqtProperty, QSize
from PyQt5.QtGui import QFont, QColor
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QProgressBar, QStatusBar,
    QFrame, QFileDialog, QStackedWidget, QButtonGroup,
)
import pyqtgraph as pg

from protocol import HRPacket


# ── 配色常量 (供 raw_data_panel 复用) ─────────────────────
COLOR_BG = "#0F1923"
COLOR_CARD = "#1A2332"
COLOR_CARD_BORDER = "#2A3A4E"
COLOR_PRIMARY = "#0891B2"
COLOR_TEXT = "#E2E8F0"
COLOR_TEXT_DIM = "#8899AA"
COLOR_GREEN = "#22C55E"
COLOR_ORANGE = "#F97316"
COLOR_RED = "#EF4444"
COLOR_BLUE = "#3B82F6"

# 心率颜色阈值
HR_COLOR_LOW = COLOR_BLUE        # < 60 BPM
HR_COLOR_NORMAL = COLOR_GREEN    # 60-100 BPM
HR_COLOR_ELEVATED = COLOR_ORANGE # 100-140 BPM
HR_COLOR_HIGH = COLOR_RED        # > 140 BPM

TREND_SECONDS = 60   # 趋势图显示最近 60 秒

# ── 国际化翻译表 ──────────────────────────────────────────
TRANSLATIONS = {
    "zh": {
        "window_title": "PPG Monitor",
        "connect": "连接",
        "disconnect": "断开",
        "refresh": "刷新",
        "clear": "清屏",
        "record": "录制",
        "stop": "停止",
        "save": "保存",
        "select_save_path": "选择保存路径",
        "recording_to": "录制中 ->",
        "record_saved": "录制已保存",
        "record_cancelled": "录制已取消",
        "record_pts": "录制",
        "lang": "EN",
        "disconnected": "未连接",
        "connected": "已连接",
        "simulated": "模拟中",
        "no_ports": "未发现串口",
        "algorithm_paths": "算法路径",
        "ppg_signal": "PPG 信号:",
        "calibration": "校准进度:",
        "window": "数据窗口:",
        "sampling_rate": "采样率:",
        "calibrated": "已校准",
        "pending": "待校准",
        "ready": "就绪",
        "filling": "填充中...",
        "rest": "静息",
        "motion": "运动",
        "pkt_count": "数据包",
        "uptime": "运行时间",
        "save_success": "数据已保存",
        "save_fail": "保存失败",
        "clear_confirm": "已清除所有数据",
        # 面板切换
        "panel_hr": "在线心率",
        "panel_raw": "原始数据",
        # 原始数据面板
        "mode": "模式",
        "hr_mode": "心率模式",
        "spo2_mode": "血氧模式",
        "sample_rate": "采样率",
        "packet_loss": "丢包率",
        "ppg_green": "PPG 绿光",
        "ppg_red": "PPG 红光",
        "ppg_ir": "PPG 红外",
        "bridge_top": "桥顶电压",
        "bridge_mid": "桥中电压",
        "acceleration": "加速度",
        "temperature": "温度",
    },
    "en": {
        "window_title": "PPG Monitor",
        "connect": "Connect",
        "disconnect": "Disconnect",
        "refresh": "Refresh",
        "clear": "Clear",
        "record": "Record",
        "stop": "Stop",
        "save": "Save",
        "select_save_path": "Select Save Path",
        "recording_to": "Recording ->",
        "record_saved": "Recording saved",
        "record_cancelled": "Recording cancelled",
        "record_pts": "Rec",
        "lang": "中文",
        "disconnected": "Disconnected",
        "connected": "Connected",
        "simulated": "Simulated",
        "no_ports": "No ports found",
        "algorithm_paths": "Algorithm Paths",
        "ppg_signal": "PPG Signal:",
        "calibration": "Calibration:",
        "window": "Window:",
        "sampling_rate": "Sample Rate:",
        "calibrated": "Calibrated",
        "pending": "Pending",
        "ready": "Ready",
        "filling": "Filling...",
        "rest": "REST",
        "motion": "MOTION",
        "pkt_count": "Packets",
        "uptime": "Uptime",
        "save_success": "Data saved",
        "save_fail": "Save failed",
        "clear_confirm": "All data cleared",
        # 面板切换
        "panel_hr": "Online HR",
        "panel_raw": "Raw Data",
        # 原始数据面板
        "mode": "Mode",
        "hr_mode": "HR Mode",
        "spo2_mode": "SpO2 Mode",
        "sample_rate": "Rate",
        "packet_loss": "Loss",
        "ppg_green": "PPG Green",
        "ppg_red": "PPG Red",
        "ppg_ir": "PPG IR",
        "bridge_top": "Bridge Top",
        "bridge_mid": "Bridge Mid",
        "acceleration": "Acceleration",
        "temperature": "Temperature",
    },
}

DARK_QSS = f"""
QMainWindow, QWidget {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
}}
QFrame#card {{
    background-color: {COLOR_CARD};
    border: 1px solid {COLOR_CARD_BORDER};
    border-radius: 12px;
}}
QLabel {{
    color: {COLOR_TEXT};
}}
QPushButton {{
    background-color: {COLOR_PRIMARY};
    color: white;
    border: none;
    border-radius: 6px;
    padding: 6px 16px;
    font-size: 13px;
    font-weight: bold;
}}
QPushButton:hover {{
    background-color: #0EA5C9;
}}
QPushButton:disabled {{
    background-color: #2A3A4E;
    color: #556677;
}}
QPushButton#btn_disconnect {{
    background-color: {COLOR_RED};
}}
QPushButton#btn_disconnect:hover {{
    background-color: #DC2626;
}}
QComboBox {{
    background-color: {COLOR_CARD};
    color: {COLOR_TEXT};
    border: 1px solid {COLOR_CARD_BORDER};
    border-radius: 6px;
    padding: 6px 12px;
    min-width: 120px;
}}
QComboBox::drop-down {{
    border: none;
}}
QComboBox QAbstractItemView {{
    background-color: {COLOR_CARD};
    color: {COLOR_TEXT};
    selection-background-color: {COLOR_PRIMARY};
}}
QStatusBar {{
    background-color: #0A1520;
    color: {COLOR_TEXT_DIM};
    font-size: 12px;
    border-top: 1px solid {COLOR_CARD_BORDER};
}}
QProgressBar {{
    background-color: #1A2332;
    border: none;
    border-radius: 4px;
    text-align: center;
    color: {COLOR_TEXT};
    font-size: 11px;
    min-height: 18px;
    max-height: 18px;
}}
QProgressBar::chunk {{
    background-color: {COLOR_PRIMARY};
    border-radius: 4px;
}}
QPushButton#btn_panel_switch {{
    background-color: #1A2332;
    color: #8899AA;
    border: 1px solid #2A3A4E;
    border-radius: 6px;
    padding: 6px 20px;
    font-size: 13px;
    font-weight: bold;
    min-width: 70px;
}}
QPushButton#btn_panel_switch:checked {{
    background-color: {COLOR_PRIMARY};
    color: white;
    border: 1px solid {COLOR_PRIMARY};
}}
QPushButton#btn_panel_switch:hover {{
    background-color: #0EA5C9;
}}
"""


def hr_color(bpm: float) -> str:
    """根据 BPM 返回对应颜色"""
    if bpm < 60:
        return HR_COLOR_LOW
    elif bpm <= 100:
        return HR_COLOR_NORMAL
    elif bpm <= 140:
        return HR_COLOR_ELEVATED
    else:
        return HR_COLOR_HIGH


class StatusDot(QLabel):
    """带脉冲动画的连接状态指示灯"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(14, 14)
        self._opacity = 1.0
        self._connected = False
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._pulse)
        self._phase = 0
        self._update_style()

    def set_connected(self, connected: bool):
        self._connected = connected
        if connected:
            self._timer.start(1000)
        else:
            self._timer.stop()
            self._opacity = 1.0
        self._update_style()

    def _pulse(self):
        self._phase = (self._phase + 1) % 2
        self._opacity = 0.5 if self._phase else 1.0
        self._update_style()

    def _update_style(self):
        color = COLOR_GREEN if self._connected else COLOR_RED
        self.setStyleSheet(
            f"background-color: {color}; border-radius: 7px; "
            f"opacity: {int(self._opacity * 255)};"
        )


# ── 顶层窗口 ──────────────────────────────────────────────

class MonitorWindow(QMainWindow):
    """统一监测主窗口: 管理工具栏、面板切换和状态栏"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lang = "zh"
        self.setWindowTitle(TRANSLATIONS[self._lang]["window_title"])
        self.setMinimumSize(1000, 700)
        self.resize(1200, 750)

        # 默认保存目录
        self._save_dir = Path.home() / "Desktop"
        if not self._save_dir.exists():
            self._save_dir = Path.home()

        # 创建子面板
        self._hr_panel = HRPanel(self)
        self._raw_panel = self._create_raw_panel()

        # 面板堆叠
        self._panel_stack = QStackedWidget()
        self._panel_stack.addWidget(self._hr_panel)     # index 0
        self._panel_stack.addWidget(self._raw_panel)    # index 1

        # 主布局
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(12, 8, 12, 8)
        root.addWidget(self._build_toolbar())
        root.addWidget(self._panel_stack, 1)

        # 状态栏
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_label = QLabel("Disconnected")
        self._status_bar.addPermanentWidget(self._status_label, 1)

        self.setStyleSheet(DARK_QSS)

        # 模拟定时器 (HR 面板)
        self._sim_timer = QTimer(self)
        self._sim_timer.timeout.connect(self._hr_panel._sim_tick)
        self._hr_sim_running = False

    def _create_raw_panel(self):
        """延迟导入避免循环依赖"""
        from raw_data_panel import RawDataPanel
        return RawDataPanel(self)

    # ── 工具栏 ───────────────────────────────────────────

    def _build_toolbar(self) -> QFrame:
        frame = QFrame()
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(0, 4, 0, 4)

        t = TRANSLATIONS[self._lang]

        # 连接状态灯
        self._status_dot = StatusDot()
        layout.addWidget(self._status_dot)

        # 串口选择
        self._combo_port = QComboBox()
        layout.addWidget(self._combo_port)

        self._btn_connect = QPushButton(t["connect"])
        layout.addWidget(self._btn_connect)

        self._btn_disconnect = QPushButton(t["disconnect"])
        self._btn_disconnect.setObjectName("btn_disconnect")
        self._btn_disconnect.setEnabled(False)
        layout.addWidget(self._btn_disconnect)

        self._btn_refresh = QPushButton(t["refresh"])
        self._btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(self._btn_refresh)

        # 分隔线
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.VLine)
        sep1.setStyleSheet(f"color: {COLOR_CARD_BORDER};")
        layout.addWidget(sep1)

        # 面板切换按钮
        self._btn_hr_panel = QPushButton(t["panel_hr"])
        self._btn_hr_panel.setCheckable(True)
        self._btn_hr_panel.setChecked(True)
        self._btn_hr_panel.setObjectName("btn_panel_switch")

        self._btn_raw_panel = QPushButton(t["panel_raw"])
        self._btn_raw_panel.setCheckable(True)
        self._btn_raw_panel.setObjectName("btn_panel_switch")

        self._panel_group = QButtonGroup(self)
        self._panel_group.addButton(self._btn_hr_panel, 0)
        self._panel_group.addButton(self._btn_raw_panel, 1)
        self._panel_group.buttonClicked[int].connect(self._switch_panel)

        layout.addWidget(self._btn_hr_panel)
        layout.addWidget(self._btn_raw_panel)

        # 分隔线
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.VLine)
        sep2.setStyleSheet(f"color: {COLOR_CARD_BORDER};")
        layout.addWidget(sep2)

        # 清屏
        self._btn_clear = QPushButton(t["clear"])
        self._btn_clear.setObjectName("btn_clear")
        self._btn_clear.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_ORANGE};
                color: white; border: none; border-radius: 6px;
                padding: 6px 16px; font-size: 13px; font-weight: bold;
            }}
            QPushButton:hover {{ background-color: #EA7B1A; }}
        """)
        self._btn_clear.clicked.connect(self._clear_active_panel)
        layout.addWidget(self._btn_clear)

        # 保存路径
        self._btn_save_path = QPushButton(self._save_dir.name)
        self._btn_save_path.setFixedWidth(80)
        self._btn_save_path.setToolTip(str(self._save_dir))
        self._btn_save_path.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_CARD}; color: {COLOR_TEXT_DIM};
                border: 1px solid {COLOR_CARD_BORDER}; border-radius: 4px;
                padding: 6px 8px; font-size: 11px;
            }}
            QPushButton:hover {{ background-color: #2A3A4E; }}
        """)
        self._btn_save_path.clicked.connect(self._browse_save_dir)
        layout.addWidget(self._btn_save_path)

        # 录制
        self._btn_record = QPushButton(t["record"])
        self._btn_record.setObjectName("btn_record")
        self._btn_record.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_RED};
                color: white; border: none; border-radius: 6px;
                padding: 6px 16px; font-size: 13px; font-weight: bold;
            }}
            QPushButton:hover {{ background-color: #DC2626; }}
        """)
        self._btn_record.clicked.connect(self._toggle_record_active)
        layout.addWidget(self._btn_record)

        layout.addStretch()

        # 语言切换
        self._btn_lang = QPushButton(t["lang"])
        self._btn_lang.setFixedWidth(60)
        self._btn_lang.setStyleSheet(f"""
            QPushButton {{
                background-color: #4B5563; color: white; border: none;
                border-radius: 6px; padding: 6px 10px;
                font-size: 12px; font-weight: bold;
            }}
            QPushButton:hover {{ background-color: #6B7280; }}
        """)
        self._btn_lang.clicked.connect(self._toggle_language)
        layout.addWidget(self._btn_lang)

        # 状态文字
        self._conn_label = QLabel(t["disconnected"])
        self._conn_label.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 12px;")
        layout.addWidget(self._conn_label)

        return frame

    # ── 面板切换 ─────────────────────────────────────────

    def _switch_panel(self, index: int):
        self._panel_stack.setCurrentIndex(index)
        self._update_record_button()

    def _update_record_button(self):
        """根据当前活动面板更新录制按钮外观"""
        t = TRANSLATIONS[self._lang]
        idx = self._panel_stack.currentIndex()
        is_rec = self._hr_panel.is_recording if idx == 0 else self._raw_panel.is_recording

        if is_rec:
            self._btn_record.setText(t["stop"])
            self._btn_record.setStyleSheet(f"""
                QPushButton {{
                    background-color: {COLOR_RED}; color: white;
                    border: 2px solid #FF0000; border-radius: 6px;
                    padding: 6px 16px; font-size: 13px; font-weight: bold;
                }}
                QPushButton:hover {{ background-color: #B91C1C; }}
            """)
        else:
            self._btn_record.setText(t["record"])
            self._btn_record.setStyleSheet(f"""
                QPushButton {{
                    background-color: {COLOR_RED}; color: white;
                    border: none; border-radius: 6px;
                    padding: 6px 16px; font-size: 13px; font-weight: bold;
                }}
                QPushButton:hover {{ background-color: #DC2626; }}
            """)

    def _browse_save_dir(self):
        """浏览选择保存目录"""
        t = TRANSLATIONS[self._lang]
        path = QFileDialog.getExistingDirectory(
            self, t["select_save_path"], str(self._save_dir)
        )
        if path:
            self._save_dir = Path(path)
            self._btn_save_path.setText(self._save_dir.name)
            self._btn_save_path.setToolTip(str(self._save_dir))

    # ── 公共接口 ─────────────────────────────────────────

    def refresh_ports(self):
        """刷新串口列表"""
        t = TRANSLATIONS[self._lang]
        import serial.tools.list_ports
        self._combo_port.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self._combo_port.addItems(ports)
        else:
            self._combo_port.addItem(t["no_ports"])

    def set_connected(self, connected: bool):
        """更新连接状态 UI"""
        t = TRANSLATIONS[self._lang]
        self._status_dot.set_connected(connected)
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)
        self._combo_port.setEnabled(not connected)
        self._conn_label.setText(t["connected"] if connected else t["disconnected"])
        self._conn_label.setStyleSheet(
            f"color: {COLOR_GREEN if connected else COLOR_TEXT_DIM}; font-size: 12px;"
        )

    def show_error(self, msg: str):
        self._conn_label.setText(msg)
        self._conn_label.setStyleSheet(f"color: {COLOR_RED}; font-size: 12px;")

    def update_status(self, text: str):
        self._status_label.setText(text)

    # ── 委托操作 ─────────────────────────────────────────

    def _clear_active_panel(self):
        idx = self._panel_stack.currentIndex()
        if idx == 0:
            self._hr_panel._clear_screen()
        else:
            self._raw_panel._clear_screen()
        t = TRANSLATIONS[self._lang]
        self._status_label.setText(t["clear_confirm"])

    def _toggle_record_active(self):
        idx = self._panel_stack.currentIndex()
        if idx == 0:
            self._hr_panel._toggle_record(self._save_dir)
        else:
            self._raw_panel._toggle_record(self._save_dir)
        self._update_record_button()

    def _toggle_language(self):
        self._lang = "en" if self._lang == "zh" else "zh"
        self._apply_language()

    def _apply_language(self):
        t = TRANSLATIONS[self._lang]
        self.setWindowTitle(t["window_title"])
        self._btn_connect.setText(t["connect"])
        self._btn_disconnect.setText(t["disconnect"])
        self._btn_refresh.setText(t["refresh"])
        self._btn_clear.setText(t["clear"])
        self._btn_lang.setText(t["lang"])
        self._btn_hr_panel.setText(t["panel_hr"])
        self._btn_raw_panel.setText(t["panel_raw"])
        self._update_record_button()
        self._hr_panel._apply_language(self._lang)
        self._raw_panel._apply_language(self._lang)
        self._status_label.setText(t["disconnected"])

    # ── 模拟模式 ─────────────────────────────────────────

    def start_hr_simulation(self):
        self._hr_panel._sim_step = 0
        self._sim_timer.start(1000)
        self._hr_sim_running = True
        self.set_connected(True)
        t = TRANSLATIONS[self._lang]
        self._conn_label.setText(t["simulated"])

    def start_raw_simulation(self):
        self._raw_panel.start_simulation()
        self.set_connected(True)
        t = TRANSLATIONS[self._lang]
        self._conn_label.setText(t["simulated"])
        self._btn_raw_panel.setChecked(True)
        self._switch_panel(1)

    def stop_simulations(self):
        self._sim_timer.stop()
        self._raw_panel.stop_simulation()


# ── 在线心率面板 ──────────────────────────────────────────

class HRPanel(QWidget):
    """在线心率展示面板"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lang = "zh"

        # 数据存储
        self._trend_bpm = deque(maxlen=TREND_SECONDS)
        self._trend_time = deque(maxlen=TREND_SECONDS)
        self._start_time = time.time()
        self._pkt_count = 0

        # 数据记录缓冲区
        self._recorded_data: list[dict] = []
        self._recording_start: Optional[float] = None
        self._last_device_ts: int = -1

        # 录制状态
        self._is_recording = False
        self._record_file_path: Optional[str] = None

        self._init_ui()
        self._init_plot()
        self._sim_step = 0

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    # ── UI 构建 ──────────────────────────────────────────

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(0, 0, 0, 0)

        top_row = QHBoxLayout()
        top_row.setSpacing(12)
        top_row.addWidget(self._build_hr_card(), 2)
        top_row.addWidget(self._build_paths_card(), 3)
        layout.addLayout(top_row, 3)

        layout.addWidget(self._build_trend_card(), 4)

    def _build_hr_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setAlignment(Qt.AlignCenter)

        self._lbl_bpm = QLabel("--.-")
        self._lbl_bpm.setAlignment(Qt.AlignCenter)
        font = QFont("Segoe UI", 72, QFont.Bold)
        self._lbl_bpm.setFont(font)
        self._lbl_bpm.setStyleSheet(f"color: {COLOR_TEXT_DIM};")
        layout.addWidget(self._lbl_bpm)

        lbl_unit = QLabel("BPM")
        lbl_unit.setAlignment(Qt.AlignCenter)
        lbl_unit.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 16px; font-weight: bold;")
        layout.addWidget(lbl_unit)

        self._lbl_motion = QLabel("REST")
        self._lbl_motion.setAlignment(Qt.AlignCenter)
        self._lbl_motion.setStyleSheet(
            f"color: {COLOR_GREEN}; font-size: 18px; font-weight: bold; "
            f"background-color: #0D2818; border-radius: 8px; padding: 4px 24px;"
        )
        layout.addWidget(self._lbl_motion)

        return card

    def _build_paths_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 12, 16, 12)

        t = TRANSLATIONS[self._lang]
        self._paths_card_title = QLabel(t["algorithm_paths"])
        self._paths_card_title.setStyleSheet(
            f"color: {COLOR_TEXT_DIM}; font-size: 13px; font-weight: bold;"
        )
        layout.addWidget(self._paths_card_title)

        self._path_bars = {}
        paths = [
            ("LMS-HF", COLOR_PRIMARY),
            ("LMS-ACC", "#8B5CF6"),
            ("FFT", COLOR_ORANGE),
        ]
        for name, color in paths:
            row = QHBoxLayout()
            lbl_name = QLabel(f"{name}:")
            lbl_name.setFixedWidth(70)
            lbl_name.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 13px;")
            row.addWidget(lbl_name)

            bar = QProgressBar()
            bar.setRange(0, 2000)
            bar.setValue(0)
            bar.setFormat("--.- BPM")
            bar.setStyleSheet(f"""
                QProgressBar::chunk {{
                    background-color: {color}; border-radius: 4px;
                }}
            """)
            row.addWidget(bar)
            self._path_bars[name] = bar
            layout.addLayout(row)

        layout.addSpacing(12)

        info_grid = QGridLayout()
        info_grid.setSpacing(6)

        info_items = [
            (t["ppg_signal"], "_lbl_ppg", "--"),
            (t["calibration"], "_lbl_calib", "-- / 8"),
            (t["window"], "_lbl_window", "--"),
            (t["sampling_rate"], "_lbl_sample_rate", "-- Hz"),
            ("HF1 AC:", "_lbl_hf1_ac", "--"),
            ("HF1 corr:", "_lbl_hf1_corr", "--"),
            ("ACC corr:", "_lbl_acc_corr", "--"),
            ("HF2 AC:", "_lbl_hf2_ac", "--"),
            ("HF2 corr:", "_lbl_hf2_corr", "--"),
        ]
        self._info_name_labels = []
        for row_idx, (label_text, attr_name, default) in enumerate(info_items):
            lbl = QLabel(label_text)
            lbl.setStyleSheet(f"color: {COLOR_TEXT_DIM}; font-size: 12px;")
            info_grid.addWidget(lbl, row_idx, 0)
            self._info_name_labels.append(lbl)
            val = QLabel(default)
            val.setStyleSheet(f"color: {COLOR_TEXT}; font-size: 12px; font-weight: bold;")
            info_grid.addWidget(val, row_idx, 1)
            setattr(self, attr_name, val)

        layout.addLayout(info_grid)
        layout.addStretch()
        return card

    def _build_trend_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)

        pg.setConfigOptions(
            antialias=True,
            background=COLOR_CARD,
            foreground=COLOR_TEXT_DIM,
        )
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setLabel("left", "Heart Rate", units="BPM")
        self._plot_widget.setLabel("bottom", "Time", units="s")
        self._plot_widget.setYRange(40, 160)
        self._plot_widget.showGrid(x=True, y=True, alpha=0.15)
        self._plot_widget.getAxis("left").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._plot_widget.getAxis("bottom").setPen(pg.mkPen(COLOR_TEXT_DIM))
        self._plot_widget.setBackground(COLOR_CARD)

        self._curve = self._plot_widget.plot(
            pen=pg.mkPen(COLOR_PRIMARY, width=2.5),
            symbol=None,
        )

        normal_region = pg.LinearRegionItem(
            values=[60, 100], movable=False,
            brush=pg.mkBrush(34, 197, 94, 25),
        )
        normal_region.lines[0].setPen(pg.mkPen(COLOR_GREEN, style=Qt.DashLine, width=1))
        normal_region.lines[1].setPen(pg.mkPen(COLOR_GREEN, style=Qt.DashLine, width=1))
        self._plot_widget.addItem(normal_region)

        layout.addWidget(self._plot_widget)
        return card

    def _init_plot(self):
        pass

    # ── 数据更新 ─────────────────────────────────────────

    def update_data(self, pkt: HRPacket):
        """接收并展示一个心率数据包"""
        t = TRANSLATIONS[self._lang]

        if pkt.timestamp == self._last_device_ts:
            return
        self._last_device_ts = pkt.timestamp

        self._pkt_count += 1
        elapsed = time.time() - self._start_time

        if self._is_recording:
            if self._recording_start is None:
                self._recording_start = elapsed
            self._recorded_data.append({
                "time": round(elapsed - self._recording_start, 3),
                "fused_bpm": pkt.fused_bpm,
                "is_motion": int(pkt.is_motion),
                "win_filled": int(pkt.win_filled),
                "hr_lms_hf": pkt.hr_lms_hf, "hr_lms_acc": pkt.hr_lms_acc,
                "hr_fft": pkt.hr_fft, "ppg_mean": pkt.ppg_mean,
                "motion_calibrated": int(pkt.motion_calibrated),
                "timestamp": pkt.timestamp, "calib_progress": pkt.calib_progress,
                "sampling_rate": pkt.sampling_rate, "hf1_ac_mv": pkt.hf1_ac_mv,
                "hf1_ppg_corr": pkt.hf1_ppg_corr, "acc_ppg_corr": pkt.acc_ppg_corr,
                "hf2_ac_mv": pkt.hf2_ac_mv, "hf2_ppg_corr": pkt.hf2_ppg_corr,
            })

        bpm = pkt.fused_bpm
        self._lbl_bpm.setText(f"{bpm:.1f}")
        self._lbl_bpm.setStyleSheet(f"color: {hr_color(bpm)};")

        if pkt.is_motion:
            self._lbl_motion.setText(t["motion"])
            self._lbl_motion.setStyleSheet(
                f"color: {COLOR_ORANGE}; font-size: 18px; font-weight: bold; "
                f"background-color: #2D1B0E; border-radius: 8px; padding: 4px 24px;"
            )
        else:
            self._lbl_motion.setText(t["rest"])
            self._lbl_motion.setStyleSheet(
                f"color: {COLOR_GREEN}; font-size: 18px; font-weight: bold; "
                f"background-color: #0D2818; border-radius: 8px; padding: 4px 24px;"
            )

        for name, val in [("LMS-HF", pkt.hr_lms_hf), ("LMS-ACC", pkt.hr_lms_acc), ("FFT", pkt.hr_fft)]:
            bar = self._path_bars[name]
            bar.setValue(int(val * 10))
            bar.setFormat(f"{val:.1f} BPM")

        self._lbl_ppg.setText(str(pkt.ppg_mean))
        self._lbl_calib.setText(
            f"{pkt.calib_progress} / 8  ({t['calibrated'] if pkt.motion_calibrated else t['pending']})"
        )
        self._lbl_window.setText(t["ready"] if pkt.win_filled else t["filling"])
        self._lbl_sample_rate.setText(f"{pkt.sampling_rate} Hz")
        self._lbl_hf1_ac.setText(f"{pkt.hf1_ac_mv:.2f} mV")
        self._lbl_hf1_corr.setText(f"{pkt.hf1_ppg_corr:.3f}")
        self._lbl_acc_corr.setText(f"{pkt.acc_ppg_corr:.3f}")
        self._lbl_hf2_ac.setText(f"{pkt.hf2_ac_mv:.2f} mV")
        self._lbl_hf2_corr.setText(f"{pkt.hf2_ppg_corr:.3f}")

        self._trend_bpm.append(bpm)
        self._trend_time.append(elapsed)
        self._curve.setData(list(self._trend_time), list(self._trend_bpm))
        if self._trend_time:
            t_max = self._trend_time[-1]
            t_min = max(0, t_max - TREND_SECONDS)
            self._plot_widget.setXRange(t_min, max(t_max, t_min + 10))
            bpm_list = list(self._trend_bpm)
            if bpm_list:
                y_min = min(max(0, min(bpm_list) - 10), 40)
                y_max = max(min(250, max(bpm_list) + 10), 160)
                self._plot_widget.setYRange(y_min, y_max)

    # ── 清屏 ─────────────────────────────────────────────

    def _clear_screen(self):
        self._trend_bpm.clear()
        self._trend_time.clear()
        self._pkt_count = 0
        self._start_time = time.time()
        self._recorded_data.clear()
        self._recording_start = None
        self._is_recording = False
        self._record_file_path = None
        self._last_device_ts = -1

        self._curve.setData([], [])
        self._lbl_bpm.setText("--.-")
        self._lbl_bpm.setStyleSheet(f"color: {COLOR_TEXT_DIM};")

        t = TRANSLATIONS[self._lang]
        self._lbl_motion.setText(t["rest"])
        self._lbl_motion.setStyleSheet(
            f"color: {COLOR_GREEN}; font-size: 18px; font-weight: bold; "
            f"background-color: #0D2818; border-radius: 8px; padding: 4px 24px;"
        )

        for name in ("LMS-HF", "LMS-ACC", "FFT"):
            bar = self._path_bars[name]
            bar.setValue(0)
            bar.setFormat("--.- BPM")

        self._lbl_ppg.setText("--")
        self._lbl_calib.setText("-- / 8")
        self._lbl_window.setText("--")
        self._lbl_sample_rate.setText("-- Hz")
        self._lbl_hf1_ac.setText("--")
        self._lbl_hf1_corr.setText("--")
        self._lbl_acc_corr.setText("--")
        self._lbl_hf2_ac.setText("--")
        self._lbl_hf2_corr.setText("--")

        self._plot_widget.setXRange(0, 10)
        self._plot_widget.setYRange(40, 160)

    # ── 录制 ─────────────────────────────────────────────

    def _toggle_record(self, save_dir: Path = None):
        t = TRANSLATIONS[self._lang]
        if not self._is_recording:
            if save_dir is None:
                save_dir = Path.home() / "Desktop"
            save_dir.mkdir(parents=True, exist_ok=True)
            self._record_file_path = str(
                save_dir / f"hr_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            self._recorded_data.clear()
            self._recording_start = None
            self._is_recording = True
        else:
            self._is_recording = False
            if self._recorded_data and self._record_file_path:
                try:
                    self._write_csv(self._record_file_path)
                except Exception:
                    pass
            self._record_file_path = None

    def _write_csv(self, path: str):
        fieldnames = [
            "time", "fused_bpm", "is_motion", "win_filled",
            "hr_lms_hf", "hr_lms_acc", "hr_fft",
            "ppg_mean", "motion_calibrated", "timestamp", "calib_progress",
            "sampling_rate", "hf1_ac_mv", "hf1_ppg_corr", "acc_ppg_corr",
            "hf2_ac_mv", "hf2_ppg_corr",
        ]
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._recorded_data)

    # ── 语言切换 ─────────────────────────────────────────

    def _apply_language(self, lang: str):
        self._lang = lang
        t = TRANSLATIONS[lang]
        self._paths_card_title.setText(t["algorithm_paths"])
        info_keys = ["ppg_signal", "calibration", "window", "sampling_rate"]
        for lbl, key in zip(self._info_name_labels, info_keys):
            lbl.setText(t[key])

    # ── 模拟模式 ─────────────────────────────────────────

    def _sim_tick(self):
        """生成一个模拟心率包"""
        self._sim_step += 1
        t = self._sim_step

        import math
        base = 72 + 8 * math.sin(t * 0.05)
        noise = (hash(t * 7) % 20 - 10) / 10.0
        bpm = base + noise
        is_motion = 30 < (t % 60) < 45

        pkt = HRPacket(
            fused_bpm=round(bpm, 1), is_motion=is_motion,
            win_filled=t >= 8, hr_lms_hf=round(bpm + 1.2, 1),
            hr_lms_acc=round(bpm - 0.8, 1), hr_fft=round(bpm + 0.3, 1),
            ppg_mean=85000 + (hash(t) % 5000), motion_calibrated=t >= 8,
            timestamp=t, calib_progress=min(t, 8), sampling_rate=100,
            hf1_ac_mv=round(0.5 + (hash(t * 7) % 100) / 100.0, 2),
            hf1_ppg_corr=round(0.3 + (hash(t * 13) % 50) / 100.0, 3),
            acc_ppg_corr=round(0.2 + (hash(t * 17) % 40) / 100.0, 3),
            hf2_ac_mv=round(0.4 + (hash(t * 11) % 80) / 100.0, 2),
            hf2_ppg_corr=round(0.25 + (hash(t * 19) % 45) / 100.0, 3),
        )
        self.update_data(pkt)
