"""
PPG Monitor - 程序入口

用法:
  实际串口模式:     python main.py
  HR 模拟数据模式:  python main.py --simulate
  原始数据模拟模式: python main.py --raw-simulate
"""
from __future__ import annotations

import sys
import argparse
from typing import Optional

from PyQt5.QtWidgets import QApplication

from dashboard import MonitorWindow
from serial_reader import SerialReader


class AppController:
    """连接 MonitorWindow 和 SerialReader 的控制器"""

    def __init__(self, window: MonitorWindow):
        self._win = window
        self._reader: Optional[SerialReader] = None

        self._win._btn_connect.clicked.connect(self._connect)
        self._win._btn_disconnect.clicked.connect(self._disconnect)

    def _connect(self):
        port = self._win._combo_port.currentText()
        if not port or "No ports" in port:
            self._win.show_error("No valid port selected")
            return

        self._reader = SerialReader(port, baudrate=115200)
        # 双协议信号连接
        self._reader.hr_packet_received.connect(self._win._hr_panel.update_data)
        self._reader.raw_packet_received.connect(self._win._raw_panel.handle_raw_data)
        self._reader.calib_status_received.connect(self._win._raw_panel.handle_calib_status)
        self._reader.error_occurred.connect(self._on_error)
        self._reader.connection_changed.connect(self._win.set_connected)
        self._reader.start()

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._win.set_connected(False)

    def _on_error(self, msg: str):
        self._win.show_error(msg)

    def cleanup(self):
        self._disconnect()
        self._win.stop_simulations()


def main():
    parser = argparse.ArgumentParser(description="PPG Monitor")
    parser.add_argument(
        "--simulate", action="store_true",
        help="Run with HR simulated data (1Hz)",
    )
    parser.add_argument(
        "--raw-simulate", action="store_true",
        help="Run with raw sensor simulated data (100Hz)",
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = MonitorWindow()
    win.refresh_ports()
    win.show()

    if args.simulate:
        win.start_hr_simulation()
    elif args.raw_simulate:
        win.start_raw_simulation()
    else:
        ctrl = AppController(win)
        app.aboutToQuit.connect(ctrl.cleanup)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
