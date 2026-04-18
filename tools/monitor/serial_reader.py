"""
PPG Monitor - 串口读取线程

在独立 QThread 中运行串口读取, 使用状态机逐字节解析帧.
支持双协议:
  - 31 字节心率结果包 (0xAA 0xCC) -> hr_packet_received
  - 33 字节多光谱原始传感器包 (0xAA 0xBB) -> raw_packet_received
"""
from __future__ import annotations

import serial
import serial.tools.list_ports
from typing import List, Optional
from PyQt5.QtCore import QThread, pyqtSignal

from protocol import (
    HEADER_BYTE_0, HEADER_BYTE_1, PACKET_LEN,
    RAW_HEADER_BYTE_1, RAW_PACKET_LEN,
    parse_hr_packet, HRPacket,
    parse_raw_packet, RawDataPacket,
)


class SerialReader(QThread):
    """串口读取与双协议帧解析线程"""

    # 信号: 心率结果包 (1Hz)
    hr_packet_received = pyqtSignal(HRPacket)
    # 信号: 原始传感器包 (100Hz)
    raw_packet_received = pyqtSignal(RawDataPacket)
    # 信号: 陀螺仪标定状态文本
    calib_status_received = pyqtSignal(str)
    # 信号: 错误信息
    error_occurred = pyqtSignal(str)
    # 信号: 连接状态变化
    connection_changed = pyqtSignal(bool)

    def __init__(self, port: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baudrate = baudrate
        self._running = False
        self._serial: Optional[serial.Serial] = None
        # 原始包统计 (用于丢包率计算)
        self._raw_total = 0
        self._raw_invalid = 0

    def run(self):
        """线程主循环: 打开串口 -> 逐字节状态机解析双协议帧"""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            self._running = True
            self._raw_total = 0
            self._raw_invalid = 0
            self.connection_changed.emit(True)
        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to open {self._port}: {e}")
            self.connection_changed.emit(False)
            return

        # 状态机: 0=等待帧头0(0xAA), 1=等待帧头1区分协议, 2=收集 payload
        state = 0
        buf = bytearray()
        expected_len = 0
        # ASCII 文本行缓冲 (用于标定状态等调试文本)
        text_buf = bytearray()

        try:
            while self._running:
                # 非阻塞读取, 增大缓冲区以适应 100Hz 吞吐
                raw = self._serial.read(4096)
                if not raw:
                    continue

                for byte in raw:
                    if state == 0:
                        if byte == HEADER_BYTE_0:  # 0xAA
                            buf = bytearray([byte])
                            state = 1
                        elif 0x20 <= byte <= 0x7E or byte in (0x0D, 0x0A):
                            # 可打印 ASCII 或换行: 收集文本行
                            if byte == 0x0A:
                                line = text_buf.decode("ascii", errors="ignore").strip()
                                if line:
                                    self.calib_status_received.emit(line)
                                text_buf.clear()
                            elif byte != 0x0D:
                                if len(text_buf) < 256:
                                    text_buf.append(byte)
                        else:
                            text_buf.clear()
                    elif state == 1:
                        if byte == HEADER_BYTE_1:  # 0xCC -> HR 结果包
                            buf.append(byte)
                            expected_len = PACKET_LEN  # 31
                            state = 2
                        elif byte == RAW_HEADER_BYTE_1:  # 0xBB -> 原始数据包
                            buf.append(byte)
                            expected_len = RAW_PACKET_LEN  # 33
                            state = 2
                        elif byte == HEADER_BYTE_0:
                            # 连续 0xAA, 重新开始
                            buf = bytearray([byte])
                        else:
                            state = 0
                    elif state == 2:
                        buf.append(byte)
                        if len(buf) == expected_len:
                            # 收集满一帧, 按类型解析
                            if expected_len == PACKET_LEN:
                                pkt = parse_hr_packet(bytes(buf))
                                if pkt is not None:
                                    self.hr_packet_received.emit(pkt)
                            else:
                                self._raw_total += 1
                                pkt = parse_raw_packet(bytes(buf))
                                if pkt is not None:
                                    self.raw_packet_received.emit(pkt)
                                else:
                                    self._raw_invalid += 1
                            # 重置状态机
                            state = 0
                            buf = bytearray()
        except serial.SerialException as e:
            if self._running:
                self.error_occurred.emit(f"Serial error: {e}")
        finally:
            if self._serial and self._serial.is_open:
                self._serial.close()
            self.connection_changed.emit(False)

    def stop(self):
        """安全停止线程"""
        self._running = False
        self.wait(2000)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def get_raw_stats(self) -> tuple[int, int]:
        """返回原始数据包统计 (总包数, 无效包数)"""
        return self._raw_total, self._raw_invalid

    @staticmethod
    def list_ports() -> List[str]:
        """返回系统可用串口列表"""
        return [p.device for p in serial.tools.list_ports.comports()]
