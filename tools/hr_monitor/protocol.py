"""
PPG Heart Rate Monitor - 协议定义与帧解析

21 字节心率结果包 (1Hz, 蓝牙/串口 115200bps):
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
  18    采样率              uint8       固定 125
  19    XOR 校验             uint8       bytes[2..18] 异或
  20    帧尾                uint8       0xCC
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

# 帧常量
HEADER_BYTE_0 = 0xAA
HEADER_BYTE_1 = 0xCC
FOOTER_BYTE = 0xCC

PACKET_LEN = 21
PAYLOAD_START = 2    # 帧头后的 payload 起始偏移
PAYLOAD_END = 19     # XOR 校验前一字节 (含)
XOR_START = 2        # XOR 计算起始偏移
XOR_END = 18         # XOR 计算结束偏移 (含)


@dataclass
class HRPacket:
    """解析后的心率数据包"""
    fused_bpm: float        # 融合心率 (BPM)
    is_motion: bool         # 运动标志
    win_filled: bool        # 窗口是否已填满
    hr_lms_hf: float        # LMS-HF 路径 BPM
    hr_lms_acc: float       # LMS-ACC 路径 BPM
    hr_fft: float           # FFT 路径 BPM
    ppg_mean: int           # PPG 信号均值 (原始均值)
    motion_calibrated: bool # 运动阈值是否已校准
    timestamp: int          # 秒计数器
    calib_progress: int     # 校准进度 0-8
    sampling_rate: int      # 采样率 (Hz)


def parse_hr_packet(data: bytes) -> Optional[HRPacket]:
    """
    解析 21 字节心率结果包.

    Args:
        data: 完整的 21 字节原始帧

    Returns:
        HRPacket 解析成功, None 校验失败
    """
    if len(data) != PACKET_LEN:
        return None

    # 校验帧头
    if data[0] != HEADER_BYTE_0 or data[1] != HEADER_BYTE_1:
        return None

    # 校验帧尾
    if data[20] != FOOTER_BYTE:
        return None

    # XOR 校验: bytes[2..18] 异或 == data[19]
    xor_val = 0
    for i in range(XOR_START, XOR_END + 1):
        xor_val ^= data[i]
    if xor_val != data[19]:
        return None

    # 大端解析各字段
    fused_bpm_raw = (data[2] << 8) | data[3]
    hr_lms_hf_raw = (data[6] << 8) | data[7]
    hr_lms_acc_raw = (data[8] << 8) | data[9]
    hr_fft_raw = (data[10] << 8) | data[11]
    ppg_mean_raw = (data[12] << 8) | data[13]
    timestamp_raw = (data[15] << 8) | data[16]

    return HRPacket(
        fused_bpm=fused_bpm_raw / 10.0,
        is_motion=bool(data[4]),
        win_filled=bool(data[5]),
        hr_lms_hf=hr_lms_hf_raw / 10.0,
        hr_lms_acc=hr_lms_acc_raw / 10.0,
        hr_fft=hr_fft_raw / 10.0,
        ppg_mean=ppg_mean_raw,
        motion_calibrated=bool(data[14]),
        timestamp=timestamp_raw,
        calib_progress=data[17],
        sampling_rate=data[18],
    )
