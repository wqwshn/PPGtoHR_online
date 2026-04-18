"""
PPG Monitor - 协议定义与帧解析

支持两种数据包格式:
  1) 31 字节心率结果包 (1Hz, 0xAA 0xCC 帧头)
  2) 33 字节原始传感器包 (100Hz, 0xAA 0xBB 帧头)

=== 31 字节心率结果包 ===
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
  19-20 HF1 AC 幅值          uint16 BE   x100 mV (桥顶1)
  21-22 HF1-PPG 相关系数     int16 BE   x10000 (-1.0~+1.0)
  23-24 ACC-PPG 相关系数     int16 BE   x10000 (-1.0~+1.0, 最优轴)
  25-26 HF2 AC 幅值          uint16 BE   x100 mV (桥顶2)
  27-28 HF2-PPG 相关系数     int16 BE   x10000 (-1.0~+1.0)
  29    XOR 校验             uint8       bytes[2..28] 异或
  30    帧尾                uint8       0xCC

=== 33 字节多光谱原始传感器包 ===
  偏移  字段                类型        说明
  0-1   帧头                uint8 x2    0xAA, 0xBB
  2-3   桥顶2 (HF2)         uint16 BE   24bit高16bit, 8bit
  4-5   桥顶1 (HF1)         uint16 BE   24bit高16bit, 8bit
  6-7   桥中2               uint16 BE   24bit高16bit, 8bit
  8-9   桥中1               uint16 BE   24bit高16bit, 8bit
  10-15 ACC X/Y/Z           int16 x3    加速度计完整16位 (大端)
  16-21 GYRO X/Y/Z          int16 x3    陀螺仪角速度 (大端)
  22-24 PPG Green           3 bytes     17-bit 原始ADC值
  25-27 PPG Red             3 bytes     17-bit 原始ADC值
  28-30 PPG IR              3 bytes     17-bit 原始ADC值
  31    XOR 校验             uint8       bytes[2..30] 异或
  32    帧尾                uint8       0xCC
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

# 帧常量
HEADER_BYTE_0 = 0xAA
HEADER_BYTE_1 = 0xCC
FOOTER_BYTE = 0xCC

PACKET_LEN = 31
PAYLOAD_START = 2    # 帧头后的 payload 起始偏移
PAYLOAD_END = 29     # XOR 校验前一字节 (含)
XOR_START = 2        # XOR 计算起始偏移
XOR_END = 28         # XOR 计算结束偏移 (含)

# 33 字节多光谱原始传感器帧常量
RAW_HEADER_BYTE_0 = 0xAA
RAW_HEADER_BYTE_1 = 0xBB
RAW_FOOTER_BYTE = 0xCC
RAW_PACKET_LEN = 33
RAW_XOR_START = 2    # XOR 计算起始偏移
RAW_XOR_END = 30     # XOR 计算结束偏移 (含)
RAW_XOR_POS = 31     # XOR 校验值位置


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
    hf1_ac_mv: float        # HF1(桥顶1) AC 幅值 (mV)
    hf1_ppg_corr: float     # HF1-PPG 相关系数 (-1~+1)
    acc_ppg_corr: float     # ACC-PPG 相关系数 (-1~+1, 最优轴)
    hf2_ac_mv: float        # HF2(桥顶2) AC 幅值 (mV)
    hf2_ppg_corr: float     # HF2-PPG 相关系数 (-1~+1)


def _decode_int16(raw: int) -> int:
    """大端 uint16 转 int16 有符号"""
    if raw >= 0x8000:
        return raw - 0x10000
    return raw


def parse_hr_packet(data: bytes) -> Optional[HRPacket]:
    """
    解析 31 字节心率结果包.

    Args:
        data: 完整的 31 字节原始帧

    Returns:
        HRPacket 解析成功, None 校验失败
    """
    if len(data) != PACKET_LEN:
        return None

    # 校验帧头
    if data[0] != HEADER_BYTE_0 or data[1] != HEADER_BYTE_1:
        return None

    # 校验帧尾
    if data[30] != FOOTER_BYTE:
        return None

    # XOR 校验: bytes[2..28] 异或 == data[29]
    xor_val = 0
    for i in range(XOR_START, XOR_END + 1):
        xor_val ^= data[i]
    if xor_val != data[29]:
        return None

    # 大端解析各字段
    fused_bpm_raw = (data[2] << 8) | data[3]
    hr_lms_hf_raw = (data[6] << 8) | data[7]
    hr_lms_acc_raw = (data[8] << 8) | data[9]
    hr_fft_raw = (data[10] << 8) | data[11]
    ppg_mean_raw = (data[12] << 8) | data[13]
    timestamp_raw = (data[15] << 8) | data[16]

    # 信号质量字段
    hf1_ac_raw = (data[19] << 8) | data[20]
    hf1_corr_raw = _decode_int16((data[21] << 8) | data[22])
    acc_corr_raw = _decode_int16((data[23] << 8) | data[24])
    hf2_ac_raw = (data[25] << 8) | data[26]
    hf2_corr_raw = _decode_int16((data[27] << 8) | data[28])

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
        hf1_ac_mv=hf1_ac_raw / 100.0,
        hf1_ppg_corr=hf1_corr_raw / 10000.0,
        acc_ppg_corr=acc_corr_raw / 10000.0,
        hf2_ac_mv=hf2_ac_raw / 100.0,
        hf2_ppg_corr=hf2_corr_raw / 10000.0,
    )


# ── 33 字节多光谱原始传感器包 ──────────────────────────────

@dataclass
class RawDataPacket:
    """解析后的 33 字节多光谱原始传感器数据包"""
    Ut2: float            # 热膜桥顶2 电压 (mV)
    Ut1: float            # 热膜桥顶1 电压 (mV)
    Uc2: float            # 热膜桥中2 电压 (mV)
    Uc1: float            # 热膜桥中1 电压 (mV)
    acc_x: float          # 加速度 X 轴 (g, 完整16位)
    acc_y: float          # 加速度 Y 轴 (g)
    acc_z: float          # 加速度 Z 轴 (g)
    gyro_x: float         # 陀螺仪 X 轴角速度 (dps)
    gyro_y: float         # 陀螺仪 Y 轴角速度 (dps)
    gyro_z: float         # 陀螺仪 Z 轴角速度 (dps)
    ppg_green: float      # 绿光 PPG 17-bit 原始值
    ppg_red: float        # 红光 PPG 17-bit 原始值
    ppg_ir: float         # 红外 PPG 17-bit 原始值


def parse_raw_packet(data: bytes) -> Optional[RawDataPacket]:
    """
    解析 33 字节多光谱原始传感器数据包.

    Args:
        data: 完整的 33 字节原始帧

    Returns:
        RawDataPacket 解析成功, None 校验失败
    """
    if len(data) != RAW_PACKET_LEN:
        return None

    # 校验帧头
    if data[0] != RAW_HEADER_BYTE_0 or data[1] != RAW_HEADER_BYTE_1:
        return None

    # 校验帧尾
    if data[32] != RAW_FOOTER_BYTE:
        return None

    # XOR 校验: bytes[2..30] 异或 == data[31]
    xor_val = 0
    for i in range(RAW_XOR_START, RAW_XOR_END + 1):
        xor_val ^= data[i]
    if xor_val != data[RAW_XOR_POS]:
        return None

    # ADC 桥压 (uint16 BE, 与旧格式兼容: 24bit高16bit, 低8bit填零)
    def _parse_bridge(b2: int, b3: int) -> float:
        num = (b2 << 16) | (b3 << 8)
        return (num / 8388608.0) * 2.5 * 1000.0  # mV

    Ut2 = _parse_bridge(data[2], data[3])
    Ut1 = _parse_bridge(data[4], data[5])
    Uc2 = _parse_bridge(data[6], data[7])
    Uc1 = _parse_bridge(data[8], data[9])

    # ACC (int16 BE, 完整16位, +-4g 量程)
    RANGE_ACC = 4.0 / 32767.0
    acc_x = _decode_int16((data[10] << 8) | data[11]) * RANGE_ACC
    acc_y = _decode_int16((data[12] << 8) | data[13]) * RANGE_ACC
    acc_z = _decode_int16((data[14] << 8) | data[15]) * RANGE_ACC

    # GYRO (int16 BE, ±500dps 灵敏度 17.50 mdps/LSB, 与固件 CTRL_REG1_G=0x68 一致)
    RANGE_GYRO = 17.50 / 1000.0   # dps/LSB
    gyro_x = _decode_int16((data[16] << 8) | data[17]) * RANGE_GYRO
    gyro_y = _decode_int16((data[18] << 8) | data[19]) * RANGE_GYRO
    gyro_z = _decode_int16((data[20] << 8) | data[21]) * RANGE_GYRO

    # PPG 三通道 (3 bytes each, 17-bit 对齐后的值)
    ppg_green = ((data[22] << 16) | (data[23] << 8) | data[24]) & 0x01FFFF
    ppg_red   = ((data[25] << 16) | (data[26] << 8) | data[27]) & 0x01FFFF
    ppg_ir    = ((data[28] << 16) | (data[29] << 8) | data[30]) & 0x01FFFF

    return RawDataPacket(
        Ut2=Ut2, Ut1=Ut1, Uc2=Uc2, Uc1=Uc1,
        acc_x=acc_x, acc_y=acc_y, acc_z=acc_z,
        gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z,
        ppg_green=ppg_green, ppg_red=ppg_red, ppg_ir=ppg_ir,
    )
