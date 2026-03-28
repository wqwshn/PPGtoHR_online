/**
 * @file    hr_dsp.h
 * @brief   DSP 原语声明: FFT 峰值提取、时延搜索、频谱惩罚、中值平滑等
 * @details 所有 DSP 辅助函数的声明, 以及预计算的 IIR 带通滤波器系数.
 *          函数实现中依赖 CMSIS-DSP 库 (arm_rfft_fast_f32, arm_dot_prod_f32 等).
 *          所有缓冲区均由调用者提供, 不使用动态内存分配.
 */

#ifndef HR_DSP_H
#define HR_DSP_H

#include "arm_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 预计算的 4 阶 Butterworth 带通滤波器系数 (0.5-5 Hz @ 125 Hz) */
/* 4 个 biquad 节, 每节 5 个系数 [b0, b1, b2, -a1, -a2] */
/* 外部通过 MATLAB/Octave 预计算, 运行时不可修改 */
extern const float HR_BPF_COEFFS[20];  /* 4 sections x 5 coeffs */

/**
 * @brief 对信号进行 FFT 并提取频谱峰值
 * @param signal:    输入信号 (时域, 长度 sig_len)
 * @param sig_len:   信号长度
 * @param fft_input: FFT 工作缓冲区 (>= HR_FFT_LEN)
 * @param fft_output:FFT 输出缓冲区 (>= HR_FFT_LEN)
 * @param Fs:        采样率
 * @param percent:   峰值阈值比例 (如 0.3)
 * @param peak_freq: 输出峰值频率数组
 * @param peak_amp:  输出峰值幅值数组
 * @param max_peaks: 峰值数组最大容量
 * @param num_peaks: 输出实际峰值数
 */
void DSP_FFTPeaks(const float *signal, uint16_t sig_len,
                  float *fft_input, float *fft_output,
                  float Fs, float percent,
                  float *peak_freq, float *peak_amp,
                  uint16_t max_peaks, uint16_t *num_peaks);

/**
 * @brief 在 +/-5 个采样点范围内搜索最优时延 (基于去均值点积, 近似皮尔逊相关)
 *
 * 实现说明:
 *   MATLAB 原版使用 corr() (皮尔逊相关), 在线版用去均值点积近似.
 *   对于每个时延 d (-5..+5):
 *     1. 从 ref 中取 [d : d+len-1] 段 (等价于参考信号时间平移 d 个采样)
 *     2. 对 ppg 段和 ref 段分别去均值
 *     3. 计算点积 arm_dot_prod_f32
 *   返回绝对值最大的点积对应的时延 d.
 *
 * @param ppg:     PPG 信号 (8 秒窗口, 长度 len)
 * @param ref:     参考信号 (8 秒窗口 + 10 点余量, 长度 >= len + 10)
 *                 ref[5..5+len-1] 对应零时延对齐位置
 * @param len:     有效窗口长度 (1000 点)
 * @param scratch: 临时缓冲区 (>= len, 用于去均值)
 * @param max_corr:输出最大相关度 (绝对值)
 * @return 最优时延 (采样点数, -5 ~ +5)
 */
int16_t DSP_FindDelay(const float *ppg, const float *ref, uint16_t len,
                      float *scratch, float *max_corr);

/**
 * @brief 按幅值降序排列峰值频率
 * @param freqs: 峰值频率数组
 * @param amps:  峰值幅值数组
 * @param n:     峰值数量
 */
void DSP_SortPeaksByAmp(float *freqs, float *amps, uint16_t n);

/**
 * @brief 在候选峰中选择最接近历史心率的峰值
 * @param freqs:    已排序的候选频率数组 (按幅值降序)
 * @param n:        候选数量
 * @param hr_prev:  上一次心率 (Hz)
 * @param range_plus: 正向搜索范围 (Hz)
 * @param range_minus: 负向搜索范围 (Hz, 通常为负值)
 * @return 选中的心率 (Hz), 若无匹配则返回 hr_prev
 */
float DSP_TrackHR(const float *freqs, uint16_t n,
                  float hr_prev, float range_plus, float range_minus);

/**
 * @brief 频谱惩罚: 抑制运动频率及其二次谐波
 * @param peak_freq: 峰值频率数组 (输入, 仅读取)
 * @param peak_amp:  峰值幅值数组 (会被修改: 命中惩罚区域的峰值幅值乘以权重)
 * @param num_peaks: 峰值数量
 * @param motion_freq: 运动主频 (Hz)
 * @param penalty_width: 惩罚宽度 (Hz)
 * @param penalty_weight: 惩罚权重 (0-1, 越小抑制越强)
 */
void DSP_SpectrumPenalty(float *peak_freq, float *peak_amp,
                         uint16_t num_peaks,
                         float motion_freq, float penalty_width,
                         float penalty_weight);

/**
 * @brief 简易中值平滑 (滑动窗口)
 * @param hist:     心率历史数组
 * @param len:      数组长度
 * @param win_len:  平滑窗口长度 (奇数)
 * @param out:      输出平滑后的最后一个值
 */
void DSP_MedianSmooth(const float *hist, uint16_t len,
                      uint8_t win_len, float *out);

#ifdef __cplusplus
}
#endif

#endif /* HR_DSP_H */
