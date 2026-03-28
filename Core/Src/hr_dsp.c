/**
 * @file    hr_dsp.c
 * @brief   DSP 原语实现: FFT 峰值提取、时延搜索、频谱惩罚、中值平滑等
 * @details 所有函数依赖 CMSIS-DSP 库, 缓冲区由调用者提供, 无动态内存分配.
 *          包含预计算的 IIR 带通滤波器系数和所有 hr_dsp.h 中声明的函数.
 */

#include "hr_dsp.h"
#include "hr_algorithm.h"
#include <string.h>

/* ============================================================
 * 预计算的 Butterworth 带通滤波器系数 (0.5-5 Hz @ 125 Hz)
 * ============================================================
 * 由 scipy.signal.butter(4, [0.5, 5], btype='bandpass', fs=125) 精确计算.
 * 4 阶带通滤波器产生 4 个 SOS 节, 此处使用前 2 节作为近似.
 * 每节格式: [b0, b1, b2, -a1, -a2] (CMSIS-DSP biquad 格式).
 */
const float HR_BPF_COEFFS[10] = {
    /* 第 1 节: [b0, b1, b2, -a1, -a2] */
    0.00554271f,  0.00000000f, -0.00554271f,  1.78965868f, -0.94280904f,
    /* 第 2 节: [b0, b1, b2, -a1, -a2] */
    0.00554271f,  0.00000000f, -0.00554271f,  1.79519549f, -0.96889637f
};

/* ============================================================
 * DSP_FFTPeaks: FFT 峰值提取
 * ============================================================
 * 对输入信号做零填充 FFT, 计算单边幅度谱, 搜索局部极大值,
 * 在 HR_FREQ_LOW ~ HR_FREQ_HIGH 范围内按阈值筛选峰值.
 */
void DSP_FFTPeaks(const float *signal, uint16_t sig_len,
                  float *fft_input, float *fft_output,
                  float Fs, float percent,
                  float *peak_freq, float *peak_amp,
                  uint16_t max_peaks, uint16_t *num_peaks)
{
    /* FFT 实例只初始化一次 */
    static arm_rfft_fast_instance_f32 fft_instance;
    static uint8_t fft_initialized = 0;

    float32_t mean_val;
    uint16_t half_n;
    uint16_t k;
    uint16_t count;
    float    max_amp;
    float    threshold;

    *num_peaks = 0;
    half_n = HR_FFT_LEN / 2;

    /* 初始化 FFT 实例 (仅首次) */
    if (!fft_initialized) {
        arm_rfft_fast_init_f32(&fft_instance, HR_FFT_LEN);
        fft_initialized = 1;
    }

    /* 清零 FFT 输入缓冲区, 然后复制信号 (零填充) */
    memset(fft_input, 0, HR_FFT_LEN * sizeof(float));
    memcpy(fft_input, signal, sig_len * sizeof(float));

    /* 去均值以消除直流分量对频谱的影响 */
    arm_mean_f32(fft_input, sig_len, &mean_val);
    arm_offset_f32(fft_input, -mean_val, fft_input, sig_len);

    /* 执行实数 FFT */
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

    /* 计算复数幅度谱: fft_output 前 half_n 个元素是复数交错格式
     * arm_cmplx_mag_f32 将 [re, im, re, im, ...] 转换为 [mag, mag, ...]
     * 输入长度为 half_n 个复数 = half_n * 2 个 float
     * 复用 fft_input 作为幅度谱缓冲区 (此时 FFT 已完成, fft_input 不再需要)
     * 避免 8KB 栈分配导致嵌入式系统栈溢出 */
    float *mag = fft_input;
    arm_cmplx_mag_f32(fft_output, mag, half_n);

    /* 构建单边幅度谱:
     * DC 分量 (bin 0) 保持原值,
     * bin 1 ~ half_n-1 乘以 2 (镜像折叠) */
    /* mag[0] 不变 */
    for (k = 1; k < half_n; k++) {
        mag[k] *= 2.0f;
    }

    /* 在心率频率范围内找到最大幅度, 用于计算阈值 */
    max_amp = 0.0f;
    for (k = 1; k < half_n - 1; k++) {
        float freq = (float)k * Fs / (float)HR_FFT_LEN;
        if (freq >= HR_FREQ_LOW && freq <= HR_FREQ_HIGH) {
            if (mag[k] > max_amp) {
                max_amp = mag[k];
            }
        }
    }

    threshold = max_amp * percent;
    count = 0;

    /* 搜索局部极大值: mag[k] > mag[k-1] && mag[k] > mag[k+1] */
    for (k = 1; k < half_n - 1 && count < max_peaks; k++) {
        float freq = (float)k * Fs / (float)HR_FFT_LEN;

        /* 频率范围过滤 */
        if (freq < HR_FREQ_LOW || freq > HR_FREQ_HIGH) {
            continue;
        }

        /* 局部极大值检测 */
        if (mag[k] > mag[k - 1] && mag[k] > mag[k + 1]) {
            /* 阈值过滤 */
            if (mag[k] >= threshold) {
                peak_freq[count] = freq;
                peak_amp[count]  = mag[k];
                count++;
            }
        }
    }

    *num_peaks = count;
}

/* ============================================================
 * DSP_FindDelay: 时延搜索
 * ============================================================
 * 在 d = -5 ~ +5 范围内搜索最优时延.
 * 对每个时延值, 取 ppg[0..len-1] 和 ref[5+d..5+d+len-1],
 * 去均值后计算点积, 返回绝对值最大的点积对应的时延 d.
 */
int16_t DSP_FindDelay(const float *ppg, const float *ref, uint16_t len,
                      float *scratch, float *max_corr)
{
    int16_t best_d = 0;
    float   best_dot = 0.0f;
    float32_t mean_ppg;
    float32_t dot;
    int16_t   d;
    uint16_t  i;

    for (d = -5; d <= 5; d++) {
        /* 复制 ppg 段到 scratch */
        memcpy(scratch, ppg, len * sizeof(float));

        /* 对 ppg 段去均值 */
        arm_mean_f32(scratch, len, &mean_ppg);
        arm_offset_f32(scratch, -mean_ppg, scratch, len);

        /* 对 ref 段去均值: 由于 scratch 仅 len 长 (已被 ppg 去均值占用),
         * 使用逐元素计算 ref 段去均值并与 scratch 中的 ppg 做点积 */
        float ref_seg_mean = 0.0f;
        for (i = 0; i < len; i++) {
            ref_seg_mean += ref[5 + d + i];
        }
        ref_seg_mean /= (float)len;

        /* 计算 ppg(去均值) 与 ref段(去均值) 的点积 */
        /* scratch 中已经是 ppg 去均值后的数据 */
        dot = 0.0f;
        for (i = 0; i < len; i++) {
            float ref_val = ref[5 + d + i] - ref_seg_mean;
            dot += scratch[i] * ref_val;
        }

        /* 寻找绝对值最大的点积 */
        if (dot < 0.0f) {
            dot = -dot;
        }

        if (dot > best_dot) {
            best_dot = dot;
            best_d   = d;
        }
    }

    *max_corr = best_dot;
    return best_d;
}

/* ============================================================
 * DSP_SortPeaksByAmp: 按幅值降序排列峰值
 * ============================================================
 * 使用插入排序, 同时交换频率和幅值数组保持对应关系.
 */
void DSP_SortPeaksByAmp(float *freqs, float *amps, uint16_t n)
{
    uint16_t i, j;
    float    key_freq, key_amp;

    for (i = 1; i < n; i++) {
        key_freq = freqs[i];
        key_amp  = amps[i];
        j = i;

        /* 按幅值降序: 将大于 key_amp 的元素后移 */
        while (j > 0 && amps[j - 1] < key_amp) {
            freqs[j] = freqs[j - 1];
            amps[j]  = amps[j - 1];
            j--;
        }

        freqs[j] = key_freq;
        amps[j]  = key_amp;
    }
}

/* ============================================================
 * DSP_TrackHR: 心率追踪
 * ============================================================
 * 在按幅值降序排列的候选峰中, 优先选择与上一次心率最接近的峰值.
 * 扫描前 min(5, n) 个候选, 返回第一个满足范围约束的频率.
 */
float DSP_TrackHR(const float *freqs, uint16_t n,
                  float hr_prev, float range_plus, float range_minus)
{
    uint16_t scan_count;
    uint16_t i;
    float    diff;

    /* 最多扫描前 5 个候选峰 */
    scan_count = (n < 5) ? n : 5;

    for (i = 0; i < scan_count; i++) {
        diff = freqs[i] - hr_prev;
        if (diff < range_plus && diff > range_minus) {
            return freqs[i];
        }
    }

    /* 无匹配, 保持上一次心率 */
    return hr_prev;
}

/* ============================================================
 * DSP_SpectrumPenalty: 频谱惩罚
 * ============================================================
 * 对运动频率及其二次谐波附近的峰值施加惩罚 (幅值乘以权重).
 * 权重越小抑制越强.
 */
void DSP_SpectrumPenalty(float *peak_freq, float *peak_amp,
                         uint16_t num_peaks,
                         float motion_freq, float penalty_width,
                         float penalty_weight)
{
    uint16_t i;
    float    diff1, diff2;
    float    motion_2x;

    motion_2x = 2.0f * motion_freq;  /* 二次谐波 */

    for (i = 0; i < num_peaks; i++) {
        diff1 = peak_freq[i] - motion_freq;
        if (diff1 < 0.0f) diff1 = -diff1;

        diff2 = peak_freq[i] - motion_2x;
        if (diff2 < 0.0f) diff2 = -diff2;

        /* 如果峰值频率落在运动频率或二次谐波的惩罚范围内 */
        if (diff1 < penalty_width || diff2 < penalty_width) {
            peak_amp[i] *= penalty_weight;
        }
    }
}

/* ============================================================
 * DSP_MedianSmooth: 中值平滑
 * ============================================================
 * 取心率历史数组的最后 win_len 个元素, 用插入排序求中值.
 * 本地数组最多 9 个元素.
 */
void DSP_MedianSmooth(const float *hist, uint16_t len,
                      uint8_t win_len, float *out)
{
    float    buf[9];
    uint16_t start;
    uint16_t actual_len;
    uint16_t i, j;
    float    key;

    /* 确定实际使用的元素数量 */
    if (len < win_len) {
        actual_len = (len < 9) ? len : 9;
    } else {
        actual_len = (win_len < 9) ? win_len : 9;
    }

    if (actual_len == 0) {
        *out = 0.0f;
        return;
    }

    /* 从历史数组末尾取 actual_len 个元素 */
    start = len - actual_len;
    memcpy(buf, &hist[start], actual_len * sizeof(float));

    /* 插入排序 (升序) */
    for (i = 1; i < actual_len; i++) {
        key = buf[i];
        j = i;
        while (j > 0 && buf[j - 1] > key) {
            buf[j] = buf[j - 1];
            j--;
        }
        buf[j] = key;
    }

    /* 返回中间元素 */
    *out = buf[actual_len / 2];
}
