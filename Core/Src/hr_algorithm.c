/**
 * @file    hr_algorithm.c
 * @brief   在线心率算法编排器: 三路融合心率估计
 * @details 实现完整的采样输入 -> 滑动窗口 -> 三路心率估计 -> 融合决策流水线.
 *          三路估计: LMS-HF, LMS-ACC, Pure-FFT, 根据运动状态选择最优路径.
 *          所有缓冲区静态分配, 不使用动态内存.
 */

#include "hr_algorithm.h"
#include "hr_dsp.h"
#include "hr_lms.h"
#include <string.h>
#include <math.h>

/* ============================================================
 * 模块内部静态配置 (Init 时从 HR_Config_t 拷贝)
 * ============================================================ */
static HR_Config_t s_config;

/* ============================================================
 * zscore_inplace: 就地 zscore 归一化
 * ============================================================
 * 对信号做去均值 + 除标准差, 使信号均值为 0, 标准差为 1.
 * 用于 LMS 输入归一化, 提升自适应滤波收敛性能.
 */
static inline void zscore_inplace(float *sig, uint16_t len)
{
    float mean_val, std_val;

    arm_mean_f32(sig, len, &mean_val);
    arm_std_f32(sig, len, &std_val);
    if (std_val < 1e-6f) {
        std_val = 1e-6f;
    }
    arm_offset_f32(sig, -mean_val, sig, len);
    arm_scale_f32(sig, 1.0f / std_val, sig, len);
}

/* ============================================================
 * HR_GetDefaultConfig - 获取默认配置参数
 * ============================================================ */
void HR_GetDefaultConfig(HR_Config_t *config)
{
    config->LMS_Mu_Base        = 0.01f;
    config->Spec_Penalty_Width = 0.2f;
    config->Spec_Penalty_Weight= 0.2f;
    config->HR_Range_Hz        = 0.25f;
    config->HR_Range_Rest_Hz   = 0.4f;
    config->Slew_Limit_BPM     = 10.0f;
    config->Slew_Step_BPM      = 2.0f;
    config->Slew_Limit_Rest_BPM= 8.0f;
    config->Slew_Step_Rest_BPM = 1.0f;
    config->Motion_Th_Scale    = 3.0f;
    config->Default_Motion_Th  = 200.0f;
    config->Spec_Penalty_Enable = 1;
}

/* ============================================================
 * HR_Init - 初始化算法状态
 * ============================================================
 * 清零所有状态, 初始化 IIR 滤波器, LMS 滤波器, 预计算 Hamming 窗.
 */
void HR_Init(const HR_Config_t *config, HR_State_t *state)
{
    uint16_t i;

    /* 清零整个状态结构体 */
    memset(state, 0, sizeof(HR_State_t));

    /* 保存配置参数 (使用静态变量存储) */
    if (config != NULL) {
        memcpy(&s_config, config, sizeof(HR_Config_t));
    } else {
        HR_GetDefaultConfig(&s_config);
    }

    /* --- 初始化 5 个 IIR biquad 滤波器实例 --- */
    /* 每个通道: 2 个 biquad 节, 使用预计算的 HR_BPF_COEFFS 系数 */
    arm_biquad_cascade_df1_init_f32(&state->biquad_ppg,  2, (float *)HR_BPF_COEFFS, state->iir_state_ppg);
    arm_biquad_cascade_df1_init_f32(&state->biquad_accx, 2, (float *)HR_BPF_COEFFS, state->iir_state_accx);
    arm_biquad_cascade_df1_init_f32(&state->biquad_accy, 2, (float *)HR_BPF_COEFFS, state->iir_state_accy);
    arm_biquad_cascade_df1_init_f32(&state->biquad_accz, 2, (float *)HR_BPF_COEFFS, state->iir_state_accz);
    arm_biquad_cascade_df1_init_f32(&state->biquad_hf,   2, (float *)HR_BPF_COEFFS, state->iir_state_hf);

    /* --- 初始化 LMS HF 路径: 2 级级联 --- */
    for (i = 0; i < HR_LMS_CASCADE_HF; i++) {
        LMS_Init(&state->lms_hf[i],
                 state->lms_hf_coeffs[i],
                 state->lms_hf_state[i],
                 HR_MAX_ORDER,
                 s_config.LMS_Mu_Base,
                 HR_WIN_SAMPLES);
    }

    /* --- 初始化 LMS ACC 路径: 3 级级联 --- */
    for (i = 0; i < HR_LMS_CASCADE_ACC; i++) {
        LMS_Init(&state->lms_acc[i],
                 state->lms_acc_coeffs[i],
                 state->lms_acc_state[i],
                 HR_MAX_ORDER,
                 s_config.LMS_Mu_Base,
                 HR_WIN_SAMPLES);
    }

    /* --- 预计算 Hamming 窗 --- */
    for (i = 0; i < HR_WIN_SAMPLES; i++) {
        state->hamming_win[i] = 0.54f - 0.46f * cosf(2.0f * PI * (float)i / (float)(HR_WIN_SAMPLES - 1));
    }

    /* --- 设置初始心率 (1.2 Hz = 72 BPM) --- */
    state->hr_lms_hf  = 1.2f;
    state->hr_lms_acc = 1.2f;
    state->hr_fft     = 1.2f;
    state->hr_fused   = 1.2f;

    /* --- 运动阈值初始化 --- */
    state->motion_threshold = s_config.Default_Motion_Th;
}

/* ============================================================
 * HR_PushSample - 向 1 秒缓冲区追加一个采样点
 * ============================================================
 * 由 ISR 或采集逻辑调用, 采样率 125 Hz, 累积 125 点后置就绪标志.
 * RunSolver 消费数据后重置索引, PushSample 不主动重置.
 */
void HR_PushSample(HR_State_t *state,
                   float ppg, float accx, float accy, float accz, float hf)
{
    /* 溢出保护: 缓冲区已满但未被消费, 丢弃新采样 */
    if (state->sample_idx >= HR_STEP_SAMPLES) {
        return;
    }

    /* 存入 1 秒采集缓冲区 */
    state->buf_1s_ppg[state->sample_idx]  = ppg;
    state->buf_1s_accx[state->sample_idx] = accx;
    state->buf_1s_accy[state->sample_idx] = accy;
    state->buf_1s_accz[state->sample_idx] = accz;
    state->buf_1s_hf[state->sample_idx]   = hf;

    state->sample_idx++;

    /* 125 点收齐, 置就绪标志通知 RunSolver */
    if (state->sample_idx >= HR_STEP_SAMPLES) {
        state->flag_1s_ready = 1;
    }
}

/* ============================================================
 * HR_RunSolver - 执行一次完整的心率解算流水线
 * ============================================================
 * 返回: 融合心率 (BPM), 0 表示窗口未填满或数据未就绪.
 *
 * 流水线共 14 步:
 *   1. 就绪检查  2. 原始窗口平移  3. IIR 滤波
 *   4. 滤波窗口平移  5. 运动检测  6. 窗口填充检查
 *   7. 时延对齐  8. LMS-HF 路径  9. LMS-ACC 路径
 *  10. Pure-FFT 路径  11. 融合决策  12. 中值平滑
 *  13. 变化率限制  14. 输出 BPM
 */
float HR_RunSolver(HR_State_t *state)
{
    /* ======================================================
     * Step 1: 检查 1 秒数据就绪标志
     * ====================================================== */
    if (!state->flag_1s_ready) {
        return 0.0f;
    }
    state->flag_1s_ready = 0;
    state->sample_idx = 0;

    /* ======================================================
     * Step 2: 原始滑动窗口平移 (丢弃最旧的 1 秒, 追加新的 1 秒)
     * ====================================================== */
    /* PPG */
    memmove(state->win_ppg,
            state->win_ppg + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->win_ppg + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->buf_1s_ppg,
           HR_STEP_SAMPLES * sizeof(float));

    /* ACC X */
    memmove(state->win_accx,
            state->win_accx + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->win_accx + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->buf_1s_accx,
           HR_STEP_SAMPLES * sizeof(float));

    /* ACC Y */
    memmove(state->win_accy,
            state->win_accy + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->win_accy + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->buf_1s_accy,
           HR_STEP_SAMPLES * sizeof(float));

    /* ACC Z */
    memmove(state->win_accz,
            state->win_accz + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->win_accz + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->buf_1s_accz,
           HR_STEP_SAMPLES * sizeof(float));

    /* HF */
    memmove(state->win_hf,
            state->win_hf + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->win_hf + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->buf_1s_hf,
           HR_STEP_SAMPLES * sizeof(float));

    /* ======================================================
     * Step 3: IIR 带通滤波 (仅对新的 125 点, 流式处理)
     * ====================================================== */
    arm_biquad_cascade_df1_f32(&state->biquad_ppg,
                               state->buf_1s_ppg,
                               state->iir_filt_1s_ppg,
                               HR_STEP_SAMPLES);

    arm_biquad_cascade_df1_f32(&state->biquad_accx,
                               state->buf_1s_accx,
                               state->iir_filt_1s_accx,
                               HR_STEP_SAMPLES);

    arm_biquad_cascade_df1_f32(&state->biquad_accy,
                               state->buf_1s_accy,
                               state->iir_filt_1s_accy,
                               HR_STEP_SAMPLES);

    arm_biquad_cascade_df1_f32(&state->biquad_accz,
                               state->buf_1s_accz,
                               state->iir_filt_1s_accz,
                               HR_STEP_SAMPLES);

    arm_biquad_cascade_df1_f32(&state->biquad_hf,
                               state->buf_1s_hf,
                               state->iir_filt_1s_hf,
                               HR_STEP_SAMPLES);

    /* ======================================================
     * Step 4: 滤波后滑动窗口平移 (与原始窗口同步)
     * ====================================================== */
    memmove(state->filt_ppg,
            state->filt_ppg + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->filt_ppg + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->iir_filt_1s_ppg,
           HR_STEP_SAMPLES * sizeof(float));

    memmove(state->filt_accx,
            state->filt_accx + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->filt_accx + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->iir_filt_1s_accx,
           HR_STEP_SAMPLES * sizeof(float));

    memmove(state->filt_accy,
            state->filt_accy + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->filt_accy + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->iir_filt_1s_accy,
           HR_STEP_SAMPLES * sizeof(float));

    memmove(state->filt_accz,
            state->filt_accz + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->filt_accz + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->iir_filt_1s_accz,
           HR_STEP_SAMPLES * sizeof(float));

    memmove(state->filt_hf,
            state->filt_hf + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->filt_hf + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->iir_filt_1s_hf,
           HR_STEP_SAMPLES * sizeof(float));

    /* ======================================================
     * Step 5: 运动检测
     * ======================================================
     * 计算三轴加速度幅值的标准差, 与阈值比较判断是否运动.
     * 支持自适应校准: 前 8 个静息窗口用于建立基线.
     */
    {
        float acc_std;

        /* 计算 ACC 幅值: sqrt(ax^2 + ay^2 + az^2) */
        /* scratch_a = ax^2 */
        arm_mult_f32(state->win_accx, state->win_accx, state->scratch_a, HR_WIN_SAMPLES);
        /* scratch_b = ay^2 */
        arm_mult_f32(state->win_accy, state->win_accy, state->scratch_b, HR_WIN_SAMPLES);
        /* scratch_a += ay^2 */
        arm_add_f32(state->scratch_a, state->scratch_b, state->scratch_a, HR_WIN_SAMPLES);
        /* scratch_b = az^2 */
        arm_mult_f32(state->win_accz, state->win_accz, state->scratch_b, HR_WIN_SAMPLES);
        /* scratch_a += az^2 */
        arm_add_f32(state->scratch_a, state->scratch_b, state->scratch_a, HR_WIN_SAMPLES);
        /* scratch_a = sqrt(scratch_a) = 幅值 */
        {
            uint16_t _si;
            for (_si = 0; _si < HR_WIN_SAMPLES; _si++) {
                float _v = state->scratch_a[_si];
                if (_v < 0.0f) _v = 0.0f;
                state->scratch_a[_si] = sqrtf(_v);
            }
        }

        /* 计算幅值的标准差 */
        arm_std_f32(state->scratch_a, HR_WIN_SAMPLES, &acc_std);

        /* 自适应校准逻辑 */
        if (!state->motion_calibrated) {
            if (acc_std < s_config.Default_Motion_Th) {
                /* 静息窗口: 累积标准差用于计算基线 */
                state->calib_std_accum += acc_std;
                state->calib_windows_done++;

                if (state->calib_windows_done >= 8) {
                    /* 计算平均静息标准差作为基线 */
                    state->acc_baseline_std = state->calib_std_accum / (float)state->calib_windows_done;
                    /* 阈值 = 基线 x 倍数 */
                    state->motion_threshold = state->acc_baseline_std * s_config.Motion_Th_Scale;
                    state->motion_calibrated = 1;
                }
            }
        }

        /* 运动判定 */
        state->is_motion = (acc_std > state->motion_threshold) ? 1 : 0;
    }

    /* ======================================================
     * Step 6: 窗口填充检查
     * ======================================================
     * 需要至少 8 个 1 秒步长 (8 秒窗口) 才能开始计算.
     */
    state->win_count++;
    if (state->win_count < HR_WIN_SEC) {
        return 0.0f;
    }
    state->win_filled = 1;

    /* ======================================================
     * Step 7: 时延对齐
     * ======================================================
     * 对每个参考通道计算与 PPG 的最优时延, 确定最高相关度通道.
     * 根据时延计算 LMS 滤波器阶数.
     */
    {
        float corr_accx, corr_accy, corr_accz, corr_hf;
        int16_t delay_accx, delay_accy, delay_accz, delay_hf;
        float max_corr_acc;
        int16_t best_delay_acc;
        uint8_t best_acc_idx;  /* 0=x, 1=y, 2=z */
        uint16_t order_hf, order_acc;

        /* ACC X 时延搜索 */
        delay_accx = DSP_FindDelay(state->filt_ppg, state->filt_accx,
                                   HR_WIN_SAMPLES, state->scratch_a, &corr_accx);
        /* ACC Y 时延搜索 */
        delay_accy = DSP_FindDelay(state->filt_ppg, state->filt_accy,
                                   HR_WIN_SAMPLES, state->scratch_a, &corr_accy);
        /* ACC Z 时延搜索 */
        delay_accz = DSP_FindDelay(state->filt_ppg, state->filt_accz,
                                   HR_WIN_SAMPLES, state->scratch_a, &corr_accz);
        /* HF 时延搜索 */
        delay_hf = DSP_FindDelay(state->filt_ppg, state->filt_hf,
                                 HR_WIN_SAMPLES, state->scratch_a, &corr_hf);

        /* 选择 ACC 三轴中相关度最高的通道 */
        max_corr_acc = corr_accx;
        best_delay_acc = delay_accx;
        best_acc_idx = 0;
        if (corr_accy > max_corr_acc) {
            max_corr_acc = corr_accy;
            best_delay_acc = delay_accy;
            best_acc_idx = 1;
        }
        if (corr_accz > max_corr_acc) {
            max_corr_acc = corr_accz;
            best_delay_acc = delay_accz;
            best_acc_idx = 2;
        }

        /* 根据 HF 时延计算 LMS 阶数 */
        if (delay_hf < 0) {
            order_hf = (uint16_t)(-delay_hf * 1.0f);
            if (order_hf < 1) order_hf = 1;
            if (order_hf > HR_MAX_ORDER) order_hf = HR_MAX_ORDER;
        } else {
            order_hf = HR_MAX_ORDER;
        }

        /* 根据 ACC 时延计算 LMS 阶数 (ACC 路径使用 1.5 倍因子) */
        if (best_delay_acc < 0) {
            order_acc = (uint16_t)(-best_delay_acc * 1.5f);
            if (order_acc < 1) order_acc = 1;
            if (order_acc > HR_MAX_ORDER) order_acc = HR_MAX_ORDER;
        } else {
            order_acc = HR_MAX_ORDER;
        }

        /* 重新初始化 LMS 滤波器以应用新阶数 */
        for (uint8_t i = 0; i < HR_LMS_CASCADE_HF; i++) {
            LMS_Init(&state->lms_hf[i],
                     state->lms_hf_coeffs[i],
                     state->lms_hf_state[i],
                     order_hf,
                     s_config.LMS_Mu_Base,
                     HR_WIN_SAMPLES);
        }
        for (uint8_t i = 0; i < HR_LMS_CASCADE_ACC; i++) {
            LMS_Init(&state->lms_acc[i],
                     state->lms_acc_coeffs[i],
                     state->lms_acc_state[i],
                     order_acc,
                     s_config.LMS_Mu_Base,
                     HR_WIN_SAMPLES);
        }

        /* ======================================================
         * Step 8: Path A - LMS-HF 路径
         * ======================================================
         * 使用 HF 参考信号通过 LMS 自适应滤波从 PPG 中消除高频噪声.
         * 对 LMS 误差输出做 FFT 峰值检测, 结合频谱惩罚和心率追踪.
         */
        {
            float *hf_refs[HR_LMS_CASCADE_HF];
            float motion_freq_hf;
            uint16_t num_peaks_hf;

            /* 将 PPG 拷贝到 scratch_a 并 zscore 归一化 */
            memcpy(state->scratch_a, state->filt_ppg, HR_WIN_SAMPLES * sizeof(float));
            zscore_inplace(state->scratch_a, HR_WIN_SAMPLES);

            /* 将 HF 信号拷贝到 scratch_b 并 zscore 归一化 */
            memcpy(state->scratch_b, state->filt_hf, HR_WIN_SAMPLES * sizeof(float));
            zscore_inplace(state->scratch_b, HR_WIN_SAMPLES);

            /* 设置 HF 参考通道指针 (两个级联均使用同一个 HF 信号) */
            hf_refs[0] = state->scratch_b;
            hf_refs[1] = state->scratch_b;

            /* 级联 LMS 处理: ref=HF噪声, desired=PPG, err=去噪后PPG */
            LMS_CascadeProcess(state->lms_hf,
                               HR_LMS_CASCADE_HF,
                               hf_refs,
                               state->scratch_a,
                               state->lms_hf_err,
                               state->lms_hf_tmp,
                               state->lms_hf_tmp2,
                               HR_WIN_SAMPLES);

            /* 对 LMS-HF 误差输出做 FFT 峰值检测 */
            DSP_FFTPeaks(state->lms_hf_err, HR_WIN_SAMPLES,
                         state->fft_input, state->fft_output,
                         (float)HR_FS, 0.3f,
                         state->peak_freqs, state->peak_amps,
                         HR_MAX_PEAKS, &num_peaks_hf);

            /* 频谱惩罚: 用 HF 信号的 FFT 峰值获取运动频率 */
            if (s_config.Spec_Penalty_Enable && num_peaks_hf > 0) {
                /* 从 HF 参考信号的 FFT 获取运动主频 */
                uint16_t num_ref_peaks;
                DSP_FFTPeaks(state->scratch_b, HR_WIN_SAMPLES,
                             state->fft_input, state->fft_output,
                             (float)HR_FS, 0.3f,
                             state->peak_freqs, state->peak_amps,
                             HR_MAX_PEAKS, &num_ref_peaks);

                motion_freq_hf = 0.0f;
                if (num_ref_peaks > 0) {
                    motion_freq_hf = state->peak_freqs[0]; /* 最强峰 = 运动频率 */
                }

                /* 重新对 LMS-HF 误差输出做 FFT 峰值检测 (因 scratch 被覆盖) */
                DSP_FFTPeaks(state->lms_hf_err, HR_WIN_SAMPLES,
                             state->fft_input, state->fft_output,
                             (float)HR_FS, 0.3f,
                             state->peak_freqs, state->peak_amps,
                             HR_MAX_PEAKS, &num_peaks_hf);

                /* 对运动频率及二次谐波附近峰值施加惩罚 */
                if (motion_freq_hf > 0.0f) {
                    DSP_SpectrumPenalty(state->peak_freqs, state->peak_amps,
                                       num_peaks_hf,
                                       motion_freq_hf,
                                       s_config.Spec_Penalty_Width,
                                       s_config.Spec_Penalty_Weight);
                }
            }

            /* 按幅值降序排列峰值 */
            DSP_SortPeaksByAmp(state->peak_freqs, state->peak_amps, num_peaks_hf);

            /* 心率追踪: 在候选峰中选择最接近历史心率的峰值 */
            state->hr_lms_hf = DSP_TrackHR(state->peak_freqs, num_peaks_hf,
                                            state->hr_lms_hf,
                                            s_config.HR_Range_Hz,
                                            -s_config.HR_Range_Hz);
        }

        /* ======================================================
         * Step 9: Path B - LMS-ACC 路径
         * ======================================================
         * 使用最优 ACC 轴作为参考, 通过 LMS 自适应滤波消除运动伪影.
         */
        {
            float *acc_refs[HR_LMS_CASCADE_ACC];
            float *best_acc_filt;
            float motion_freq_acc;
            uint16_t num_peaks_acc;

            /* 选择相关度最高的 ACC 通道 */
            if (best_acc_idx == 0) {
                best_acc_filt = state->filt_accx;
            } else if (best_acc_idx == 1) {
                best_acc_filt = state->filt_accy;
            } else {
                best_acc_filt = state->filt_accz;
            }

            /* 将 PPG 拷贝到 scratch_a 并 zscore 归一化 */
            memcpy(state->scratch_a, state->filt_ppg, HR_WIN_SAMPLES * sizeof(float));
            zscore_inplace(state->scratch_a, HR_WIN_SAMPLES);

            /* 将最优 ACC 通道拷贝到 scratch_b 并 zscore 归一化 */
            memcpy(state->scratch_b, best_acc_filt, HR_WIN_SAMPLES * sizeof(float));
            zscore_inplace(state->scratch_b, HR_WIN_SAMPLES);

            /* 设置 ACC 参考通道指针 (三个级联均使用同一信号) */
            acc_refs[0] = state->scratch_b;
            acc_refs[1] = state->scratch_b;
            acc_refs[2] = state->scratch_b;

            /* 级联 LMS 处理: ref=ACC噪声, desired=PPG, err=去噪后PPG */
            LMS_CascadeProcess(state->lms_acc,
                               HR_LMS_CASCADE_ACC,
                               acc_refs,
                               state->scratch_a,
                               state->lms_acc_err,
                               state->lms_acc_tmp,
                               state->lms_acc_tmp2,
                               HR_WIN_SAMPLES);

            /* 对 LMS-ACC 误差输出做 FFT 峰值检测 */
            DSP_FFTPeaks(state->lms_acc_err, HR_WIN_SAMPLES,
                         state->fft_input, state->fft_output,
                         (float)HR_FS, 0.3f,
                         state->peak_freqs, state->peak_amps,
                         HR_MAX_PEAKS, &num_peaks_acc);

            /* 频谱惩罚: 用 ACC 参考信号的 FFT 峰值获取运动频率 */
            if (s_config.Spec_Penalty_Enable && num_peaks_acc > 0) {
                uint16_t num_ref_peaks;
                DSP_FFTPeaks(state->scratch_b, HR_WIN_SAMPLES,
                             state->fft_input, state->fft_output,
                             (float)HR_FS, 0.3f,
                             state->peak_freqs, state->peak_amps,
                             HR_MAX_PEAKS, &num_ref_peaks);

                motion_freq_acc = 0.0f;
                if (num_ref_peaks > 0) {
                    motion_freq_acc = state->peak_freqs[0];
                }

                /* 重新对 LMS-ACC 误差输出做 FFT 峰值检测 */
                DSP_FFTPeaks(state->lms_acc_err, HR_WIN_SAMPLES,
                             state->fft_input, state->fft_output,
                             (float)HR_FS, 0.3f,
                             state->peak_freqs, state->peak_amps,
                             HR_MAX_PEAKS, &num_peaks_acc);

                /* 频谱惩罚 */
                if (motion_freq_acc > 0.0f) {
                    DSP_SpectrumPenalty(state->peak_freqs, state->peak_amps,
                                       num_peaks_acc,
                                       motion_freq_acc,
                                       s_config.Spec_Penalty_Width,
                                       s_config.Spec_Penalty_Weight);
                }
            }

            /* 按幅值降序排列峰值 */
            DSP_SortPeaksByAmp(state->peak_freqs, state->peak_amps, num_peaks_acc);

            /* 心率追踪 */
            state->hr_lms_acc = DSP_TrackHR(state->peak_freqs, num_peaks_acc,
                                             state->hr_lms_acc,
                                             s_config.HR_Range_Hz,
                                             -s_config.HR_Range_Hz);
        }
    }

    /* ======================================================
     * Step 10: Path C - Pure FFT 路径
     * ======================================================
     * 直接对滤波后 PPG 信号做 FFT, 不经过 LMS 去噪.
     * 适用于静息状态, 信号质量较好的场景.
     */
    {
        float mean_val;
        uint16_t num_peaks_fft;

        /* 将滤波后 PPG 拷贝到 scratch_a (作为 FFT 输入源) */
        memcpy(state->scratch_a, state->filt_ppg, HR_WIN_SAMPLES * sizeof(float));

        /* 去均值 */
        arm_mean_f32(state->scratch_a, HR_WIN_SAMPLES, &mean_val);
        arm_offset_f32(state->scratch_a, -mean_val, state->scratch_a, HR_WIN_SAMPLES);

        /* 应用 Hamming 窗 */
        arm_mult_f32(state->scratch_a, state->hamming_win, state->scratch_a, HR_WIN_SAMPLES);

        /* 注意: DSP_FFTPeaks 内部会 memset(fft_input, 0) 再 memcpy(signal -> fft_input),
         * 因此 signal 不能指向 fft_input 自身, 必须使用独立缓冲区 (scratch_a). */

        /* FFT 峰值检测 */
        DSP_FFTPeaks(state->scratch_a, HR_WIN_SAMPLES,
                     state->fft_input, state->fft_output,
                     (float)HR_FS, 0.3f,
                     state->peak_freqs, state->peak_amps,
                     HR_MAX_PEAKS, &num_peaks_fft);

        /* 频谱惩罚: 使用 ACC Z 轴作为参考 */
        if (s_config.Spec_Penalty_Enable && num_peaks_fft > 0) {
            float motion_freq_fft;
            uint16_t num_ref_peaks;

            DSP_FFTPeaks(state->filt_accz, HR_WIN_SAMPLES,
                         state->fft_input, state->fft_output,
                         (float)HR_FS, 0.3f,
                         state->peak_freqs, state->peak_amps,
                         HR_MAX_PEAKS, &num_ref_peaks);

            motion_freq_fft = 0.0f;
            if (num_ref_peaks > 0) {
                motion_freq_fft = state->peak_freqs[0];
            }

            /* 重新对 PPG FFT 做峰值检测 (因 scratch_a 可能被其他路径覆盖, 使用 fft_input 中已填充的数据) */
            DSP_FFTPeaks(state->scratch_a, HR_WIN_SAMPLES,
                         state->fft_input, state->fft_output,
                         (float)HR_FS, 0.3f,
                         state->peak_freqs, state->peak_amps,
                         HR_MAX_PEAKS, &num_peaks_fft);

            if (motion_freq_fft > 0.0f) {
                DSP_SpectrumPenalty(state->peak_freqs, state->peak_amps,
                                   num_peaks_fft,
                                   motion_freq_fft,
                                   s_config.Spec_Penalty_Width,
                                   s_config.Spec_Penalty_Weight);
            }
        }

        /* 按幅值降序排列 */
        DSP_SortPeaksByAmp(state->peak_freqs, state->peak_amps, num_peaks_fft);

        /* 心率追踪 */
        state->hr_fft = DSP_TrackHR(state->peak_freqs, num_peaks_fft,
                                     state->hr_fft,
                                     s_config.HR_Range_Rest_Hz,
                                     -s_config.HR_Range_Rest_Hz);
    }

    /* ======================================================
     * Step 11: 融合决策
     * ======================================================
     * 运动状态: 选择 LMS-ACC 路径 (运动伪影消除效果好)
     * 静息状态: 选择 Pure-FFT 路径 (信号干净, FFT 精度高)
     */
    if (state->is_motion) {
        state->hr_fused = state->hr_lms_acc;
    } else {
        state->hr_fused = state->hr_fft;
    }

    /* ======================================================
     * Step 12: 中值平滑 + 历史更新
     * ======================================================
     * 对心率历史做中值滤波平滑, 抑制突变.
     * 将当前融合心率写入环形历史缓冲区.
     */
    {
        float smoothed;
        uint16_t hist_len;

        /* 将融合心率写入历史 (环形缓冲区) */
        state->hr_history_lms_hf[state->hr_history_idx]  = state->hr_lms_hf;
        state->hr_history_lms_acc[state->hr_history_idx] = state->hr_lms_acc;
        state->hr_history_fft[state->hr_history_idx]     = state->hr_fft;

        /* 推进环形索引 */
        state->hr_history_idx++;
        if (state->hr_history_idx >= HR_HR_HISTORY_LEN) {
            state->hr_history_idx = 0;
        }

        /* 计算有效历史长度 */
        hist_len = state->win_count - HR_WIN_SEC + 1;
        if (hist_len > HR_HR_HISTORY_LEN) {
            hist_len = HR_HR_HISTORY_LEN;
        }

        /* 中值平滑 */
        DSP_MedianSmooth(state->hr_history_fft, hist_len,
                         HR_SMOOTH_WIN, &smoothed);

        /* 用平滑后的值替换融合心率 */
        state->hr_fused = smoothed;
    }

    /* ======================================================
     * Step 13: 变化率限制 (Slew Rate Limiting)
     * ======================================================
     * 限制相邻两次心率输出的最大变化幅度, 防止心率跳变.
     * 运动和静息使用不同的限制参数.
     */
    {
        float hr_prev;
        float diff;
        float limit;
        float step;

        /* 取上一次输出心率作为基准 (Hz) */
        if (state->hr_bpm > 0.0f) {
            hr_prev = state->hr_bpm / 60.0f;
        } else {
            /* 首次运行: 使用初始心率 */
            hr_prev = state->hr_fused;
        }

        diff = state->hr_fused - hr_prev;

        if (state->is_motion) {
            limit = s_config.Slew_Limit_BPM / 60.0f;
            step  = s_config.Slew_Step_BPM / 60.0f;
        } else {
            limit = s_config.Slew_Limit_Rest_BPM / 60.0f;
            step  = s_config.Slew_Step_Rest_BPM / 60.0f;
        }

        if (diff > limit) {
            state->hr_fused = hr_prev + step;
        } else if (diff < -limit) {
            state->hr_fused = hr_prev - step;
        }
    }

    /* ======================================================
     * Step 14: 输出 BPM
     * ====================================================== */
    state->hr_bpm = state->hr_fused * 60.0f;

    return state->hr_bpm;
}
