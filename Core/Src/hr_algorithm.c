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
    config->Spec_Penalty_Width = 0.15f;      /* 惩罚带宽 0.15Hz (9 BPM) */
    config->Spec_Penalty_Weight= 0.1f;       /* 惩罚权重: 降至 10% (原 20%) */
    config->HR_Range_Hz        = 0.3333f;    /* MATLAB贝叶斯优化: 跳绳 0.3333 */
    config->HR_Range_Rest_Hz   = 0.6667f;    /* MATLAB贝叶斯优化: 跳绳 0.6667 */
    config->Slew_Limit_BPM     = 14.0f;      /* MATLAB贝叶斯优化: 跳绳 14 */
    config->Slew_Step_BPM      = 5.0f;       /* MATLAB贝叶斯优化: 跳绳 5 */
    config->Slew_Limit_Rest_BPM= 8.0f;       /* MATLAB贝叶斯优化: 跳绳 8 */
    config->Slew_Step_Rest_BPM = 5.0f;       /* MATLAB贝叶斯优化: 跳绳 5 */
    config->Motion_Th_Scale    = 3.0f;
    config->Default_Motion_Th  = 0.07f;      /* 基于实测: 静息 0.06-0.07g */
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

    /* --- 初始化 6 个 IIR biquad 滤波器实例 --- */
    /* 每个通道: 4 个 biquad 节, 使用预计算的 HR_BPF_COEFFS 系数 */
    arm_biquad_cascade_df1_init_f32(&state->biquad_ppg,  4, (float *)HR_BPF_COEFFS, state->iir_state_ppg);
    arm_biquad_cascade_df1_init_f32(&state->biquad_accx, 4, (float *)HR_BPF_COEFFS, state->iir_state_accx);
    arm_biquad_cascade_df1_init_f32(&state->biquad_accy, 4, (float *)HR_BPF_COEFFS, state->iir_state_accy);
    arm_biquad_cascade_df1_init_f32(&state->biquad_accz, 4, (float *)HR_BPF_COEFFS, state->iir_state_accz);
    arm_biquad_cascade_df1_init_f32(&state->biquad_hf,   4, (float *)HR_BPF_COEFFS, state->iir_state_hf);
    arm_biquad_cascade_df1_init_f32(&state->biquad_hf2,  4, (float *)HR_BPF_COEFFS, state->iir_state_hf2);

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

    /* --- 运动阈值初始化 (g 值转换为 LSB) --- */
    state->motion_threshold = s_config.Default_Motion_Th * HR_ACC_LSB_PER_G;
}

/* ============================================================
 * HR_PushSample - 向 1 秒缓冲区追加一个采样点
 * ============================================================
 * 由 ISR 或采集逻辑调用, 采样率 125 Hz, 累积 125 点后置就绪标志.
 * RunSolver 消费数据后重置索引, PushSample 不主动重置.
 */
void HR_PushSample(HR_State_t *state,
                   float ppg, float accx, float accy, float accz,
                   float hf1, float hf2)
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
    state->buf_1s_hf[state->sample_idx]   = hf1;
    state->buf_1s_hf2[state->sample_idx]  = hf2;

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

    /* HF1 (桥顶1) */
    memmove(state->win_hf,
            state->win_hf + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->win_hf + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->buf_1s_hf,
           HR_STEP_SAMPLES * sizeof(float));

    /* HF2 (桥顶2) */
    memmove(state->win_hf2,
            state->win_hf2 + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->win_hf2 + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->buf_1s_hf2,
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

    arm_biquad_cascade_df1_f32(&state->biquad_hf2,
                               state->buf_1s_hf2,
                               state->iir_filt_1s_hf2,
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

    memmove(state->filt_hf2,
            state->filt_hf2 + HR_STEP_SAMPLES,
            (HR_WIN_SAMPLES - HR_STEP_SAMPLES) * sizeof(float));
    memcpy(state->filt_hf2 + (HR_WIN_SAMPLES - HR_STEP_SAMPLES),
           state->iir_filt_1s_hf2,
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
        /* scratch_a = sqrt(scratch_a) = 幅值 (输入恒 >= 0: ax^2+ay^2+az^2) */
        {
            uint16_t _si;
            for (_si = 0; _si < HR_WIN_SAMPLES; _si++) {
                state->scratch_a[_si] = sqrtf(state->scratch_a[_si]);
            }
        }

        /* 计算幅值的标准差 (LSB 单位) */
        arm_std_f32(state->scratch_a, HR_WIN_SAMPLES, &acc_std);

        /* 自适应校准逻辑 */
        if (!state->motion_calibrated) {
            /* 将默认阈值 (g 值) 转换为 LSB 进行比较 */
            float motion_th_lsb = s_config.Default_Motion_Th * HR_ACC_LSB_PER_G;
            if (acc_std < motion_th_lsb) {
                /* 静息窗口: 累积标准差用于计算基线 */
                state->calib_std_accum += acc_std;
                state->calib_windows_done++;

                if (state->calib_windows_done >= 8) {
                    /* 计算平均静息标准差作为基线 (LSB) */
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
     * Step 5b: HF 信号质量 - AC 幅值 (BPF 后标准差)
     * ======================================================
     * 计算双路 HF 桥顶信号的交流幅值, 反映传感器噪声底噪或信号活跃度.
     * 必须在 Step 8 就地 zscore 之前完成.
     */
    arm_std_f32(state->filt_hf, HR_WIN_SAMPLES, &state->hf1_signal_std);
    arm_std_f32(state->filt_hf2, HR_WIN_SAMPLES, &state->hf2_signal_std);

    /* ======================================================
     * Step 6: 窗口填充检查
     * ======================================================
     * 需要至少 8 个 1 秒步长 (8 秒窗口) 才能开始计算.
     */
    state->win_count++;
    if (state->win_count < HR_WIN_SEC) {
        state->prev_is_motion = state->is_motion;
        return 0.0f;
    }
    state->win_filled = 1;

    /* ======================================================
     * Step 7: 时延对齐 + 相关性排序
     * ======================================================
     * 对 5 个参考通道 (HF1, HF2, ACCx, ACCy, ACCz) 计算与 PPG 的时延和相关度.
     * HF 通道按相关性排序选择级联顺序; ACC 三轴按相关性排序选择级联顺序.
     * 根据最优通道的时延计算 LMS 滤波器阶数.
     */
    {
        float corr_accx, corr_accy, corr_accz;
        float corr_hf1, corr_hf2;
        int16_t delay_accx, delay_accy, delay_accz;
        int16_t delay_hf1, delay_hf2;
        uint16_t order_hf, order_acc;

        /* 排序用结构: 参考信号指针 + 相关性绝对值 + 时延 */
        typedef struct { float *ref; float abs_corr; int16_t delay; } ref_rank_t;
        ref_rank_t hf_rank[2], acc_rank[3];
        uint8_t i;

        /* HF1 (桥顶1) 时延搜索 */
        delay_hf1 = DSP_FindDelay(state->filt_ppg, state->filt_hf,
                                  HR_WIN_SAMPLES, state->scratch_a, &corr_hf1);
        /* HF2 (桥顶2) 时延搜索 */
        delay_hf2 = DSP_FindDelay(state->filt_ppg, state->filt_hf2,
                                  HR_WIN_SAMPLES, state->scratch_a, &corr_hf2);
        /* ACC X 时延搜索 */
        delay_accx = DSP_FindDelay(state->filt_ppg, state->filt_accx,
                                   HR_WIN_SAMPLES, state->scratch_a, &corr_accx);
        /* ACC Y 时延搜索 */
        delay_accy = DSP_FindDelay(state->filt_ppg, state->filt_accy,
                                   HR_WIN_SAMPLES, state->scratch_a, &corr_accy);
        /* ACC Z 时延搜索 */
        delay_accz = DSP_FindDelay(state->filt_ppg, state->filt_accz,
                                   HR_WIN_SAMPLES, state->scratch_a, &corr_accz);

        /* --- HF 按相关性排序 (降序, 用于 2 级级联) --- */
        hf_rank[0].ref = state->filt_hf;   hf_rank[0].abs_corr = fabsf(corr_hf1); hf_rank[0].delay = delay_hf1;
        hf_rank[1].ref = state->filt_hf2;  hf_rank[1].abs_corr = fabsf(corr_hf2); hf_rank[1].delay = delay_hf2;
        if (hf_rank[0].abs_corr < hf_rank[1].abs_corr) {
            ref_rank_t tmp = hf_rank[0];
            hf_rank[0] = hf_rank[1];
            hf_rank[1] = tmp;
        }

        /* --- ACC 三轴按相关性排序 (降序, 用于 3 级级联) --- */
        acc_rank[0].ref = state->filt_accx; acc_rank[0].abs_corr = fabsf(corr_accx); acc_rank[0].delay = delay_accx;
        acc_rank[1].ref = state->filt_accy; acc_rank[1].abs_corr = fabsf(corr_accy); acc_rank[1].delay = delay_accy;
        acc_rank[2].ref = state->filt_accz; acc_rank[2].abs_corr = fabsf(corr_accz); acc_rank[2].delay = delay_accz;
        /* 插入排序 (3 个元素) */
        for (i = 1; i < 3; i++) {
            ref_rank_t key = acc_rank[i];
            int8_t j = (int8_t)i - 1;
            while (j >= 0 && acc_rank[j].abs_corr < key.abs_corr) {
                acc_rank[j + 1] = acc_rank[j];
                j--;
            }
            acc_rank[j + 1] = key;
        }

        /* --- LMS 阶数计算 --- */
        /* HF: 使用相关性最高的通道的时延 */
        {
            int16_t best_delay_hf = hf_rank[0].delay;
            if (best_delay_hf < 0) {
                order_hf = (uint16_t)(-best_delay_hf * 1.0f);
                if (order_hf < 1) order_hf = 1;
                if (order_hf > HR_MAX_ORDER) order_hf = HR_MAX_ORDER;
            } else {
                order_hf = HR_MAX_ORDER;
            }
        }

        /* ACC: 使用相关性最高的轴的时延 (1.5 倍因子) */
        {
            int16_t best_delay_acc = acc_rank[0].delay;
            if (best_delay_acc < 0) {
                order_acc = (uint16_t)(-best_delay_acc * 1.5f);
                if (order_acc < 1) order_acc = 1;
                if (order_acc > HR_MAX_ORDER) order_acc = HR_MAX_ORDER;
            } else {
                order_acc = HR_MAX_ORDER;
            }
        }

        /* 条件性 LMS 初始化: 首次运行 / 阶数变化 / 静息->运动切换 时重置 */
        {
            uint8_t motion_transition = state->is_motion && !state->prev_is_motion;
            uint8_t need_reinit_hf = (state->win_count == HR_WIN_SEC) ||
                                     (order_hf != state->prev_order_hf) ||
                                     motion_transition;
            uint8_t need_reinit_acc = (state->win_count == HR_WIN_SEC) ||
                                      (order_acc != state->prev_order_acc) ||
                                      motion_transition;

            if (need_reinit_hf) {
                for (i = 0; i < HR_LMS_CASCADE_HF; i++) {
                    LMS_Init(&state->lms_hf[i],
                             state->lms_hf_coeffs[i],
                             state->lms_hf_state[i],
                             order_hf,
                             s_config.LMS_Mu_Base,
                             HR_WIN_SAMPLES);
                }
                state->prev_order_hf = order_hf;
            }

            if (need_reinit_acc) {
                for (i = 0; i < HR_LMS_CASCADE_ACC; i++) {
                    LMS_Init(&state->lms_acc[i],
                             state->lms_acc_coeffs[i],
                             state->lms_acc_state[i],
                             order_acc,
                             s_config.LMS_Mu_Base,
                             HR_WIN_SAMPLES);
                }
                state->prev_order_acc = order_acc;
            }
        }

        /* ======================================================
         * Step 7b: Pearson 相关系数归一化 (信号质量评估)
         * ====================================================== */
        {
            float ppg_std, ref_std;

            arm_std_f32(state->filt_ppg, HR_WIN_SAMPLES, &ppg_std);

            /* HF1-PPG */
            ref_std = state->hf1_signal_std;
            { float norm = (float)HR_WIN_SAMPLES * ppg_std * ref_std;
              state->hf1_ppg_corr = (norm > 1e-6f) ? (corr_hf1 / norm) : 0.0f; }

            /* HF2-PPG */
            ref_std = state->hf2_signal_std;
            { float norm = (float)HR_WIN_SAMPLES * ppg_std * ref_std;
              state->hf2_ppg_corr = (norm > 1e-6f) ? (corr_hf2 / norm) : 0.0f; }

            /* ACC-PPG (最优轴) */
            arm_std_f32(acc_rank[0].ref, HR_WIN_SAMPLES, &ref_std);
            { float norm = (float)HR_WIN_SAMPLES * ppg_std * ref_std;
              float best_corr = (acc_rank[0].ref == state->filt_accx) ? corr_accx :
                                (acc_rank[0].ref == state->filt_accy) ? corr_accy : corr_accz;
              state->acc_ppg_corr = (norm > 1e-6f) ? (best_corr / norm) : 0.0f; }
        }

        /* ======================================================
         * Step 8: Path A - LMS-HF 路径 (双路桥顶, 相关性排序级联)
         * ======================================================
         * HF1(桥顶1) 和 HF2(桥顶2) 按相关性排序后逐级级联 LMS.
         * 每级使用动态步长: mu = max(0.001, Mu_Base - |corr|/100).
         * 参考信号就地 zscore (filt_hf/filt_hf2 在此之后不再需要原始值).
         */
        {
            float motion_freq_hf;
            uint16_t num_peaks_hf;

            /* 就地 zscore 双路 HF 参考信号 */
            zscore_inplace(state->filt_hf, HR_WIN_SAMPLES);
            zscore_inplace(state->filt_hf2, HR_WIN_SAMPLES);

            /* PPG 拷贝到 scratch_a 并 zscore (必须拷贝, filt_ppg 留给 FFT) */
            memcpy(state->scratch_a, state->filt_ppg, HR_WIN_SAMPLES * sizeof(float));
            zscore_inplace(state->scratch_a, HR_WIN_SAMPLES);

            /* 逐级 LMS: 每级使用排序后的 HF 参考和动态步长 */
            for (i = 0; i < HR_LMS_CASCADE_HF; i++) {
                float mu_stage = s_config.LMS_Mu_Base - hf_rank[i].abs_corr / 100.0f;
                if (mu_stage < 0.001f) mu_stage = 0.001f;
                state->lms_hf[i].mu = mu_stage;

                if (i == 0) {
                    /* 第 0 级: desired=PPG_zscore, ref=HF_sorted[0] */
                    LMS_Process(&state->lms_hf[0],
                                hf_rank[0].ref, state->scratch_a,
                                state->lms_hf_err, state->lms_hf_tmp,
                                HR_WIN_SAMPLES);
                } else {
                    /* 后续级: desired=上一级误差输出, ref=HF_sorted[i] */
                    memcpy(state->scratch_a, state->lms_hf_err,
                           HR_WIN_SAMPLES * sizeof(float));
                    LMS_Process(&state->lms_hf[i],
                                hf_rank[i].ref, state->scratch_a,
                                state->lms_hf_err,
                                (i & 1U) ? state->lms_hf_tmp2 : state->lms_hf_tmp,
                                HR_WIN_SAMPLES);
                }
            }

            /* 从最优 HF 参考信号 FFT 获取运动主频 */
            motion_freq_hf = 0.0f;
            if (s_config.Spec_Penalty_Enable) {
                uint16_t num_ref_peaks;
                DSP_FFTPeaks(hf_rank[0].ref, HR_WIN_SAMPLES,
                             state->fft_input, state->fft_output,
                             (float)HR_FS, 0.3f,
                             state->peak_freqs, state->peak_amps,
                             HR_MAX_PEAKS, &num_ref_peaks);
                if (num_ref_peaks > 0) {
                    motion_freq_hf = state->peak_freqs[0];
                }
            }

            /* 对 LMS-HF 误差输出做 FFT 峰值检测 */
            DSP_FFTPeaks(state->lms_hf_err, HR_WIN_SAMPLES,
                         state->fft_input, state->fft_output,
                         (float)HR_FS, 0.3f,
                         state->peak_freqs, state->peak_amps,
                         HR_MAX_PEAKS, &num_peaks_hf);

            if (state->is_motion && motion_freq_hf > 0.0f && num_peaks_hf > 0) {
                DSP_SpectrumPenalty(state->peak_freqs, state->peak_amps,
                                   num_peaks_hf, motion_freq_hf,
                                   s_config.Spec_Penalty_Width,
                                   s_config.Spec_Penalty_Weight);
            }

            DSP_SortPeaksByAmp(state->peak_freqs, state->peak_amps, num_peaks_hf);

            state->hr_lms_hf = DSP_TrackHR(state->peak_freqs, num_peaks_hf,
                                            state->hr_lms_hf,
                                            s_config.HR_Range_Hz,
                                            -s_config.HR_Range_Hz);
        }

        /* ======================================================
         * Step 9: Path B - LMS-ACC 路径 (三轴排序级联)
         * ======================================================
         * ACC 三轴按相关性排序后逐级级联 LMS, 每级使用不同轴作为参考.
         * 就地 zscore filt_accx/y/z (后续仅 Step 10 FFT 需读 filt_accz, 不影响).
         */
        {
            float motion_freq_acc;
            uint16_t num_peaks_acc;

            /* 就地 zscore 三轴 ACC 参考信号 */
            zscore_inplace(state->filt_accx, HR_WIN_SAMPLES);
            zscore_inplace(state->filt_accy, HR_WIN_SAMPLES);
            zscore_inplace(state->filt_accz, HR_WIN_SAMPLES);

            /* PPG 重新拷贝到 scratch_a 并 zscore */
            memcpy(state->scratch_a, state->filt_ppg, HR_WIN_SAMPLES * sizeof(float));
            zscore_inplace(state->scratch_a, HR_WIN_SAMPLES);

            /* 逐级 LMS: 每级使用排序后的 ACC 轴和动态步长 */
            for (i = 0; i < HR_LMS_CASCADE_ACC; i++) {
                float mu_stage = s_config.LMS_Mu_Base - acc_rank[i].abs_corr / 100.0f;
                if (mu_stage < 0.001f) mu_stage = 0.001f;
                state->lms_acc[i].mu = mu_stage;

                if (i == 0) {
                    LMS_Process(&state->lms_acc[0],
                                acc_rank[0].ref, state->scratch_a,
                                state->lms_acc_err, state->lms_acc_tmp,
                                HR_WIN_SAMPLES);
                } else {
                    memcpy(state->scratch_a, state->lms_acc_err,
                           HR_WIN_SAMPLES * sizeof(float));
                    LMS_Process(&state->lms_acc[i],
                                acc_rank[i].ref, state->scratch_a,
                                state->lms_acc_err,
                                (i & 1U) ? state->lms_acc_tmp2 : state->lms_acc_tmp,
                                HR_WIN_SAMPLES);
                }
            }

            /* 从 ACC-Z (或最优轴) FFT 获取运动主频 */
            motion_freq_acc = 0.0f;
            if (s_config.Spec_Penalty_Enable) {
                uint16_t num_ref_peaks;
                DSP_FFTPeaks(acc_rank[0].ref, HR_WIN_SAMPLES,
                             state->fft_input, state->fft_output,
                             (float)HR_FS, 0.3f,
                             state->peak_freqs, state->peak_amps,
                             HR_MAX_PEAKS, &num_ref_peaks);
                if (num_ref_peaks > 0) {
                    motion_freq_acc = state->peak_freqs[0];
                }
            }

            DSP_FFTPeaks(state->lms_acc_err, HR_WIN_SAMPLES,
                         state->fft_input, state->fft_output,
                         (float)HR_FS, 0.3f,
                         state->peak_freqs, state->peak_amps,
                         HR_MAX_PEAKS, &num_peaks_acc);

            if (state->is_motion && motion_freq_acc > 0.0f && num_peaks_acc > 0) {
                DSP_SpectrumPenalty(state->peak_freqs, state->peak_amps,
                                   num_peaks_acc, motion_freq_acc,
                                   s_config.Spec_Penalty_Width,
                                   s_config.Spec_Penalty_Weight);
            }

            DSP_SortPeaksByAmp(state->peak_freqs, state->peak_amps, num_peaks_acc);

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
     * 注: filt_accz 已被 Step 9 就地 zscore, 但频率信息完整, 不影响频谱惩罚.
     */
    {
        float mean_val;
        float motion_freq_fft;
        uint16_t num_peaks_fft;

        /* 将滤波后 PPG 拷贝到 scratch_a (作为 FFT 输入源) */
        memcpy(state->scratch_a, state->filt_ppg, HR_WIN_SAMPLES * sizeof(float));

        /* 去均值 */
        arm_mean_f32(state->scratch_a, HR_WIN_SAMPLES, &mean_val);
        arm_offset_f32(state->scratch_a, -mean_val, state->scratch_a, HR_WIN_SAMPLES);

        /* 应用 Hamming 窗 */
        arm_mult_f32(state->scratch_a, state->hamming_win, state->scratch_a, HR_WIN_SAMPLES);

        /* 先从参考信号 FFT 获取运动主频 (使用 ACC Z 轴, 已 zscore 但频率信息完整) */
        motion_freq_fft = 0.0f;
        if (s_config.Spec_Penalty_Enable) {
            uint16_t num_ref_peaks;
            DSP_FFTPeaks(state->filt_accz, HR_WIN_SAMPLES,
                         state->fft_input, state->fft_output,
                         (float)HR_FS, 0.3f,
                         state->peak_freqs, state->peak_amps,
                         HR_MAX_PEAKS, &num_ref_peaks);
            if (num_ref_peaks > 0) {
                motion_freq_fft = state->peak_freqs[0];
            }
        }

        /* 对 PPG 信号做 FFT 峰值检测 */
        DSP_FFTPeaks(state->scratch_a, HR_WIN_SAMPLES,
                     state->fft_input, state->fft_output,
                     (float)HR_FS, 0.3f,
                     state->peak_freqs, state->peak_amps,
                     HR_MAX_PEAKS, &num_peaks_fft);

        if (state->is_motion && motion_freq_fft > 0.0f && num_peaks_fft > 0) {
            DSP_SpectrumPenalty(state->peak_freqs, state->peak_amps,
                               num_peaks_fft, motion_freq_fft,
                               s_config.Spec_Penalty_Width,
                               s_config.Spec_Penalty_Weight);
        }

        DSP_SortPeaksByAmp(state->peak_freqs, state->peak_amps, num_peaks_fft);

        state->hr_fft = DSP_TrackHR(state->peak_freqs, num_peaks_fft,
                                     state->hr_fft,
                                     s_config.HR_Range_Rest_Hz,
                                     -s_config.HR_Range_Rest_Hz);
    }

    /* ======================================================
     * Step 11: 融合决策
     * ======================================================
     * 运动状态: 选择 LMS-HF 路径 (HF 桥顶信号 Ut1/Ut2 作为参考,
     *           自适应滤波消除运动伪影效果最优)
     * 静息状态: 选择 Pure-FFT 路径 (信号干净, FFT 精度高)
     *
     * 注: LMS-ACC 和 FFT 路径始终计算并保留, 作为底线对比参考.
     */
    if (state->is_motion) {
        state->hr_fused = state->hr_lms_hf;
    } else {
        state->hr_fused = state->hr_fft;
    }

    /* ======================================================
     * Step 12: 中值平滑 + 历史更新
     * ======================================================
     * 对心率历史做中值滤波平滑, 抑制突变.
     * 将当前融合心率写入环形历史缓冲区.
     * 平滑时跟随融合决策: 运动段平滑 HF 历史, 静息段平滑 FFT 历史.
     * 注意: 环形缓冲区需线性化后传入 DSP_MedianSmooth.
     */
    {
        float smoothed;
        uint16_t hist_len;
        const float *target_hist;
        float recent_buf[9];
        uint16_t actual_len;
        uint16_t base;
        uint16_t k;

        /* 将三路心率写入历史 (环形缓冲区, 始终记录) */
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

        /* 选择融合路径对应的历史缓冲区 */
        target_hist = state->is_motion ? state->hr_history_lms_hf
                                       : state->hr_history_fft;

        /* 从环形缓冲区提取最近 actual_len 个值到线性数组 */
        actual_len = (hist_len < HR_SMOOTH_WIN) ? hist_len : HR_SMOOTH_WIN;
        if (actual_len > 9) { actual_len = 9; }

        if (actual_len == 0) {
            smoothed = 0.0f;
        } else {
            base = (state->hr_history_idx + HR_HR_HISTORY_LEN - actual_len)
                   % HR_HR_HISTORY_LEN;
            for (k = 0; k < actual_len; k++) {
                recent_buf[k] = target_hist[(base + k) % HR_HR_HISTORY_LEN];
            }
            DSP_MedianSmooth(recent_buf, actual_len, actual_len, &smoothed);
        }

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

    state->prev_is_motion = state->is_motion;
    return state->hr_bpm;
}
