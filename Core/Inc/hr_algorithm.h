/**
 * @file    hr_algorithm.h
 * @brief   在线心率算法主 API 头文件
 * @details 定义编译期常量、算法参数结构体、算法状态结构体和公共 API.
 *          所有数组和状态均静态分配, 禁止使用 malloc/free.
 *          基于滑动窗口 (8秒窗口, 1秒步长) 实现三路心率估计:
 *          LMS-HF, LMS-ACC, Pure-FFT, 并进行融合决策.
 */

#ifndef HR_ALGORITHM_H
#define HR_ALGORITHM_H

#include "arm_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 编译期常量
 * ============================================================ */
#define HR_FS               125     /* 采样率 125 Hz (硬件固定) */
#define HR_WIN_SEC          8       /* 窗口长度 8 秒 */
#define HR_STEP_SEC         1       /* 滑动步长 1 秒 */
#define HR_WIN_SAMPLES      (HR_FS * HR_WIN_SEC)    /* 1000 点 */
#define HR_STEP_SAMPLES     (HR_FS * HR_STEP_SEC)   /* 125 点 */
#define HR_NUM_CHANNELS     6       /* PPG, ACCx, ACCy, ACCz, HF1, HF2 */
#define HR_FFT_LEN          4096    /* FFT 点数 (实数 RFFT) */
#define HR_MAX_PEAKS        20      /* 最大候选峰值数 */
#define HR_MAX_ORDER        20      /* LMS 滤波器最大阶数 (MATLAB贝叶斯优化: 跳绳场景20) */
#define HR_LMS_CASCADE_HF   2       /* HF 路径级联数 */
#define HR_LMS_CASCADE_ACC  3       /* ACC 路径级联数 */
#define HR_HR_HISTORY_LEN   120     /* 心率历史 (120 秒 @1Hz 输出) */
#define HR_SMOOTH_WIN       5       /* 中值平滑窗口 (编译期默认值, 运行时可通过 Smooth_Win_Len 覆盖) */

/* 频率范围 (Hz) */
#define HR_FREQ_LOW         1.0f    /* 有效心率下限 60 BPM */
#define HR_FREQ_HIGH        4.0f    /* 有效心率上限 240 BPM */
#define HR_BANDPASS_LOW     0.5f    /* 带通滤波下限 */
#define HR_BANDPASS_HIGH    5.0f    /* 带通滤波上限 */

/* ACC 灵敏度 (LSM9DS1 @ ±4g 量程) */
#define HR_ACC_LSB_PER_G    8192.0f /* 16-bit ADC, ±4g: 32768/4 = 8192 LSB/g */

/* HF/ADS124S06 ADC 电压转换 (24-bit ADC, 仅取高 16 位, VREF=2.5V, Gain=1) */
#define HR_HF_LSB_TO_MV     (2.5f * 1000.0f / 32768.0f) /* 0.076294 mV/LSB */

/* ============================================================
 * 算法参数结构体 (运行时可调)
 * ============================================================ */
typedef struct {
    float   LMS_Mu_Base;            /* LMS 基础步长, 默认 0.01 */
    float   Spec_Penalty_Width;     /* 频谱惩罚宽度 (Hz), 默认 0.2 */
    float   Spec_Penalty_Weight;    /* 频谱惩罚权重, 默认 0.2 */
    float   HR_Range_Hz;            /* 运动段心率搜索范围 (Hz), 默认 0.25 */
    float   HR_Range_Rest_Hz;       /* 静息段心率搜索范围 (Hz) */
    float   Slew_Limit_BPM;         /* 运动段心率变化率限制 (BPM/s) */
    float   Slew_Step_BPM;          /* 运动段心率步进 (BPM) */
    float   Slew_Limit_Rest_BPM;    /* 静息段心率变化率限制 (BPM/s) */
    float   Slew_Step_Rest_BPM;     /* 静息段心率步进 (BPM) */
    float   Motion_Th_Scale;        /* 运动阈值倍数, 默认 3.0 */
    float   Default_Motion_Th;      /* 默认运动阈值 (g 值, 默认 0.07g) */
    uint8_t Spec_Penalty_Enable;    /* 是否启用频谱惩罚 */
    uint8_t Smooth_Win_Len;         /* 中值平滑窗口长度 (奇数), 默认 9 (MATLAB贝叶斯优化) */
} HR_Config_t;

/* ============================================================
 * 算法状态结构体 (全局唯一实例, 静态分配)
 * ============================================================ */
typedef struct {
    /* --- 1 秒采集缓冲区 (前台 ISR 写入) --- */
    float   buf_1s_ppg[HR_STEP_SAMPLES];
    float   buf_1s_accx[HR_STEP_SAMPLES];
    float   buf_1s_accy[HR_STEP_SAMPLES];
    float   buf_1s_accz[HR_STEP_SAMPLES];
    float   buf_1s_hf[HR_STEP_SAMPLES];
    float   buf_1s_hf2[HR_STEP_SAMPLES];
    volatile uint16_t sample_idx;   /* 当前 1 秒内采样索引 */
    volatile uint8_t  flag_1s_ready;/* 1 秒数据就绪标志 */

    /* --- 8 秒滑动窗口 - 原始数据 (后台算法使用) --- */
    float   win_ppg[HR_WIN_SAMPLES];
    float   win_accx[HR_WIN_SAMPLES];
    float   win_accy[HR_WIN_SAMPLES];
    float   win_accz[HR_WIN_SAMPLES];
    float   win_hf[HR_WIN_SAMPLES];
    float   win_hf2[HR_WIN_SAMPLES];
    uint8_t win_filled;             /* 窗口是否已填满至少一次 */

    /* --- 8 秒滑动窗口 - IIR 滤波后数据 (仅追加新滤波数据 + 平移) --- */
    float   filt_ppg[HR_WIN_SAMPLES];
    float   filt_accx[HR_WIN_SAMPLES];
    float   filt_accy[HR_WIN_SAMPLES];
    float   filt_accz[HR_WIN_SAMPLES];
    float   filt_hf[HR_WIN_SAMPLES];
    float   filt_hf2[HR_WIN_SAMPLES];

    /* --- IIR 带通滤波器状态 (5 通道 x 4 biquad 节, 流式) --- */
    arm_biquad_casd_df1_inst_f32 biquad_ppg;
    arm_biquad_casd_df1_inst_f32 biquad_accx;
    arm_biquad_casd_df1_inst_f32 biquad_accy;
    arm_biquad_casd_df1_inst_f32 biquad_accz;
    arm_biquad_casd_df1_inst_f32 biquad_hf;
    arm_biquad_casd_df1_inst_f32 biquad_hf2;
    float   iir_state_ppg[16];     /* 4 sections x 4 state */
    float   iir_state_accx[16];
    float   iir_state_accy[16];
    float   iir_state_accz[16];
    float   iir_state_hf[16];
    float   iir_state_hf2[16];
    float   iir_filt_1s_ppg[HR_STEP_SAMPLES];   /* 1 秒滤波临时输出 */
    float   iir_filt_1s_accx[HR_STEP_SAMPLES];
    float   iir_filt_1s_accy[HR_STEP_SAMPLES];
    float   iir_filt_1s_accz[HR_STEP_SAMPLES];
    float   iir_filt_1s_hf[HR_STEP_SAMPLES];
    float   iir_filt_1s_hf2[HR_STEP_SAMPLES];

    /* --- FFT 工作缓冲区 --- */
    float   fft_input[HR_FFT_LEN];
    float   fft_output[HR_FFT_LEN]; /* RFFT 输出 (complex interleaved) */

    /* --- LMS 状态 --- */
    /* HF 路径: 2 级级联 */
    arm_lms_norm_instance_f32 lms_hf[HR_LMS_CASCADE_HF];
    float   lms_hf_coeffs[HR_LMS_CASCADE_HF][HR_MAX_ORDER];
    float   lms_hf_state[HR_LMS_CASCADE_HF][HR_MAX_ORDER + HR_WIN_SAMPLES - 1];
    float   lms_hf_err[HR_WIN_SAMPLES];
    float   lms_hf_tmp[HR_WIN_SAMPLES];
    float   lms_hf_tmp2[HR_WIN_SAMPLES]; /* 级联第二临时缓冲区 */

    /* ACC 路径: 3 级级联 */
    arm_lms_norm_instance_f32 lms_acc[HR_LMS_CASCADE_ACC];
    float   lms_acc_coeffs[HR_LMS_CASCADE_ACC][HR_MAX_ORDER];
    float   lms_acc_state[HR_LMS_CASCADE_ACC][HR_MAX_ORDER + HR_WIN_SAMPLES - 1];
    float   lms_acc_err[HR_WIN_SAMPLES];
    float   lms_acc_tmp[HR_WIN_SAMPLES];
    float   lms_acc_tmp2[HR_WIN_SAMPLES]; /* 级联第二临时缓冲区 */

    /* --- 运动检测 --- */
    float   motion_threshold;       /* ACC 幅值标准差阈值 */
    uint8_t motion_calibrated;      /* 校准完成标志 */
    float   acc_baseline_std;       /* 静息段 ACC 标准差基线 */
    uint16_t calib_count;           /* 校准窗口计数器 (0~HR_CALIB_WINDOWS) */
    float   calib_std_accum;        /* 校准期 std 累积器 */
    uint16_t calib_windows_done;    /* 已完成的校准窗口数 */

    /* --- Hamming 窗 (预计算) --- */
    float   hamming_win[HR_WIN_SAMPLES];

    /* --- 通用 scratch 缓冲区 (zscore 归一化等) --- */
    float   scratch_a[HR_WIN_SAMPLES];
    float   scratch_b[HR_WIN_SAMPLES];

    /* --- 心率输出 --- */
    float   hr_lms_hf;              /* 当前 LMS-HF 路径心率 (Hz) */
    float   hr_lms_acc;             /* 当前 LMS-ACC 路径心率 (Hz) */
    float   hr_fft;                 /* 当前 FFT 路径心率 (Hz) */
    float   hr_fused;               /* 融合心率 (Hz) */
    float   hr_bpm;                 /* 融合心率 (BPM) */
    uint8_t is_motion;              /* 当前运动标志 */

    /* --- 心率历史追踪 --- */
    float   hr_history_lms_hf[HR_HR_HISTORY_LEN];
    float   hr_history_lms_acc[HR_HR_HISTORY_LEN];
    float   hr_history_fft[HR_HR_HISTORY_LEN];
    uint16_t hr_history_idx;        /* 当前历史索引 */

    /* --- 峰值检测临时缓冲 --- */
    float   peak_freqs[HR_MAX_PEAKS];
    float   peak_amps[HR_MAX_PEAKS];
    uint16_t num_peaks;

    /* --- 窗口计数器 --- */
    uint16_t win_count;             /* 已处理的窗口步数 */

    /* --- LMS 收敛性保护 --- */
    uint16_t prev_order_hf;         /* 上一次 HF 路径 LMS 阶数 */
    uint16_t prev_order_acc;        /* 上一次 ACC 路径 LMS 阶数 */

    /* --- 信号质量评估 (调试用) --- */
    float   hf1_signal_std;         /* HF1(桥顶1) AC 幅值 (BPF 后标准差, LSB) */
    float   hf2_signal_std;         /* HF2(桥顶2) AC 幅值 (BPF 后标准差, LSB) */
    float   hf1_ppg_corr;           /* HF1-PPG Pearson 相关系数 (-1~+1) */
    float   hf2_ppg_corr;           /* HF2-PPG Pearson 相关系数 (-1~+1) */
    float   acc_ppg_corr;           /* ACC-PPG Pearson 相关系数 (-1~+1, 最优轴) */
} HR_State_t;

/* ============================================================
 * 公共 API
 * ============================================================ */

/**
 * @brief 初始化算法状态 (必须在 main 初始化阶段调用)
 * @param config: 算法参数, 传 NULL 使用默认值
 * @param state:  算法状态结构体指针
 */
void HR_Init(const HR_Config_t *config, HR_State_t *state);

/**
 * @brief 向 1 秒缓冲区追加一个采样点 (由 ISR 或 ISR 触发的采集逻辑调用)
 * @param state:  算法状态
 * @param ppg:    PPG 采样值 (float)
 * @param accx:   ACC X 采样值
 * @param accy:   ACC Y 采样值
 * @param accz:   ACC Z 采样值
 * @param hf1:    HF1/ADC 采样值 (桥顶1)
 * @param hf2:    HF2/ADC 采样值 (桥顶2)
 */
void HR_PushSample(HR_State_t *state,
                   float ppg, float accx, float accy, float accz,
                   float hf1, float hf2);

/**
 * @brief 执行一次完整的 8 秒窗口心率解算 (由主循环调用)
 * @param state:  算法状态
 * @return 融合心率值 (BPM), 0 表示窗口未填满或错误
 */
float HR_RunSolver(HR_State_t *state);

/**
 * @brief 获取默认配置参数
 * @param config: 输出配置
 */
void HR_GetDefaultConfig(HR_Config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGORITHM_H */
