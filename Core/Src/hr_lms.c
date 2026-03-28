/**
 * @file    hr_lms.c
 * @brief   LMS 自适应滤波器实现: 单级初始化/处理、多级级联处理
 * @details 基于 CMSIS-DSP 的 arm_lms_norm_f32 (归一化 LMS) 封装.
 *          多级级联时使用两个交替临时缓冲区避免读写冲突.
 */

#include "hr_lms.h"
#include <string.h>

/* ============================================================
 * LMS_Init - 初始化一个归一化 LMS 滤波器实例
 * ============================================================ */
void LMS_Init(arm_lms_norm_instance_f32 *inst,
              float *coeffs, float *state,
              uint16_t num_taps, float mu, uint32_t block_size)
{
    /* 系数清零: LMS 自适应从零开始学习 */
    memset(coeffs, 0, num_taps * sizeof(float));

    /* 状态缓冲区清零 */
    uint32_t state_len = (uint32_t)num_taps + block_size - 1U;
    memset(state, 0, state_len * sizeof(float));

    /* 调用 CMSIS-DSP 初始化 (blockSize 在处理时由 arm_lms_norm_f32 使用) */
    arm_lms_norm_init_f32(inst, num_taps, coeffs, state, mu, block_size);
}

/* ============================================================
 * LMS_Process - 执行一次 LMS 滤波
 * ============================================================ */
void LMS_Process(arm_lms_norm_instance_f32 *inst,
                 const float *ref, const float *desired,
                 float *err_out, float *tmp_out,
                 uint32_t block_len)
{
    /*
     * CMSIS-DSP arm_lms_norm_f32 参数说明:
     *   pSrc  = ref     (参考噪声信号, 如 ACC/HF)
     *   pRef  = desired (期望信号, 即 PPG)
     *   pOut  = tmp_out (滤波器输出, 本模块不使用)
     *   pErr  = err_out (误差 = desired - tmp_out, 即去噪后的信号)
     */
    arm_lms_norm_f32(inst,
                     (float *)ref,
                     (float *)desired,
                     tmp_out,
                     err_out,
                     block_len);
}

/* ============================================================
 * LMS_CascadeProcess - 级联 LMS 滤波
 *
 * 交替使用 tmp_out / tmp_out2 作为 desired 缓冲区,
 * 避免 arm_lms_norm_f32 读写同一缓冲区导致数据损坏.
 *
 *   Level 0: desired=ppg,                err -> err_out, out -> tmp_out
 *   Level 1: desired=tmp_out(err_out拷贝), err -> err_out, out -> tmp_out2
 *   Level 2: desired=tmp_out2(err_out拷贝),err -> err_out, out -> tmp_out
 *   ...
 * ============================================================ */
void LMS_CascadeProcess(arm_lms_norm_instance_f32 *instances,
                        uint8_t num_cascade,
                        float **ref_channels,
                        const float *ppg,
                        float *err_out,
                        float *tmp_out, float *tmp_out2,
                        uint32_t block_len)
{
    /* --- 第 0 级: PPG vs ref_channels[0] --- */
    LMS_Process(&instances[0],
                ref_channels[0], ppg,
                err_out, tmp_out, block_len);

    /* --- 第 1 ~ N-1 级: 级联 --- */
    for (uint8_t i = 1; i < num_cascade; i++) {
        if (i & 1U) {
            /* 奇数级: 将 err_out 拷贝到 tmp_out 作为 desired */
            memcpy(tmp_out, err_out, block_len * sizeof(float));
            LMS_Process(&instances[i],
                        ref_channels[i], tmp_out,
                        err_out, tmp_out2, block_len);
        } else {
            /* 偶数级: 将 err_out 拷贝到 tmp_out2 作为 desired */
            memcpy(tmp_out2, err_out, block_len * sizeof(float));
            LMS_Process(&instances[i],
                        ref_channels[i], tmp_out2,
                        err_out, tmp_out, block_len);
        }
    }
}
