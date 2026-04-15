/**
 ******************************************************************************
 * @file    sample_rate_config.h
 * @brief   PPG 采样率编译时配置
 ******************************************************************************
 * @attention
 * 通过修改 PPG_SAMPLE_RATE 宏选择目标采样率 (50/100/125 Hz).
 * 修改后重新编译烧录即可生效.
 *
 * 各模式 MAX30101 内部采样策略:
 *   50Hz  -> 内部 800sps / 16x 平均 = 50sps 有效输出
 *   100Hz -> 内部 800sps / 8x 平均  = 100sps 有效输出
 *   125Hz -> 内部 1000sps / 4x 平均 = 250sps 有效输出 (MCU 125Hz 读取)
 *
 * 约束: 脉宽固定 411us (18-bit), 内部速率不超过 1000sps.
 ******************************************************************************
 */

#ifndef __SAMPLE_RATE_CONFIG_H
#define __SAMPLE_RATE_CONFIG_H

/* ============================================================
 * 采样率选择
 * 注意: 此值已在 main.h 中统一定义, 请在 main.h 中修改
 * 此处仅作兜底默认值, main.h 中有定义时自动跳过
 * 可选值: 50, 100, 125
 * ============================================================ */
#ifndef PPG_SAMPLE_RATE
#define PPG_SAMPLE_RATE    125
#endif

/* ============================================================
 * 编译期参数校验
 * ============================================================ */
#if (PPG_SAMPLE_RATE != 50) && (PPG_SAMPLE_RATE != 100) && (PPG_SAMPLE_RATE != 125)
#error "PPG_SAMPLE_RATE must be 50, 100, or 125"
#endif

/* ============================================================
 * TIM16 定时器参数 (APB2 时钟 80MHz)
 * 频率 = 80MHz / (PSC+1) / (ARR+1)
 * ============================================================ */
#if (PPG_SAMPLE_RATE == 50)
#define TIM16_PSC_VAL      799     /* 80MHz / 800 = 100kHz */
#define TIM16_ARR_VAL      1999    /* 100kHz / 2000 = 50Hz */

#elif (PPG_SAMPLE_RATE == 100)
#define TIM16_PSC_VAL      799     /* 80MHz / 800 = 100kHz */
#define TIM16_ARR_VAL      999     /* 100kHz / 1000 = 100Hz */

#elif (PPG_SAMPLE_RATE == 125)
#define TIM16_PSC_VAL      799     /* 80MHz / 800 = 100kHz */
#define TIM16_ARR_VAL      799     /* 100kHz / 800 = 125Hz */

#endif

/* ============================================================
 * MAX30101 SPO2_CONFIG_REG (0x0A) 寄存器值
 * 布局: [7:5]=ADC_RGE(3bit) [4:2]=SR(3bit) [1:0]=LED_PW(2bit)
 * ============================================================ */
#if (PPG_SAMPLE_RATE == 50)
/* ADC=16384nA(011) | SR=800sps(100) | PW=411us(11) = 0x73 */
#define MAX30101_SPO2_CONFIG_VAL    0x73

#elif (PPG_SAMPLE_RATE == 100)
/* ADC=16384nA(011) | SR=800sps(100) | PW=411us(11) = 0x73 */
#define MAX30101_SPO2_CONFIG_VAL    0x73

#elif (PPG_SAMPLE_RATE == 125)
/* ADC=16384nA(011) | SR=1000sps(101) | PW=411us(11) = 0x77 */
#define MAX30101_SPO2_CONFIG_VAL    0x77

#endif

/* ============================================================
 * MAX30101 FIFO_CONFIG_REG (0x08) 寄存器值
 * 布局: [7:5]=SMP_AVE(3bit) [4]=ROLLOVER_EN [3:0]=A_FULL
 * ============================================================ */
#if (PPG_SAMPLE_RATE == 50)
/* SMP_AVE=16x(100) | ROLLOVER(1) | A_FULL=15(1111) = 0x9F */
#define MAX30101_FIFO_CONFIG_VAL    0x9F

#elif (PPG_SAMPLE_RATE == 100)
/* SMP_AVE=8x(011) | ROLLOVER(1) | A_FULL=15(1111) = 0x7F */
#define MAX30101_FIFO_CONFIG_VAL    0x7F

#elif (PPG_SAMPLE_RATE == 125)
/* SMP_AVE=4x(010) | ROLLOVER(1) | A_FULL=15(1111) = 0x5F */
#define MAX30101_FIFO_CONFIG_VAL    0x5F

#endif

/* ============================================================
 * 采样率信息字符串 (调试输出用)
 * ============================================================ */
#if (PPG_SAMPLE_RATE == 50)
#define PPG_RATE_STR    "50Hz (800/16x)"

#elif (PPG_SAMPLE_RATE == 100)
#define PPG_RATE_STR    "100Hz (800/8x)"

#elif (PPG_SAMPLE_RATE == 125)
#define PPG_RATE_STR    "125Hz (1000/4x)"

#endif

#endif /* __SAMPLE_RATE_CONFIG_H */
