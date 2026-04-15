/**
 ******************************************************************************
 * @file    ppg_channel.h
 * @brief   PPG通道选择接口 - 统一封装MAX30101通道1和通道2的操作
 ******************************************************************************
 * @attention
 * 通过全局变量 g_ppg_channel 选择当前使用的PPG传感器通道:
 *   PPG_CH1 = 1: 使用MAX30101 (IIC1总线)
 *   PPG_CH2 = 2: 使用MAX30101_2 (IIC2总线)
 *
 * 所有PPG操作通过 PPG_xxx 前缀的包装函数调用，内部自动分发到对应通道
 ******************************************************************************
 */

#ifndef __PPG_CHANNEL_H
#define __PPG_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "MAX30101.h"
#include "MAX30101_2.h"

/* Exported types ------------------------------------------------------------*/
/* PPG通道枚举 */
typedef enum {
    PPG_CH1 = 1,  // 通道1: MAX30101 (IIC1)
    PPG_CH2 = 2   // 通道2: MAX30101_2 (IIC2)
} PPG_Channel_t;

/* Exported constants --------------------------------------------------------*/

/*
 * 当前PPG通道选择 (编译时默认值)
 * 注意: 此值已在 main.h 中统一定义, 请在 main.h 中修改
 * 此处仅作兜底默认值, main.h 中有定义时自动跳过
 */
#ifndef PPG_DEFAULT_CHANNEL
#define PPG_DEFAULT_CHANNEL  PPG_CH2
#endif

/* Exported variables --------------------------------------------------------*/
extern PPG_Channel_t g_ppg_channel;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 设置当前PPG通道
 * @param ch 目标通道 (PPG_CH1 或 PPG_CH2)
 */
void PPG_SetChannel(PPG_Channel_t ch);

/**
 * @brief 获取当前PPG通道
 * @return 当前通道
 */
PPG_Channel_t PPG_GetChannel(void);

/**
 * @brief 初始化当前通道的MAX30101传感器
 */
void PPG_Init(void);

/**
 * @brief 检测当前通道的MAX30101是否存在
 * @return 1: 存在, 0: 不存在
 */
uint8_t PPG_Check(void);

/**
 * @brief 从当前通道读取单个寄存器
 * @param addr 寄存器地址
 * @return 读取到的数据
 */
uint8_t PPG_ReadOneByte(uint16_t addr);

/**
 * @brief 向当前通道写入单个寄存器
 * @param addr 寄存器地址
 * @param data 要写入的数据
 */
void PPG_WriteOneByte(uint16_t addr, uint8_t data);

/**
 * @brief 从当前通道批量读取FIFO数据
 * @param buf 数据缓冲区
 * @param len 要读取的字节数
 */
void PPG_ReadFIFO_Burst(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __PPG_CHANNEL_H */
