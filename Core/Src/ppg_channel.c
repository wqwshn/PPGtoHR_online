/**
 ******************************************************************************
 * @file    ppg_channel.c
 * @brief   PPG通道选择接口实现 - 根据g_ppg_channel分发到对应驱动
 ******************************************************************************
 */

#include "ppg_channel.h"

/* 全局通道选择变量，默认使用编译时配置 */
PPG_Channel_t g_ppg_channel = PPG_DEFAULT_CHANNEL;

/**
 * @brief 设置当前PPG通道
 * @param ch 目标通道 (PPG_CH1 或 PPG_CH2)
 */
void PPG_SetChannel(PPG_Channel_t ch)
{
    g_ppg_channel = ch;
}

/**
 * @brief 获取当前PPG通道
 * @return 当前通道
 */
PPG_Channel_t PPG_GetChannel(void)
{
    return g_ppg_channel;
}

/**
 * @brief 初始化当前通道的MAX30101传感器
 */
void PPG_Init(void)
{
    if (g_ppg_channel == PPG_CH2)
        MAX30101_2_Init();
    else
        MAX30101_Init();
}

/**
 * @brief 检测当前通道的MAX30101是否存在
 * @return 1: 存在, 0: 不存在
 */
uint8_t PPG_Check(void)
{
    if (g_ppg_channel == PPG_CH2)
        return MAX2_Check();
    else
        return MAX_Check();
}

/**
 * @brief 从当前通道读取单个寄存器
 * @param addr 寄存器地址
 * @return 读取到的数据
 */
uint8_t PPG_ReadOneByte(uint16_t addr)
{
    if (g_ppg_channel == PPG_CH2)
        return MAX2_ReadOneByte(addr);
    else
        return MAX_ReadOneByte(addr);
}

/**
 * @brief 向当前通道写入单个寄存器
 * @param addr 寄存器地址
 * @param data 要写入的数据
 */
void PPG_WriteOneByte(uint16_t addr, uint8_t data)
{
    if (g_ppg_channel == PPG_CH2)
        MAX2_WriteOneByte(addr, data);
    else
        MAX_WriteOneByte(addr, data);
}

/**
 * @brief 从当前通道批量读取FIFO数据
 * @param buf 数据缓冲区
 * @param len 要读取的字节数
 */
void PPG_ReadFIFO_Burst(uint8_t *buf, uint8_t len)
{
    if (g_ppg_channel == PPG_CH2)
        MAX2_ReadFIFO_Burst(buf, len);
    else
        MAX_ReadFIFO_Burst(buf, len);
}
