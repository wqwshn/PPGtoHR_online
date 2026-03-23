/**
 ******************************************************************************
 * @file    MAX30101.h
 * @brief   MAX30101 PPG传感器驱动头文件 (支持心率/血氧双模式)
 ******************************************************************************
 * @attention
 * MAX30101 寄存器地址定义和函数声明
 * 根据数据手册定义，包含温度相关寄存器
 ******************************************************************************
 */

#ifndef __MAX30101_H
#define __MAX30101_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "MyIIC.h"  // IIC通信驱动 (注意大小写，根据项目实际头文件名调整)

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
// MAX30101 寄存器地址定义
#define INTERRUPT_REG                  0x00    // 中断状态寄存器1
#define INTERRUPT_ENABLE_1_REG         0x02    // 中断使能寄存器1
#define INTERRUPT_ENABLE_2_REG         0x03    // 中断使能寄存器2 (包含温度中断使能)
#define FIFO_WR_PTR_REG                0x04    // FIFO写指针
#define OVF_COUNTER_REG                0x05    // 溢出计数器
#define FIFO_RD_PTR_REG                0x06    // FIFO读指针 (兼容: RD_PTR_REG)
#define FIFO_DATA_REG                  0x07    // FIFO数据寄存器 (兼容: FIF0_DATA_REG)
#define FIFO_CONFIG_REG                0x08    // FIFO配置寄存器
#define MODE_CONFIG_REG                0x09    // 模式配置寄存器
#define SPO2_CONFIG_REG                0x0A    // SpO2配置寄存器
#define LED1_PA_REG                    0x0C    // LED1脉冲幅度 (Red)
#define LED2_PA_REG                    0x0D    // LED2脉冲幅度 (IR)
#define LED3_PA_REG                    0x0E    // LED3脉冲幅度 (Green)
#define LED4_PA_REG                    0x0F    // LED4脉冲幅度
#define LED_CONTROL1                   0x11    // LED控制寄存器1 (Multi-LED模式)
#define LED_CONTROL2                   0x12    // LED控制寄存器2 (Multi-LED模式)
#define DIE_TEMP_INT_REG               0x1F    // 芯片温度整数部分
#define DIE_TEMP_FRAC_REG              0x20    // 芯片温度小数部分
#define DIE_TEMP_CONFIG_REG            0x21    // 温度配置寄存器

// 兼容性定义
#define RD_PTR_REG                     FIFO_RD_PTR_REG
#define FIF0_DATA_REG                  FIFO_DATA_REG

// INTERRUPT_REG 位定义
#define INTERRUPT_REG_A_FULL           (0x01 << 7)
#define INTERRUPT_REG_PPG_RDY          (0x01 << 6)
#define INTERRUPT_REG_ALC_OVF          (0x01 << 5)
#define INTERRUPT_REG_PWR_RDY          (0x01 << 0)

// INTERRUPT_ENABLE_1_REG 位定义
#define INTERRUPT_ENABLE_REG_A_FULL_EN         (0x01 << 7)
#define INTERRUPT_ENABLE_REG_PPG_RDY_EN        (0x01 << 6)
#define INTERRUPT_ENABLE_REG_ALC_OVF_EN        (0x01 << 5)

// INTERRUPT_ENABLE_2_REG 位定义 (温度中断)
#define INTERRUPT_DIE_TEMP_REG                 0x03
#define INTERRUPT_DIE_TEMP_REG_DIE_TEMP_EN     (0x01 << 1)

// MODE_CONFIG_REG 位定义
#define MODE_CONFIG_REG_SHDN            0x80    // 休眠使能
#define MODE_CONFIG_REG_RESET           0x40    // 复位
#define MODE_CONFIG_REG_MODE_SpO2       0x03    // SpO2模式 (Red + IR)
#define MODE_CONFIG_REG_MODE_HR         0x02    // 心率模式 (Red only)
#define MODE_CONFIG_REG_MODE_Multi      0x07    // 多LED模式

// FIFO_CONFIG_REG 位定义
#define FIFO_CONFIG_REG_SMP_AVE0        0x00    // 采样平均：无平均
#define FIFO_CONFIG_REG_SMP_AVE4        (0x02 << 5)  // 4 samples average
#define FIFO_CONFIG_REG_FIFO_ROLLOVER_EN  0x10    // FIFO回滚使能
#define FIFO_CONFIG_REG_FIFO_ROLLOVER_DIS  0x00    // FIFO回滚禁用
#define FIFO_CONFIG_REG_FIFO_ALL_FULL     0x00    // FIFO几乎满中断

// SPO2_CONFIG_REG 位定义
#define SPO2_CONFIG_REG_ADC_RGE        0x60    // ADC量程: 16384nA (最大)
#define ADC_RGE_00                     0x00    // ADC量程: 2048nA
#define ADC_RGE_01                     0x20    // ADC量程: 4096nA
#define ADC_RGE_10                     0x40    // ADC量程: 8192nA
#define ADC_RGE_11                     0x60    // ADC量程: 16384nA

// 采样率定义
#define SAMPLE_50                      0x00    // 采样率: 50sps
#define SAMPLE_100                     0x04    // 采样率: 100sps
#define SAMPLE_200                     0x08    // 采样率: 200sps
#define SAMPLE_400                     0x0C    // 采样率: 400sps
#define SAMPLE_800                     0x10    // 采样率: 800sps
#define SAMPLE_1000                    0x14    // 采样率: 1000sps
#define SAMPLE_1600                    0x18    // 采样率: 1600sps
#define SAMPLE_3200                    0x1C    // 采样率: 3200sps

#define SPO2_CONFIG_REG_LED_PW         0x03    // LED脉宽: 411us (18-bit)

// FIFO读取数量宏
#define ONES_READ_DATA_BY_FIFO         (32 - FIFO_CONFIG_REG_FIFO_ALL_FULL)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
// 初始化函数
void MAX30101_Init(void);

// 基本读写函数
uint8_t MAX_ReadOneByte(uint16_t ReadAddr);
void MAX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite);
void MAX_ReadLenByte(uint16_t ReadAddr, uint8_t *pBuffer, uint8_t Len);
void MAX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len);

// FIFO操作函数
void MAX_ReadFIFOByte(uint32_t *pBuffer);              // 读取单个FIFO数据（3字节）
void MAX_SpO2_ReadFIFOByte(uint32_t *pBuffer);         // 读取SpO2模式FIFO数据（6字节：Red+IR）
void MAX_ReadFIFO_Burst(uint8_t *buf, uint8_t len);    // 批量读取FIFO数据

// 检测函数
uint8_t MAX_Check(void);

// 辅助函数
void MAX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
void MAX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);

#ifdef __cplusplus
}
#endif

#endif /* __MAX30101_H */
