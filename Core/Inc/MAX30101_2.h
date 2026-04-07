/**
 ******************************************************************************
 * @file    MAX30101_2.h
 * @brief   MAX30101 PPG传感器驱动头文件 - 通道2 (IIC2总线)
 ******************************************************************************
 * @attention
 * MAX30101 寄存器地址定义复用 MAX30101.h，函数前缀为 MAX2_
 * 使用独立的 IIC2 总线通信
 ******************************************************************************
 */

#ifndef __MAX30101_2_H
#define __MAX30101_2_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "myiic_2.h"   // IIC2通信驱动 (第二路I2C总线)
#include "MAX30101.h"  // 共享寄存器地址定义

/* Exported functions prototypes ---------------------------------------------*/
/* 初始化函数 */
void MAX30101_2_Init(void);

/* 基本读写函数 */
uint8_t MAX2_ReadOneByte(uint16_t ReadAddr);
void MAX2_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite);
void MAX2_ReadLenByte(uint16_t ReadAddr, uint8_t *pBuffer, uint8_t Len);
void MAX2_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len);

/* FIFO操作函数 */
void MAX2_ReadFIFOByte(uint32_t *pBuffer);              // 读取单个FIFO数据(3字节)
void MAX2_SpO2_ReadFIFOByte(uint32_t *pBuffer);         // 读取SpO2模式FIFO数据(6字节: Red+IR)
void MAX2_ReadFIFO_Burst(uint8_t *buf, uint8_t len);    // 批量读取FIFO数据

/* 检测函数 */
uint8_t MAX2_Check(void);

/* 辅助函数 */
void MAX2_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
void MAX2_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);

#ifdef __cplusplus
}
#endif

#endif /* __MAX30101_2_H */
