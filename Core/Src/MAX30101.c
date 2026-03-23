/**
 ******************************************************************************
 * @file    MAX30101.c
 * @brief   MAX30101 PPG传感器驱动实现 (支持心率/血氧双模式)
 ******************************************************************************
 * @attention
 * MAX30101 寄存器读写和FIFO操作实现
 ******************************************************************************
 */

#include "MAX30101.h"

/**
 * @brief MAX30101寄存器初始化
 * @note  配置为Multi-LED模式，后续可通过PPG_Config函数切换具体模式
 */
void MAX30101_Init(void)
{
	/* 清除中断 */
	MAX_WriteOneByte(0x00, 0x00);  // 中断状态寄存器1
	MAX_WriteOneByte(0x01, 0x00);  // 中断状态寄存器2
	MAX_WriteOneByte(0x02, 0x00);  // 关闭所有中断
	MAX_WriteOneByte(0x03, 0x00);  // 温度中断使能

	/* 清除FIFO相关寄存器 */
	MAX_WriteOneByte(FIFO_WR_PTR_REG, 0x00);   // 写指针清零
	MAX_WriteOneByte(OVF_COUNTER_REG, 0x00);   // 溢出计数器清零
	MAX_WriteOneByte(FIFO_RD_PTR_REG, 0x00);   // 读指针清零

	/* 配置模式 */
	MAX_WriteOneByte(FIFO_CONFIG_REG, FIFO_CONFIG_REG_SMP_AVE0 |
	                                   FIFO_CONFIG_REG_FIFO_ROLLOVER_EN |
	                                   FIFO_CONFIG_REG_FIFO_ALL_FULL);

	MAX_WriteOneByte(MODE_CONFIG_REG, MODE_CONFIG_REG_SHDN | MODE_CONFIG_REG_RESET | MODE_CONFIG_REG_MODE_Multi);

	MAX_WriteOneByte(SPO2_CONFIG_REG, ADC_RGE_00 | SAMPLE_3200 | SPO2_CONFIG_REG_LED_PW);

	MAX_WriteOneByte(LED1_PA_REG, 0xB1);  // Red LED 亮度
	MAX_WriteOneByte(LED2_PA_REG, 0xB1);  // IR LED 亮度
	MAX_WriteOneByte(LED3_PA_REG, 0xB1);  // Green LED 亮度
	MAX_WriteOneByte(LED4_PA_REG, 0xB1);  // 保留

	MAX_WriteOneByte(LED_CONTROL1, 0x01);  // LED控制
	MAX_WriteOneByte(LED_CONTROL2, 0x00);
}

/**
 * @brief 从MAX30101指定地址读出一个字节数据
 * @param ReadAddr 寄存器地址
 * @return 读到的数据
 */
uint8_t MAX_ReadOneByte(uint16_t ReadAddr)
{
	uint8_t temp = 0;
	IIC_Start();
	IIC_Send_Byte(0XAE);    // 发送器件写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr); // 发送寄存器地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);    // 发送器件读地址
	IIC_Wait_Ack();
	temp = IIC_Read_Byte(0); // 读取数据
	IIC_Stop();             // 产生停止信号
	return temp;
}

/**
 * @brief 向MAX30101指定地址写入一个字节数据
 * @param WriteAddr 目标寄存器地址
 * @param DataToWrite 要写入的数据
 */
void MAX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite)
{
	IIC_Start();
	IIC_Send_Byte(0XAE);     // 发送器件写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr); // 发送寄存器地址
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite); // 发送数据
	IIC_Wait_Ack();
	IIC_Stop();              // 产生停止信号
}

/**
 * @brief 从MAX30101指定地址开始写入长度为Len的数据
 * @note  该函数适合写入16bit或者32bit的数据
 * @param WriteAddr 开始写入的地址
 * @param DataToWrite 数据数组首地址
 * @param Len 要写入数据的长度 (2或4)
 */
void MAX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len)
{
	uint8_t t;
	for(t = 0; t < Len; t++)
	{
		MAX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xff);
	}
}

/**
 * @brief 从MAX30101指定地址开始读取长度为Len的数据
 * @note  该函数适合读取16bit或者32bit的数据
 * @param ReadAddr 开始读取的地址
 * @param pBuffer 数据缓冲区
 * @param Len 要读取数据的长度 (2或4)
 */
void MAX_ReadLenByte(uint16_t ReadAddr, uint8_t *pBuffer, uint8_t Len)
{
	uint8_t i = 0;
	IIC_Start();
	IIC_Send_Byte(0XAE);
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);    // 进入读模式
	IIC_Wait_Ack();

	for(i = 0; i < Len; i++)
	{
		if(i == Len - 1)
			*pBuffer = IIC_Read_Byte(0);  // 最后一个字节发送NACK
		else
		{
			*pBuffer = IIC_Read_Byte(1);  // 发送ACK继续读取
			pBuffer++;
		}
	}
	IIC_Stop();  // 产生停止信号
}

/**
 * @brief 检测MAX30101是否存在
 * @return 1: 检测成功, 0: 检测失败
 */
uint8_t MAX_Check(void)
{
	uint8_t temp = 0;
	temp = MAX_ReadOneByte(0xFF);  // 读取revision ID
	if(temp == 0X15)
		return 1;
	else
		return 0;
}

/**
 * @brief 从MAX30101指定地址开始读取指定长度的数据
 * @param ReadAddr 开始读取的地址
 * @param pBuffer 数据缓冲区
 * @param NumToRead 要读取数据的个数
 */
void MAX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++ = MAX_ReadOneByte(ReadAddr++);
		NumToRead--;
	}
}

/**
 * @brief 从MAX30101指定地址开始写入指定长度的数据
 * @param WriteAddr 开始写入的地址
 * @param pBuffer 数据缓冲区
 * @param NumToWrite 要写入数据的个数
 */
void MAX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
	while(NumToWrite--)
	{
		MAX_WriteOneByte(WriteAddr, *pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}

/**
 * @brief 从FIFO中读取单个3字节PPG数据 (心率模式)
 * @param pBuffer 读取数据缓冲区指针
 */
void MAX_ReadFIFOByte(uint32_t *pBuffer)
{
	uint8_t Buf_temp[3] = {0};
	IIC_Start();
	IIC_Send_Byte(0XAE);
	IIC_Wait_Ack();
	IIC_Send_Byte(FIFO_DATA_REG);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);    // 进入读模式
	IIC_Wait_Ack();

	Buf_temp[0] = IIC_Read_Byte(1) & 0x03;  // 只保留低2位 (18-bit有效位)
	Buf_temp[1] = IIC_Read_Byte(1);
	Buf_temp[2] = IIC_Read_Byte(0);
	*pBuffer = ((Buf_temp[0] << 16) | (Buf_temp[1] << 8)) | Buf_temp[2];

	IIC_Stop();  // 产生停止信号
}

/**
 * @brief 血氧模式每次读取2个channel的数据，共6字节 (Red + IR)
 * @param pBuffer 读取数据缓冲区指针 (至少2个uint32_t)
 */
void MAX_SpO2_ReadFIFOByte(uint32_t *pBuffer)
{
	uint8_t Buf_temp[6] = {0};
	IIC_Start();
	IIC_Send_Byte(0XAE);
	IIC_Wait_Ack();
	IIC_Send_Byte(FIFO_DATA_REG);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);    // 进入读模式
	IIC_Wait_Ack();

	Buf_temp[0] = IIC_Read_Byte(1) & 0x03;  // Red高2位
	Buf_temp[1] = IIC_Read_Byte(1);         // Red中8位
	Buf_temp[2] = IIC_Read_Byte(1);         // Red低8位
	Buf_temp[3] = IIC_Read_Byte(1) & 0x03;  // IR高2位
	Buf_temp[4] = IIC_Read_Byte(1);         // IR中8位
	Buf_temp[5] = IIC_Read_Byte(0);         // IR低8位

	*pBuffer = ((Buf_temp[0] << 16) | (Buf_temp[1] << 8)) | Buf_temp[2];
	pBuffer++;
	*pBuffer = ((Buf_temp[3] << 16) | (Buf_temp[4] << 8)) | Buf_temp[5];

	IIC_Stop();  // 产生停止信号
}

/**
 * @brief 批量读取FIFO数据 (用于主循环FIFO排空)
 * @param buf 数据缓冲区
 * @param len 要读取的字节数
 * @note 支持心率模式(3字节)和血氧模式(6字节)的批量读取
 */
void MAX_ReadFIFO_Burst(uint8_t *buf, uint8_t len)
{
	IIC_Start();
	IIC_Send_Byte(0XAE);           // 发送器件写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(FIFO_DATA_REG);  // 发送FIFO数据寄存器地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XAF);           // 发送器件读地址
	IIC_Wait_Ack();

	// 批量读取 len 字节
	for(uint8_t i = 0; i < len; i++)
	{
		if(i == len - 1)
			buf[i] = IIC_Read_Byte(0);  // 最后一个字节发送NACK
		else
			buf[i] = IIC_Read_Byte(1);  // 发送ACK继续读取
	}

	IIC_Stop();
}
