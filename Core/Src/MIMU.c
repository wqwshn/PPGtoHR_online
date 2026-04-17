#include "MIMU.h"

extern uint8_t ACC_XYZ[];
extern uint8_t GYRO_XYZ[];
extern uint8_t MAG_XYZ[];

/* ── SPI 单字节写寄存器 ── */
void ACC_GYRO_Write(uint8_t RegAddress, uint8_t txData){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);  // 选中ACC/GYRO
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);        // 取消磁力计

	spiTxData[0] = RegAddress & 0x7F;  // bit7=0: 写操作
	spiTxData[1] = txData;

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);    // 释放CS
}


/* ── SPI 单字节读寄存器 ── */
uint8_t ACC_GYRO_Read(uint8_t RegAddress){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	spiTxData[0] = 0x80 | RegAddress;  // bit7=1: 读操作

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);

	return spiRxData[1];
}


/* ── MAG 写寄存器 ── */
void MAG_Write(uint8_t RegAddress, uint8_t txData){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);

	spiTxData[0] = RegAddress & 0x7F;
	spiTxData[1] = txData;

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);
}


/* ── MAG 读寄存器 ── */
uint8_t MAG_Read(uint8_t RegAddress){
	uint8_t spiTxData[2] = {0};
	uint8_t spiRxData[2] = {0};
	spiTxData[0] = 0x80 | RegAddress;

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 2, 0xffff);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	return spiRxData[1];
}


/* ── 加速度计6字节突发读取 (小端序, OUT_X_L_XL=0x28) ── */
void ACC_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};
	spiTxData[0] = 0x80 | 0x28;  // 读 OUT_X_L_XL, IF_ADD_INC 自动递增

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);

	for(uint8_t i = 0; i < 6; i++){
		ACC_XYZ[i] = spiRxData[i+1];
	}

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);  // 读取完毕释放CS
}


/* ── 陀螺仪6字节突发读取 (小端序, OUT_X_L_G=0x18) ── */
void GYRO_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);

	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};
	spiTxData[0] = 0x80 | 0x18;  // 读 OUT_X_L_G, IF_ADD_INC 自动递增

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);

	for(uint8_t i = 0; i < 6; i++){
		GYRO_XYZ[i] = spiRxData[i+1];
	}

	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);  // 读取完毕释放CS
}


/* ── 磁力计6字节突发读取 ── */
void MAG_6BytesRead(void){
	HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_RESET);

	uint8_t spiTxData[7] = {0};
	uint8_t spiRxData[7] = {0};
	spiTxData[0] = 0xC0 | 0x28;  // 读 OUT_X_L_M, 地址递增

	HAL_SPI_TransmitReceive(&hspi2, spiTxData, spiRxData, 7, 0xffff);

	for(uint8_t i = 0; i < 6; i++){
		MAG_XYZ[i] = spiRxData[i+1];
	}

	HAL_GPIO_WritePin(CS_M_GPIO_Port, CS_M_Pin, GPIO_PIN_SET);
}


/*
 * MIMU 初始化
 *
 * 初始化流程 (参考 LSM9DS1.md):
 *   1. 软复位 CTRL_REG8=0x05, 延时20ms
 *   2. 启用 BDU + IF_ADD_INC: CTRL_REG8=0x44
 *   3. 关闭 FIFO 和硬件中断
 *   4. 配置陀螺仪: 119Hz, ±500dps, BW=14Hz, 高通滤波
 *   5. 配置加速度计: 119Hz, ±4g, 高分辨率, 数字滤波
 *   6. 配置磁力计
 */
void MIMU_Init(void){
	/* Step-1: 软复位 + IF_ADD_INC, 等待寄存器重装完成 */
	ACC_GYRO_Write(CTRL_REG8, 0x05);   // SW_RESET=1 + IF_ADD_INC=1
	HAL_Delay(20);

	/* Step-2: BDU(防撕裂) + IF_ADD_INC(突发读取地址递增) */
	ACC_GYRO_Write(CTRL_REG8, 0x44);   // BDU=1 + IF_ADD_INC=1

	/* Step-3: 关闭 FIFO 和硬件中断 */
	ACC_GYRO_Write(CTRL_REG9, 0x04);   // 禁用FIFO, 禁用I2C

	/* Step-4: 陀螺仪配置
	 * CTRL_REG1_G=0x68: ODR=119Hz, FS=±500dps, BW=14Hz
	 * CTRL_REG3_G=0x46: HP_EN=1, HPCF=0b0110 (约0.1Hz截止)
	 */
	ACC_GYRO_Write(CTRL_REG1_G, 0x68);
	ACC_GYRO_Write(CTRL_REG3_G, 0x46);  // 开启高通滤波消除零偏漂移

	/* Step-5: 加速度计配置
	 * CTRL_REG6_XL=0x70: ODR=119Hz, FS=±4g, 自动带宽
	 * CTRL_REG7_XL=0xC4: 高分辨率, 数字滤波ODR/9, 滤波后输出
	 */
	ACC_GYRO_Write(CTRL_REG6_XL, 0x70);
	ACC_GYRO_Write(CTRL_REG7_XL, 0x00);  // FDS=1会导致ACC输出趋零

	/* Step-6: 磁力计配置 (保持原有配置不变) */
	MAG_Write(CTRL_REG1_M, 0xFC);  // 磁温补, XY轴UHP模式, ODR=80Hz
	MAG_Write(CTRL_REG2_M, 0x00);  // 量程±4gauss
	MAG_Write(CTRL_REG3_M, 0x80);  // 禁用I2C, SPI读写, 连续转换模式
	MAG_Write(CTRL_REG4_M, 0x0C);  // Z轴UHP模式
	MAG_Write(CTRL_REG5_M, 0x00);  // 连续转换模式
}


/* ── 陀螺仪零偏标定 (静态采集N个样本取均值) ── */
int16_t gyro_offset[3] = {0, 0, 0};

void MIMU_GyroCalibrate(void){
	int32_t sum_x = 0, sum_y = 0, sum_z = 0;
	const int N = 200;  // 200个样本 ~2秒

	HAL_Delay(500);  // 等待传感器稳定

	for(int i = 0; i < N; i++){
		GYRO_6BytesRead();
		sum_x += (int16_t)((GYRO_XYZ[1] << 8) | GYRO_XYZ[0]);
		sum_y += (int16_t)((GYRO_XYZ[3] << 8) | GYRO_XYZ[2]);
		sum_z += (int16_t)((GYRO_XYZ[5] << 8) | GYRO_XYZ[4]);
		HAL_Delay(10);  // 100Hz, 略低于ODR 119Hz
	}

	gyro_offset[0] = (int16_t)(sum_x / N);
	gyro_offset[1] = (int16_t)(sum_y / N);
	gyro_offset[2] = (int16_t)(sum_z / N);
}


/* ── MIMU 连接检查 ── */
uint8_t MIMU_check(void){
	uint8_t mimu_id[2] = {0};
	mimu_id[0] = ACC_GYRO_Read(WHO_AM_I);
	mimu_id[1] = MAG_Read(WHO_AM_I_M);

	if((mimu_id[0] == 0x68) && (mimu_id[1] == 0x3D))
		return 1;
	else
		return 0;
}
