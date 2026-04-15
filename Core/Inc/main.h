/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C_SCL1_Pin GPIO_PIN_0
#define I2C_SCL1_GPIO_Port GPIOA
#define I2C_SDA1_Pin GPIO_PIN_1
#define I2C_SDA1_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define I2C_SCL2_Pin GPIO_PIN_0
#define I2C_SCL2_GPIO_Port GPIOB
#define I2C_SDA2_Pin GPIO_PIN_1
#define I2C_SDA2_GPIO_Port GPIOB
#define BLE_STATE_Pin GPIO_PIN_12
#define BLE_STATE_GPIO_Port GPIOB
#define BLE_RST_Pin GPIO_PIN_13
#define BLE_RST_GPIO_Port GPIOB
#define V5_0_CE_Pin GPIO_PIN_8
#define V5_0_CE_GPIO_Port GPIOA
#define V1_8_CE_Pin GPIO_PIN_9
#define V1_8_CE_GPIO_Port GPIOA
#define START_CONV_Pin GPIO_PIN_10
#define START_CONV_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_11
#define DRDY_GPIO_Port GPIOA
#define DRDY_EXTI_IRQn EXTI15_10_IRQn
#define AD_RESET_Pin GPIO_PIN_12
#define AD_RESET_GPIO_Port GPIOA
#define DEN_A_G_Pin GPIO_PIN_15
#define DEN_A_G_GPIO_Port GPIOA
#define INT2_A_G_Pin GPIO_PIN_3
#define INT2_A_G_GPIO_Port GPIOB
#define INT1_A_G_Pin GPIO_PIN_4
#define INT1_A_G_GPIO_Port GPIOB
#define INT2_M_Pin GPIO_PIN_5
#define INT2_M_GPIO_Port GPIOB
#define DRDY_M_Pin GPIO_PIN_6
#define DRDY_M_GPIO_Port GPIOB
#define CS_A_G_Pin GPIO_PIN_7
#define CS_A_G_GPIO_Port GPIOB
#define CS_M_Pin GPIO_PIN_8
#define CS_M_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* ============================================================
 * 系统配置开关 (统一修改区, 改后重新编译烧录即可生效)
 * ============================================================ */

/* 工作模式选择 */
#define MODE_HEART_RATE 0
#define MODE_SPO2       1
#define CURRENT_WORK_MODE    MODE_HEART_RATE   /* 0=心率模式, 1=血氧模式 */

/* 数据发送模式选择 (两种模式互斥, 不会同时发送)
 * 0 = 在线心率模式: 运行算法, 仅发送 1Hz HR 结果包 (0xAA 0xCC) [仅125Hz]
 * 1 = 原始数据模式: 仅发送原始数据包 (0xAA 0xBB), 不运行算法 [所有采样率]
 */
#define ENABLE_RAW_DATA_PACKET  1

/* PPG 通道选择: 1=PPG1(IIC1总线), 2=PPG2(IIC2总线) */
#define PPG_DEFAULT_CHANNEL     2

/* PPG 采样率 (Hz): 50 / 100 / 125
 * 50Hz:  内部 800sps / 16x 硬件平均 = 50sps
 * 100Hz: 内部 800sps / 8x  硬件平均 = 100sps
 * 125Hz: 内部 1000sps / 4x 硬件平均 = 250sps, MCU 125Hz 读取
 */
#define PPG_SAMPLE_RATE         125

/* ============================================================ */

/* 统一数据包长度为21字节 */
#define PACKET_LEN 21
#define XOR_CHECK_LEN 17  // 校验区域统一为17字节 (ADC(8) + ACC(3) + PPG(4) + 扩展(2))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
