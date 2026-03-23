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
/*
 *  I2C1 通信接口：
 *    I2C_SCL1_Pin      -> GPIOA PIN0   // I2C1 时钟线
 *    I2C_SDA1_Pin      -> GPIOA PIN1   // I2C1 数据线
*/
#define I2C_SCL1_Pin GPIO_PIN_0
#define I2C_SCL1_GPIO_Port GPIOA
#define I2C_SDA1_Pin GPIO_PIN_1
#define I2C_SDA1_GPIO_Port GPIOA
/*
 *  SPI1 通信接口：
 *    SPI1_NSS_Pin      -> GPIOA PIN4   // SPI1 片选信号 (Slave Select)
*/
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
/*
 *  I2C2 通信接口：
 *    I2C_SCL2_Pin      -> GPIOB PIN0   // I2C2 时钟线
 *    I2C_SDA2_Pin      -> GPIOB PIN1   // I2C2 数据线
*/
#define I2C_SCL2_Pin GPIO_PIN_0
#define I2C_SCL2_GPIO_Port GPIOB
#define I2C_SDA2_Pin GPIO_PIN_1
#define I2C_SDA2_GPIO_Port GPIOB
/*
 *  蓝牙模块 (BLE)：
 *    BLE_STATE_Pin     -> GPIOB PIN12  // 蓝牙状态指示
 *    BLE_RST_Pin       -> GPIOB PIN13  // 蓝牙复位信号
*/
#define BLE_STATE_Pin GPIO_PIN_12
#define BLE_STATE_GPIO_Port GPIOB
#define BLE_RST_Pin GPIO_PIN_13
#define BLE_RST_GPIO_Port GPIOB
/*
 *  电源控制模块：
 *    V5_0_CE_Pin       -> GPIOA PIN8   // 5.0V 电源使能
 *    V1_8_CE_Pin       -> GPIOA PIN9   // 1.8V 电源使能
*/
#define V5_0_CE_Pin GPIO_PIN_8
#define V5_0_CE_GPIO_Port GPIOA
#define V1_8_CE_Pin GPIO_PIN_9
#define V1_8_CE_GPIO_Port GPIOA
/*
 *  模数/ADC 模块：
 *    START_CONV_Pin    -> GPIOA PIN10  // 启动转换信号
 *    DRDY_Pin          -> GPIOA PIN11  // 数据准备好 (Data Ready)
 *    AD_RESET_Pin      -> GPIOA PIN12  // 模数转换器复位
*/
#define START_CONV_Pin GPIO_PIN_10
#define START_CONV_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_11
#define DRDY_GPIO_Port GPIOA
#define DRDY_EXTI_IRQn EXTI15_10_IRQn
#define AD_RESET_Pin GPIO_PIN_12
#define AD_RESET_GPIO_Port GPIOA
/*
 *  运动模块 (加速度计 / 陀螺仪 / 磁力计)：
 *    DEN_A_G_Pin       -> GPIOA PIN15  // 加速度计/陀螺仪使能
 *    INT2_A_G_Pin      -> GPIOB PIN3   // 加速度计/陀螺仪中断 2
 *    INT1_A_G_Pin      -> GPIOB PIN4   // 加速度计/陀螺仪中断 1
 *    INT2_M_Pin        -> GPIOB PIN5   // 磁力计中断 2
 *    DRDY_M_Pin        -> GPIOB PIN6   // 磁力计数据准备好
 *    CS_A_G_Pin        -> GPIOB PIN7   // 加速度计/陀螺仪片选
 *    CS_M_Pin          -> GPIOB PIN8   // 磁力计片选
*/
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

// 工作模式宏定义
#define MODE_HEART_RATE 0
#define MODE_SPO2       1

// 全局模式开关（修改此项后重新编译烧录即可切换模式）
// 心率：MODE_HEART_RATE 血氧：MODE_SPO2
#define CURRENT_WORK_MODE MODE_HEART_RATE

// 统一数据包长度为21字节
#define PACKET_LEN 21
#define XOR_CHECK_LEN 17  // 校验区域统一为17字节 (ADC(8) + ACC(3) + PPG(4) + 扩展(2))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
