/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define CAM_D6_Pin GPIO_PIN_6
#define CAM_D6_GPIO_Port GPIOI
#define CAM_VSYNC_Pin GPIO_PIN_5
#define CAM_VSYNC_GPIO_Port GPIOI
#define CAM_D5_Pin GPIO_PIN_4
#define CAM_D5_GPIO_Port GPIOI
#define CAM_D2_Pin GPIO_PIN_10
#define CAM_D2_GPIO_Port GPIOG
#define LORA_RXD_Pin GPIO_PIN_9
#define LORA_RXD_GPIO_Port GPIOG
#define GPS_TX_Pin GPIO_PIN_5
#define GPS_TX_GPIO_Port GPIOD
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOC
#define CAM_D7_Pin GPIO_PIN_7
#define CAM_D7_GPIO_Port GPIOI
#define CAM_D3_Pin GPIO_PIN_1
#define CAM_D3_GPIO_Port GPIOE
#define INA219_SCL_Pin GPIO_PIN_6
#define INA219_SCL_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_6
#define GPS_RX_GPIO_Port GPIOD
#define CAM_D4_Pin GPIO_PIN_11
#define CAM_D4_GPIO_Port GPIOC
#define GATE_GPS_Pin GPIO_PIN_2
#define GATE_GPS_GPIO_Port GPIOE
#define INA219_SDA_Pin GPIO_PIN_7
#define INA219_SDA_GPIO_Port GPIOB
#define TEMP_DATA_Pin GPIO_PIN_12
#define TEMP_DATA_GPIO_Port GPIOC
#define IMC_SDA_Pin GPIO_PIN_9
#define IMC_SDA_GPIO_Port GPIOB
#define IMC_SCL_Pin GPIO_PIN_8
#define IMC_SCL_GPIO_Port GPIOB
#define LORA_TXD_Pin GPIO_PIN_14
#define LORA_TXD_GPIO_Port GPIOG
#define LORA_AUX_Pin GPIO_PIN_14
#define LORA_AUX_GPIO_Port GPIOJ
#define LORA_M1_Pin GPIO_PIN_2
#define LORA_M1_GPIO_Port GPIOD
#define LORA_M0_Pin GPIO_PIN_0
#define LORA_M0_GPIO_Port GPIOD
#define CAM_D1_Pin GPIO_PIN_10
#define CAM_D1_GPIO_Port GPIOA
#define CAM_D0_Pin GPIO_PIN_9
#define CAM_D0_GPIO_Port GPIOA
#define MAG_DRDY_Pin GPIO_PIN_6
#define MAG_DRDY_GPIO_Port GPIOC
#define GPS_PPS_Pin GPIO_PIN_8
#define GPS_PPS_GPIO_Port GPIOG
#define CAM_PWDN_Pin GPIO_PIN_5
#define CAM_PWDN_GPIO_Port GPIOG
#define CAM_RST_Pin GPIO_PIN_6
#define CAM_RST_GPIO_Port GPIOG
#define GATE_CAM_Pin GPIO_PIN_1
#define GATE_CAM_GPIO_Port GPIOC
#define GATE_LORA_Pin GPIO_PIN_2
#define GATE_LORA_GPIO_Port GPIOC
#define GATE_IMU_Pin GPIO_PIN_3
#define GATE_IMU_GPIO_Port GPIOC
#define DEBUG_RX_Pin GPIO_PIN_11
#define DEBUG_RX_GPIO_Port GPIOB
#define CAM_DCLK_Pin GPIO_PIN_6
#define CAM_DCLK_GPIO_Port GPIOA
#define CQAM_HRFF_Pin GPIO_PIN_8
#define CQAM_HRFF_GPIO_Port GPIOH
#define GATE_TEMP_Pin GPIO_PIN_13
#define GATE_TEMP_GPIO_Port GPIOB
#define GATE_MAG_Pin GPIO_PIN_14
#define GATE_MAG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
