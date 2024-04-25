/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
void CDC_FS_RxDataReady_Callback(uint8_t *, uint8_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RF_AMP_EN1_Pin GPIO_PIN_14
#define RF_AMP_EN1_GPIO_Port GPIOC
#define RF_AMP_EN2_Pin GPIO_PIN_15
#define RF_AMP_EN2_GPIO_Port GPIOC
#define TRANS_GPIO1_Pin GPIO_PIN_0
#define TRANS_GPIO1_GPIO_Port GPIOA
#define TRANS_GPIO0_Pin GPIO_PIN_1
#define TRANS_GPIO0_GPIO_Port GPIOA
#define SYS_RST_Pin GPIO_PIN_4
#define SYS_RST_GPIO_Port GPIOA
#define FWD_MEAS_ADC_Pin GPIO_PIN_6
#define FWD_MEAS_ADC_GPIO_Port GPIOA
#define REV_MEAS_ADC_Pin GPIO_PIN_7
#define REV_MEAS_ADC_GPIO_Port GPIOA
#define TRANS_SDN_Pin GPIO_PIN_8
#define TRANS_SDN_GPIO_Port GPIOA
#define LDO_PG_Pin GPIO_PIN_9
#define LDO_PG_GPIO_Port GPIOA
#define TRANS_NIRQ_Pin GPIO_PIN_10
#define TRANS_NIRQ_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define FWD_MEAS_AMP_EN_Pin GPIO_PIN_6
#define FWD_MEAS_AMP_EN_GPIO_Port GPIOB
#define REV_MEAS_AMP_EN_Pin GPIO_PIN_7
#define REV_MEAS_AMP_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
