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
#include "stm32f4xx_hal.h"

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
#define BIN1_RF_Pin GPIO_PIN_5
#define BIN1_RF_GPIO_Port GPIOE
#define BIN2_RF_Pin GPIO_PIN_6
#define BIN2_RF_GPIO_Port GPIOE
#define CIN1_RB_Pin GPIO_PIN_9
#define CIN1_RB_GPIO_Port GPIOE
#define CIN2_RB_Pin GPIO_PIN_11
#define CIN2_RB_GPIO_Port GPIOE
#define DIN2_LB_Pin GPIO_PIN_13
#define DIN2_LB_GPIO_Port GPIOE
#define DIN1_LB_Pin GPIO_PIN_14
#define DIN1_LB_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define AIN2_LF_Pin GPIO_PIN_8
#define AIN2_LF_GPIO_Port GPIOB
#define AIN1_LF_Pin GPIO_PIN_9
#define AIN1_LF_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
