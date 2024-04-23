/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdlib.h"
#include "string.h"
#include "task.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern volatile uint32_t Task1;
extern uint8_t Get_Start_WT9011G4K_Angle_Flag;
extern uint8_t X,Y;
  
//»úÐµ±Û
extern int16_t Uart_Read(uint8_t *buf , uint16_t len, uint32_t timeout);
extern void Uart_Send(uint8_t *buf , uint16_t len);
extern int16_t Uart_Read_CL(void);

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
void Uart_Flush_CL(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIN1_LF_Pin GPIO_PIN_5
#define BIN1_LF_GPIO_Port GPIOE
#define BIN2_LF_Pin GPIO_PIN_6
#define BIN2_LF_GPIO_Port GPIOE
#define OpenMV_RST_Pin GPIO_PIN_1
#define OpenMV_RST_GPIO_Port GPIOC
#define PS2_CS_Pin GPIO_PIN_3
#define PS2_CS_GPIO_Port GPIOC
#define EncoderB_RB_Pin GPIO_PIN_0
#define EncoderB_RB_GPIO_Port GPIOA
#define EncoderA_RB_Pin GPIO_PIN_1
#define EncoderA_RB_GPIO_Port GPIOA
#define G0O2_Pin GPIO_PIN_2
#define G0O2_GPIO_Port GPIOA
#define G0I1_Pin GPIO_PIN_3
#define G0I1_GPIO_Port GPIOA
#define G0I2_Pin GPIO_PIN_4
#define G0I2_GPIO_Port GPIOA
#define Beep_Pin GPIO_PIN_6
#define Beep_GPIO_Port GPIOA
#define G0O1_Pin GPIO_PIN_4
#define G0O1_GPIO_Port GPIOC
#define LKW_L_Pin GPIO_PIN_5
#define LKW_L_GPIO_Port GPIOC
#define Battery_Pin GPIO_PIN_0
#define Battery_GPIO_Port GPIOB
#define Regulator_Pin GPIO_PIN_1
#define Regulator_GPIO_Port GPIOB
#define LKW_F_Pin GPIO_PIN_8
#define LKW_F_GPIO_Port GPIOE
#define CIN1_RB_Pin GPIO_PIN_9
#define CIN1_RB_GPIO_Port GPIOE
#define LKW_R_Pin GPIO_PIN_10
#define LKW_R_GPIO_Port GPIOE
#define CIN2_RB_Pin GPIO_PIN_11
#define CIN2_RB_GPIO_Port GPIOE
#define LKW_B_Pin GPIO_PIN_12
#define LKW_B_GPIO_Port GPIOE
#define DIN2_RF_Pin GPIO_PIN_13
#define DIN2_RF_GPIO_Port GPIOE
#define DIN1_RF_Pin GPIO_PIN_14
#define DIN1_RF_GPIO_Port GPIOE
#define Ukey_Mode_Pin GPIO_PIN_15
#define Ukey_Mode_GPIO_Port GPIOE
#define SunX3_TX_Pin GPIO_PIN_8
#define SunX3_TX_GPIO_Port GPIOD
#define SunX3_RX_Pin GPIO_PIN_9
#define SunX3_RX_GPIO_Port GPIOD
#define Ukey_Up_Pin GPIO_PIN_11
#define Ukey_Up_GPIO_Port GPIOD
#define EncoderB_LB_Pin GPIO_PIN_12
#define EncoderB_LB_GPIO_Port GPIOD
#define EncoderA_LB_Pin GPIO_PIN_13
#define EncoderA_LB_GPIO_Port GPIOD
#define Ukey_Down_Pin GPIO_PIN_14
#define Ukey_Down_GPIO_Port GPIOD
#define Toggle_Pin GPIO_PIN_15
#define Toggle_GPIO_Port GPIOD
#define Arm_TX_Pin GPIO_PIN_6
#define Arm_TX_GPIO_Port GPIOC
#define Arm_RX_Pin GPIO_PIN_7
#define Arm_RX_GPIO_Port GPIOC
#define Gripper_Pin GPIO_PIN_8
#define Gripper_GPIO_Port GPIOC
#define Gyro_SDA_Pin GPIO_PIN_9
#define Gyro_SDA_GPIO_Port GPIOC
#define Gyro_SCL_Pin GPIO_PIN_8
#define Gyro_SCL_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define EncoderB_LF_Pin GPIO_PIN_15
#define EncoderB_LF_GPIO_Port GPIOA
#define LaserX_TX_Pin GPIO_PIN_10
#define LaserX_TX_GPIO_Port GPIOC
#define LaserX_RX_Pin GPIO_PIN_11
#define LaserX_RX_GPIO_Port GPIOC
#define LaserY_TX_Pin GPIO_PIN_12
#define LaserY_TX_GPIO_Port GPIOC
#define LaserY_RX_Pin GPIO_PIN_2
#define LaserY_RX_GPIO_Port GPIOD
#define Ukey_RST_Pin GPIO_PIN_3
#define Ukey_RST_GPIO_Port GPIOD
#define OpenMV_TX_Pin GPIO_PIN_5
#define OpenMV_TX_GPIO_Port GPIOD
#define OpenMV_RX_Pin GPIO_PIN_6
#define OpenMV_RX_GPIO_Port GPIOD
#define EncoderA_LF_Pin GPIO_PIN_3
#define EncoderA_LF_GPIO_Port GPIOB
#define EncoderB_RF_Pin GPIO_PIN_4
#define EncoderB_RF_GPIO_Port GPIOB
#define EncoderA_RF_Pin GPIO_PIN_5
#define EncoderA_RF_GPIO_Port GPIOB
#define G0_TX_Pin GPIO_PIN_6
#define G0_TX_GPIO_Port GPIOB
#define G0_RX_Pin GPIO_PIN_7
#define G0_RX_GPIO_Port GPIOB
#define AIN2_LB_Pin GPIO_PIN_8
#define AIN2_LB_GPIO_Port GPIOB
#define AIN1_LB_Pin GPIO_PIN_9
#define AIN1_LB_GPIO_Port GPIOB
#define NRST_Pin GPIO_PIN_0
#define NRST_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
