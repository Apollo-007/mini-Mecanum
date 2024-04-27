/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void motor_pwm(uint8_t motor,int16_t pwm);
void go_4_OL(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim10,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim11,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
		go_4_OL(1000,1000,1000,1000);//200.300

		/*
		HAL_Delay(5000);
		for(int i=1;i<=20;i++)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			HAL_Delay(100);
		}
		go_4_OL(-297,308,304,-299);
		HAL_Delay(4470);
		go_4_OL(0,0,0,0);
		for(int i=1;i<=16;i++)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			HAL_Delay(100);
		}
		go_4_OL(-300,-300,312,300);
		HAL_Delay(3150);
		go_4_OL(0,0,0,0);
		for(int i=1;i<=16;i++)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			HAL_Delay(100);
		}
		go_4_OL(303,-311,-296,312);
		HAL_Delay(4490);
		go_4_OL(0,0,0,0);
		for(int i=1;i<=16;i++)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			HAL_Delay(100);
		}
		go_4_OL(314,300,-299,-299);
		HAL_Delay(3128);
		go_4_OL(0,0,0,0);
		for(int i=1;i<=30;i++)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			HAL_Delay(100);
		}
		
		*/
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void motor_pwm(uint8_t motor,int16_t pwm){
	if(motor == 1)			//motor£ºLF 2  1 RF
	{										//				            pwm[-1000,1000] 
		if(pwm<0)//clockwise       LB 3  4 RB
		{
			if(pwm<-1000)
				pwm=-1000;
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,1000+pwm);
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,1000);
		}
		else//anticlockwise
		{
			if(pwm>1000)
				pwm=1000;
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,1000);
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,1000-pwm);
		}
	}
	else if(motor == 2)
	{
		if(pwm<0)//clockwise
		{
			if(pwm<-1000)
				pwm=-1000;
			__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,1000+pwm);
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,1000);
		}
		else//anticlockwise
		{
			if(pwm>1000)
				pwm=1000;
			__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,1000);
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,1000-pwm);
		}
	}
	else if(motor == 3)
	{
		if(pwm<0)//clockwise
		{
			if(pwm<-1000)
				pwm=-1000;
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1000+pwm);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1000);
		}
		else//anticlockwise
		{
			if(pwm>1000)
				pwm=1000;
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1000);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1000-pwm);
		}
	}
	else if(motor == 4)
	{
		if(pwm<0)//clockwise
		{
			if(pwm<-1000)
				pwm=-1000;
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000+pwm);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1000);
		}
		else//anticlockwise
		{
			if(pwm>1000)
				pwm=1000;
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1000-pwm);
		}
	}
}
void go_4_OL(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB){
	motor_pwm(1,motor_RF);
	motor_pwm(2,motor_LF);
	motor_pwm(3,motor_LB);
	motor_pwm(4,motor_RB);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
