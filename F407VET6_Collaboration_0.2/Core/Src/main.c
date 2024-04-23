/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "SCServo.h"
#include "motor.h"
#include "control.h"
#include "wt9011g4k.h"
#include "laser.h"
#include "delay.h"

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
//用于定时器6中的变量，用于PID控制
volatile uint32_t PID_TimerCount = 0;

//用于定时器7中的变量，控制不同的动作
volatile uint32_t Task1  = 0;

//PID标志位
uint8_t PID_Flag;

uint8_t Run_Flag=0,Run_Flag_Start=0,Chassis_mode=0,Back_mode=0,ARM_mode=6;

//清除多余编码器数值标志位
uint8_t Clean_Flag=0;

//X轴,Y轴完成调整标志位
uint8_t X_Flag=0,Y_Flag=0;

//激光调试标志位
uint8_t X=0,Y=0;

//陀螺仪开始取值完成标志位
uint8_t Get_Start_WT9011G4K_Angle_Flag=0;

//PID切换标志位
/*
* Change_PID_Flag = 0 -> 用于运动PID
* Change_PID_Flag = 1 -> 用于陀螺仪PID
* Change_PID_Flag = 2 -> 用于激光PID
* Change_PID_Flag = 3 -> 用于拖动
*/
uint8_t Change_PID_Flag=0;

//每个任务标志位
/*
* mission_step = 1  ->出库任务
*
* mission_step = 2  ->扫码电机任务
* mission_step = 3  ->扫码操作任务
*
* mission_step = 4  ->第1轮取料1电机任务
* mission_step = 5  ->第1轮取料1操作任务
*
* mission_step = 6  ->第1轮转弯1操作任务
* mission_step = 7  ->第1轮转弯1微调任务
*
* mission_step = 8  ->第1轮物流放置1电机任务
* mission_step = 9  ->第1轮物流放置1操作任务
*
* mission_step = 10  ->第1轮转弯2操作任务
* mission_step = 11  ->第1轮转弯2微调任务
*
* mission_step = 12 ->第1轮物流放置2电机任务
* mission_step = 13 ->第1轮物流放置2操作任务
*
* mission_step = 14 ->第1轮转弯3操作任务
* mission_step = 15 ->第1轮转弯3微调任务
* 
* mission_step = 16 ->第1轮转弯4操作任务
* mission_step = 17 ->第1轮转弯4微调任务
*/
uint16_t mission_step=1;

uint8_t TxBuf[8] = {0xFF,0xFF,0x06,0x03,0x03,0x2A,0xC8,0x01};

//UART 读数据缓冲区
__IO uint8_t uartBuf[128];
__IO int head = 0;
__IO int tail  = 0;

struct
{
	int8_t Beep;
	int8_t Beep_Loop;
}Delay_Flag;		//传感器计时开始标志位

struct
{
	int32_t Beep;
	int32_t Beep_Loop;
}t_ms;					//传感器毫秒计时

struct
{
	int8_t Beep;
	int8_t Beep_Loop;
	int8_t Arm_Up;
	int8_t Arm_Down;
}Flag;					//传感器开启标志位

int16_t Beep_t=0,Beep_Loop=0,Beep_Loop_ms=0,Beep_Loop_t=0;
uint8_t UkeyCount_R=0,UkeyCount_M=0,UkeyCount_U=0,UkeyCount_D=0,UkeyCount_T=0;	//UkeyCount	表示	按键计数变量
uint8_t UkeyFlag_R=0,	UkeyFlag_M=0,	UkeyFlag_U=0,	UkeyFlag_D=0,	UkeyFlag_T=0;		//UkeyFlag	表示	重按键标志,	1代表重新按键,	0为没有重新按键
uint8_t UkeyState_R=0,UkeyState_M=0,UkeyState_U=0,UkeyState_D=0,UkeyState_T=0;	//UkeyState	表示	按键标志,		1代表按下,			0代表松开

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Beep_Init(void);
void Encoder_Init(void);
void RST_Enable(void);
void Beep_Enable(void);
void Beep_ms(uint16_t t);
void Beep_Loop_Cycle(uint16_t Beep_delay,uint8_t Loop,uint16_t Cycle);

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
	Flag.Arm_Up=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	Beep_Init();
	Flag.Arm_Up=-1;
	Flag.Arm_Down=0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  MX_TIM12_Init();
  MX_TIM14_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  
	HAL_TIM_Base_Start_IT(&htim6);
//  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim12);

  //开启PWM波生成
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim10,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim11,TIM_CHANNEL_1);
  
  //开启编码器
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
  
  //陀螺仪初始化
  WT9011G4K_Init(&WT9011G4K_Data);
  //X轴激光测距初始化
  LaserX_Init(&LaserX);
  //Y轴激光测距初始化
  LaserY_Init(&LaserY);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
		RST_Enable();
		Beep_Enable();
		
//    if(Run_Flag == 0)
//		{
//			if(UkeyState_T == 1||UkeyState_M == 1)
//			{
//				Run_Flag=1;
//				if(HAL_GPIO_ReadPin(Ukey_Mode_GPIO_Port,Ukey_Mode_Pin) == GPIO_PIN_RESET)
//        {
//					Back_mode = 1;
//        }
//				if(HAL_GPIO_ReadPin(Toggle_GPIO_Port,Toggle_Pin) == GPIO_PIN_RESET)
//        {
//          Chassis_mode  = 1;
//        }
//          UkeyState_T=0;
//          UkeyState_M=0;
//			}
//		} 
    
		if(UkeyState_M)
		{
			Beep_Loop_Cycle(30,1,100);
			UkeyState_M=0;
		}
		if(UkeyState_U)
		{
			Beep_Loop_Cycle(30,2,100);
			Flag.Arm_Up=1;
			UkeyState_U=0;
		}
		if(UkeyState_D)
		{
			Beep_Loop_Cycle(30,3,100);
			Flag.Arm_Up=0;
			UkeyState_D=0;
		}
		if(Flag.Arm_Up==1&&Beep_Loop==1)
		{
			WritePosEx(1, 3071, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
			WritePosEx(2, 2047, 1000, 0);
			WritePosEx(3, 2047, 3000, 0);
			WritePosEx(4, 3088, 2000, 0);
			WritePosEx(5, 2047, 2000, 0);
			
			Flag.Arm_Up=-1;
		}
		if(Flag.Arm_Up==0&&Beep_Loop==1)
		{
//			WritePosEx(1, 3071, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
//			WritePosEx(2, 1620, 1000, 0);
//			WritePosEx(3, 1410, 3000, 0);
//			WritePosEx(4, 3088, 2000, 0);
//			WritePosEx(5, 720, 2000, 0);
//			WritePos(6,1, 2000, 1);	
			
			WritePosEx(1, 3092, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
			WritePosEx(2, 2709, 1000, 0);
			WritePosEx(3, 547, 3000, 0);
			WritePosEx(4, 3088, 2000, 0);
			WritePosEx(5, 877, 2000, 0);

			Flag.Arm_Up=-1;
		}

		if(UkeyState_T)
		{
			Beep_ms(100);
			Chassis_mode  = 1;
//			Beep_Loop_Cycle(30,4,100);			
			UkeyState_T=0;
		}

    /*
     * 
     * 以下是各种传感器数据处理的区域
     * 
     */
    
      //PID数据处理
      if(PID_Flag == 1)
      {
        //PID控制权在运动PID
        if(Change_PID_Flag==0)
        {
          RF_Position_Incremental_PID();
          LB_Position_Incremental_PID();
          LF_Position_Incremental_PID();
          RB_Position_Incremental_PID();
        }
        
        //PID控制权在陀螺仪PID
        if(Change_PID_Flag==1 && Get_Start_WT9011G4K_Angle_Flag==1)
        {
          RF_Position_Incremental_PID_WT9011G4K();
          LF_Position_Incremental_PID_WT9011G4K();
          LB_Position_Incremental_PID_WT9011G4K();
          RB_Position_Incremental_PID_WT9011G4K();
        }
        
        //PID控制权在激光PID
        if(Change_PID_Flag==2 && X==1 && Y==1)
        {
          RF_Position_Incremental_PID_LASER();
          LF_Position_Incremental_PID_LASER();
          LB_Position_Incremental_PID_LASER();
          RB_Position_Incremental_PID_LASER();
        }
        
        //用于拖动
        if(Change_PID_Flag==3)
        {
        }
        
        PID_Flag = 0;
      }
    
    //这里会导致按键不灵敏
    //陀螺仪数据处理
    if(WT9011G4K_Data.Rx_flag == 1)
    {
      WT9011G4K_Process();
      WT9011G4K_Data.Rx_flag=0;
    }
      
    //X轴激光测距处理函数
    if(LaserX.Rx_flag == 1)
    {
      LaserX_Process();
      LaserX.Rx_flag=0;
    }
    
    //Y轴激光测距处理函数
    if(LaserY.Rx_flag == 1)
    {
      LaserY_Process();
      LaserY.Rx_flag=0;
    }
    
    /*
     * 
     * 以上是各种传感器数据处理的区域
     * 
     */

      
        
  /*
   * 
   * 以下是任务执行区域
   * 
   */
    if(Chassis_mode)
    {
			Encoder_Init();
			if(mission_step == 1 && Task1 <= 10000)	//出库
			{
				speed_4(20,20,20,20);
				go_4(-1320*0.50,-1320*0.50,1320*0.50,1320*0.50);
				if((1320*0.50 - Reality_Position_LB) < 6)
				{ 
					Chassis_OFF();
					mission_step++;
					Task1 = 0;
					Beep_ms(100);
				}
			}
		  if(mission_step == 2 && Task1 <= 10000)	//到原料区 (*2.5)
			{
				speed_4(20,20,20,20);
				go_4(-1320*5.8,1320*5.8,1320*5.8,-1320*5.8);
				if((1320*5.8 - Reality_Position_LF) < 6)
				{
					Chassis_OFF();
					mission_step++; 
					Task1 = 0;
					Beep_ms(100);
				}
			}

			if(mission_step==3 && Task1 <= 3000)
			{
				WritePosEx(1, 3071, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
				WritePosEx(2, 2709, 1000, 0);
				WritePosEx(3, 900, 3000, 0);
				WritePosEx(4, 3088, 2000, 0);
				WritePosEx(5, 877, 2000, 0);
				WritePos(6,1, 2000, 1);
				if(Task1 > 1600)
				{
					mission_step++;
					Task1=0;
					Beep_ms(100);
				}
			}
			if(mission_step==4 && Task1 <= 4000)
			{
				WritePosEx(1, 3071, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
				WritePosEx(2, 1620, 1000, 0);
				WritePosEx(3, 1410, 3000, 0);
				WritePosEx(4, 3088, 2000, 0);
				WritePosEx(5, 720, 2000, 0);
				if(Task1 > 3000)
				{
					mission_step++;
					Task1=0;
					Beep_ms(100);
				}
			}
			
			if(mission_step==5 && Task1 <= 3000)
			{
				WritePosEx(1, 3071, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
				WritePosEx(2, 2709, 1000, 0);
				WritePosEx(3, 900, 3000, 0);
				WritePosEx(4, 3088, 2000, 0);
				WritePosEx(5, 877, 2000, 0);
				WritePos(6,1, 2000, 1);

				if(Task1 > 2000)
				{
					mission_step++;
					Task1=0;
					Beep_ms(100);
				}
			}
			if(mission_step==6 && Task1 <= 3000)
			{
				WritePosEx(1, 3092, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
				WritePosEx(2, 2709, 1000, 0);
				WritePosEx(3, 547, 3000, 0);
				WritePosEx(4, 3088, 2000, 0);
				WritePosEx(5, 877, 2000, 0);
				WritePos(6,3, 2000, 1);

				if(Task1 > 2000)
				{
					mission_step++;
					Task1=0;
					Beep_ms(100);
				}
				
			}				
    /*
     *扫码的工作区间
     */ 
      
      /*                         
      *                          |->电机运动 
      * 功 能 :任务3 -> 取料任务 |
      *                          |->扫码运动 
      */         
//        if()
//        {
//          
//        }
    }      

  /*
   * 
   * 以上是任务执行区域
   * 
   */
    
//		if(UkeyState_M||UkeyState_U||UkeyState_D||UkeyState_T)
//		{
//			Beep_ms(30);			
//			UkeyState_M=0;
//			UkeyState_U=0;
//			UkeyState_D=0;
//			UkeyState_T=0;
//			WritePosEx(1, 3092, 1200, 0);	//舵机(ID1),以最高速度V=60步/秒,加速度A=20(50*100步/秒^2),运行至P1=3092
//			WritePosEx(2, 2709, 1000, 0);
//			WritePosEx(3, 547, 3000, 0);
//			WritePosEx(4, 3088, 2000, 0);
//			WritePosEx(5, 877, 2000, 0);

//		}
     
    /* USER CODE END WHILE */

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
void Uart_Send(uint8_t *buf , uint16_t len)
{
	HAL_UART_Transmit_DMA(&huart6,buf,len);

//	HAL_UART_Transmit(&huart6, buf, len, HAL_MAX_DELAY);
}

int16_t Uart_Read(uint8_t *buf , uint16_t len, uint32_t timeout)
{
	if(HAL_UART_Receive(&huart6, buf, len, timeout)==HAL_OK){
		return len;
	}else{
		return 0;
	}
}

void Uart_Flush_CL(void)
{
	head = tail = 0;
}

int16_t Uart_Read_CL(void)
{
	if(head!=tail){
		uint8_t Data = uartBuf[head];
		head =  (head+1)%128;
		return Data;
	}else{
		return -1;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
  //判断当进入定时器6中断时
	if(htim->Instance == TIM6)
	{
		if(Delay_Flag.Beep==1)
			t_ms.Beep++;
		if(Delay_Flag.Beep_Loop==1)
			t_ms.Beep_Loop++;
		
    //10ms处理一次PID
    PID_TimerCount++;
    if(PID_TimerCount %10 ==0)
    {
      PID_Flag = 1;
      PID_TimerCount = 0;
    }
    
    //任务时间
    Task1++;
    
		if( HAL_GPIO_ReadPin(Ukey_Mode_GPIO_Port,Ukey_Mode_Pin) == GPIO_PIN_RESET )//如果引脚检测到低电平
		{
			UkeyCount_M++; //按键按下，计数UkeyCount加1
			if(UkeyCount_M>=7) //1ms中断服务函数里运行一次，UkeyCount大于等于7，即按键已稳定按下7ms
			{
				if(UkeyFlag_M==0) //判断有没有重按键，1为有，0为没有
				{
					UkeyState_M=1; //设置按键标志
					UkeyCount_M=0;
					UkeyFlag_M=1; //设置重按键标志
				}
				else //如果重按键，则重新计数
					UkeyCount_M=0;
			}
			else //如果没有稳定按下7ms，则代表没有按下按键
				UkeyState_M=0;
		}
		else //如果一直无检测到低电平，即一直无按键按下
		{
			UkeyCount_M=0; //清零UkeyCount
			UkeyState_M=0; //清除按键标志
			UkeyFlag_M=0; //清除重按键标志
		}
		
		if( HAL_GPIO_ReadPin(Ukey_Up_GPIO_Port,Ukey_Up_Pin) == GPIO_PIN_RESET )//如果引脚检测到          低电平
		{
			UkeyCount_U++; //按键按下，计数UkeyCount加1
			if(UkeyCount_U>=7) //1ms中断服务函数里运行一次，UkeyCount大于等于7，即按键已稳定按下7ms
			{
				if(UkeyFlag_U==0) //判断有没有重按键，1为有，0为没有
				{
					UkeyState_U=1; //设置按键标志
					UkeyCount_U=0;
					UkeyFlag_U=1; //设置重按键标志
				}
				else //如果重按键，则重新计数
					UkeyCount_U=0;
			}
			else //如果没有稳定按下7ms，则代表没有按下按键
				UkeyState_U=0;
		}
		else //如果一直无检测到低电平，即一直无按键按下
		{
			UkeyCount_U=0; //清零UkeyCount
			UkeyState_U=0; //清除按键标志
			UkeyFlag_U=0; //清除重按键标志
		}
		
		if( HAL_GPIO_ReadPin(Ukey_Down_GPIO_Port,Ukey_Down_Pin) == GPIO_PIN_RESET )//如果引脚检测到低电平
		{
			UkeyCount_D++; //按键按下，计数UkeyCount加1
			if(UkeyCount_D>=7) //1ms中断服务函数里运行一次，UkeyCount大于等于7，即按键已稳定按下7ms
			{
				if(UkeyFlag_D==0) //判断有没有重按键，1为有，0为没有
				{
					UkeyState_D=1; //设置按键标志
					UkeyCount_D=0;
					UkeyFlag_D=1; //设置重按键标志
				}
				else //如果重按键，则重新计数
					UkeyCount_D=0;
			}
			else //如果没有稳定按下7ms，则代表没有按下按键
				UkeyState_D=0;
		}
		else //如果一直无检测到低电平，即一直无按键按下
		{
			UkeyCount_D=0; //清零UkeyCount
			UkeyState_D=0; //清除按键标志
			UkeyFlag_D=0; //清除重按键标志
		}
		
		if( HAL_GPIO_ReadPin(Toggle_GPIO_Port,Toggle_Pin) == GPIO_PIN_RESET )//如果引脚检测到          低电平
		{
			UkeyCount_T++; //按键按下，计数UkeyCount加1
			if(UkeyCount_T>=7) //1ms中断服务函数里运行一次，UkeyCount大于等于7，即按键已稳定按下7ms
			{
				if(UkeyFlag_T==0) //判断有没有重按键，1为有，0为没有
				{
					UkeyState_T=1; //设置按键标志
					UkeyCount_T=0;
					UkeyFlag_T=1; //设置重按键标志
				}
				else //如果重按键，则重新计数
					UkeyCount_T=0;
			}
			else //如果没有稳定按下7ms，则代表没有按下按键
				UkeyState_T=0;
		}
		else //如果一直无检测到低电平，即一直无按键按下
		{
			UkeyCount_T=0; //清零UkeyCount
			UkeyState_T=0; //清除按键标志
			UkeyFlag_T=0; //清除重按键标志
		}
	}
	
}

void RST_Enable(void)
{
	if(HAL_GPIO_ReadPin(Ukey_RST_GPIO_Port,Ukey_RST_Pin)==GPIO_PIN_RESET)
		HAL_GPIO_WritePin(NRST_GPIO_Port,NRST_Pin,GPIO_PIN_RESET);
}
void Beep_Init(void)
{
	Delay_Flag.Beep=0;
	t_ms.Beep=0;
	Flag.Beep=0;
	Delay_Flag.Beep_Loop=0;
	t_ms.Beep_Loop=0;
	Flag.Beep_Loop=0;
}
void Beep_Enable(void)
{
	if(Flag.Beep)
		HAL_GPIO_WritePin(Beep_GPIO_Port,Beep_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Beep_GPIO_Port,Beep_Pin,GPIO_PIN_RESET);
		
	if(t_ms.Beep<Beep_t)
	{
		Flag.Beep=1;
		Delay_Flag.Beep=1;
	}
	else
	{
		Flag.Beep=0;
		Delay_Flag.Beep=0;
		Beep_t=0;
		t_ms.Beep=0;
	}
	
	if(t_ms.Beep_Loop<Beep_Loop*Beep_Loop_ms)
	{
		Flag.Beep_Loop=1;
		Delay_Flag.Beep_Loop=1;
		if(t_ms.Beep_Loop>Beep_Loop_ms)
		{
			Beep_ms(Beep_Loop_t);
			Beep_Loop--;
			t_ms.Beep_Loop-=Beep_Loop_ms;
		}
	}
	else
	{
		Flag.Beep_Loop=0;
		Delay_Flag.Beep_Loop=0;
		Beep_Loop_ms=0;
		t_ms.Beep_Loop=0;
	}
}
void Beep_ms(uint16_t t)
{
	Beep_t=t;
}
void Beep_Loop_Cycle(uint16_t Beep_delay,uint8_t Loop,uint16_t Cycle)
{
	Beep_Loop_t=Beep_delay;	//响的时间		(ms)
	Beep_ms(Beep_delay);		//先响一下保证loop次数的正确
	Beep_Loop=Loop;					//响的次数
	Beep_Loop_ms=Cycle;			//响的周期		(ms)
}

void Encoder_Init(void)
{
	if(Clean_Flag == 0)	//清除启动前的编码器读数
	{
		Clean_Flag = 1;
		Clear_All_Motor_Now_Encoder_val();
		Clean_Task_Encoder_val(); 
		Task1 = 0;
	}
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
