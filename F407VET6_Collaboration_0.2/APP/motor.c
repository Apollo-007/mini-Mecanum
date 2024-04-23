#include "motor.h"

//电机位置示意图
   //机械臂 
//LF 2   1 RF

//LB 3   4 RB

/*电机PWM赋值
*motor：选择电机(1,2,3,4)
*speed：PWM[-1000--+1000]
*/
void Motor_Pwm_Set(uint8_t motor,int speed)
{
  switch(motor)
  {
    case 1:
      if(speed > 0||speed == 0)
        {
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,MIN_Speed);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,speed);
        }
        else
        {
          speed=-speed;
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,MIN_Speed);
        }  
      break;
    case 2:
      if(speed > 0||speed == 0)
        {
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,speed);
          __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MIN_Speed);
        }
        else
        {
          speed=-speed;
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MIN_Speed);
          __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,speed);
        }  
      break;
    case 3:
      if(speed > 0||speed == 0)
        {
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,speed);
          __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,MIN_Speed);
        }
        else
        {
          speed=-speed;
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,MIN_Speed);
          __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,speed);
        }
      break;
    case 4:
      if(speed > 0||speed == 0)
        {
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,MIN_Speed);
        }
        else
        {
          speed=-speed;
          if(speed>MAX_Speed)
            speed = MAX_Speed;
          if(speed <min_speed)
            speed = min_speed;
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,MIN_Speed);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);
        }  
      break;
    default:
      break;
  }
  
}


/**************************************************************************
函数功能：把所有电机都停止
入口参数：无
返回  值：无
**************************************************************************/
void Clean_All_Motor_Pwm(void)
{
  Motor_Pwm_Set(RF,0);
  Motor_Pwm_Set(LF,0);
  Motor_Pwm_Set(LB,0);
  Motor_Pwm_Set(RB,0);
}
  
/**************************************************************************
函数功能：清除每次任务后的编码器累计值
入口参数：无
返回  值：无
**************************************************************************/
void Clean_Task_Encoder_val(void)
{
  Reality_Position_RF = 0;
  Reality_Position_LF = 0;
  Reality_Position_LB = 0;
  Reality_Position_RB = 0;
}

/**************************************************************************
函数功能：清除当前编码器数值
入口参数：无
返回  值：无
**************************************************************************/
void Clear_All_Motor_Now_Encoder_val(void)
{
  TIM2 -> CNT = 0;
  TIM3 -> CNT = 0;
  TIM4 -> CNT = 0;
  TIM5 -> CNT = 0;
}

/**************************************************************************
函数功能：设定运动的目标值
入口参数：每个轮子要走的距离，单位为累计的脉冲数
返回  值：无
**************************************************************************/
void go_4(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB)
{
   Target_Position_RF = motor_RF;
   Target_Position_LF = motor_LF;
   Target_Position_LB = motor_LB;
   Target_Position_RB = motor_RB;
}

/**************************************************************************
函数功能：设定轮子的速度
入口参数：速度，单位为单位时间脉冲数(0-60)
返回  值：无
**************************************************************************/
void speed_4(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB)
{
  Target_Velocity_RF = motor_RF;
  Target_Velocity_LF = motor_LF;
  Target_Velocity_LB = motor_LB;
  Target_Velocity_RB = motor_RB;
}

/**************************************************************************
函数功能：设定进行陀螺仪微调时，轮子的速度
入口参数：速度，单位为单位时间脉冲数(0-60)
返回  值：无
**************************************************************************/
void speed_4_WT9011G4K(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB)
{
  Target_Velocity_RF_WT9011G4K = -motor_RF;
  Target_Velocity_LF_WT9011G4K = -motor_LF;
  Target_Velocity_LB_WT9011G4K = -motor_LB;
  Target_Velocity_RB_WT9011G4K = -motor_RB;
}

/**************************************************************************
函数功能：设置X轴微调时的速度
入口参数：速度，单位为单位时间脉冲数(0-60)
返回  值：无
**************************************************************************/
void speed_4_LaserX(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB)
{
  Target_Velocity_RF_LASER = -motor_RF;
  Target_Velocity_LF_LASER = -motor_LF;
  Target_Velocity_LB_LASER =  motor_LB;
  Target_Velocity_RB_LASER =  motor_LB;
}

/**************************************************************************
函数功能：设置Y轴微调时的速度
入口参数：速度，单位为单位时间脉冲数(0-60)
返回  值：无
**************************************************************************/
void speed_4_LaserY(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB)
{
  Target_Velocity_RF_LASER = -motor_RF;
  Target_Velocity_LF_LASER =  motor_LF;
  Target_Velocity_LB_LASER =  motor_LB;
  Target_Velocity_RB_LASER = -motor_LB;
}

/**************************************************************************
函数功能：选择那个轴进行微调，调整指定位置
入口参数：Reality实际值，Target目标值
返回  值：无
**************************************************************************/
void Choice_Laser(int16_t Reality,uint16_t Target)
{
  Reality_Position_RF_LASER = Reality;  
  Reality_Position_LF_LASER = Reality;
  Reality_Position_LB_LASER = Reality;
  Reality_Position_RB_LASER = Reality;
  Target_Position_RF_LASER  = Target;
  Target_Position_LF_LASER  = Target;
  Target_Position_LB_LASER  = Target;
  Target_Position_RB_LASER  = Target;
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：单位时间编码器读数
**************************************************************************/
short Read_Speed_Encoder(uint8_t TIMX)
{
   short Encoder_TIM;    
   switch(TIMX)
	 {  
      //LB
	    case 2:   Encoder_TIM = -(short)TIM2 -> CNT;  TIM2 -> CNT = 0;  	 break;
      //LF
      case 3:   Encoder_TIM = -(short)TIM3 -> CNT;  TIM3 -> CNT = 0; 	 break;	
      //RB
      case 4:   Encoder_TIM = -(short)TIM4 -> CNT;  TIM4 -> CNT = 0;	   break;	
      //RF
      case 5:   Encoder_TIM = -(short)TIM5 -> CNT;  TIM5 -> CNT = 0;	   break;	
     
      default:  Encoder_TIM = 0;break;
	 }
		return Encoder_TIM;
}


/**************************************************************************
函数功能：读取位置信息
入口参数：定时器
返回  值：累计时间编码器读数
**************************************************************************/
short Read_Position_Encoder(uint8_t TIMX)
{
   short Encoder_TIM;    
   switch(TIMX)
	 {
      //LB
	    case 2:   Encoder_TIM = -(short)TIM2 -> CNT;   	 break;
      //LF
      case 3:   Encoder_TIM = -(short)TIM3 -> CNT;   	 break;
      //RB
      case 4:   Encoder_TIM = -(short)TIM4 -> CNT;   	 break;	
      //RF
      case 5:   Encoder_TIM = -(short)TIM5 -> CNT;   	 break;	
     
      default:  Encoder_TIM = 0;break;
	 }
		return Encoder_TIM;
}

