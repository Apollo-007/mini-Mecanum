#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"
#include "tim.h"
#include "control.h"
#include "math.h"

//进行PWM滤波
#define MAX_Speed 1000
#define min_speed 0

#define MIN_Speed 0

//小车轮子序号
#define RF 1
#define LF 2
#define LB 3
#define RB 4

   //机械臂 
//LF 2   1 RF

//LB 3   4 RB
   
//电机的参数

//编码器线数
#define PPR   11.0
//减速比
#define RATIO 30.0
//倍频数
#define BP    4.0

//这些是可以使用的函数
void go_4(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void speed_4(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void Clean_All_Motor_Pwm(void);
void Clean_Task_Encoder_val(void);
void Clear_All_Motor_Now_Encoder_val(void);
//陀螺仪控制函数
void speed_4_WT9011G4K(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
//激光控制函数
void speed_4_LaserX(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void speed_4_LaserY(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void Choice_Laser(int16_t Reality,uint16_t Target);


//这个是底层的PWM赋值函数
void Motor_Pwm_Set(uint8_t motor,int speed);

//下面的函数在上面的函数中调用
short Read_Speed_Encoder(uint8_t TIMX);
short Read_Position_Encoder(uint8_t TIMX);

#endif
