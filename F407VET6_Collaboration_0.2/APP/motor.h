#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"
#include "tim.h"
#include "control.h"
#include "math.h"

//����PWM�˲�
#define MAX_Speed 1000
#define min_speed 0

#define MIN_Speed 0

//С���������
#define RF 1
#define LF 2
#define LB 3
#define RB 4

   //��е�� 
//LF 2   1 RF

//LB 3   4 RB
   
//����Ĳ���

//����������
#define PPR   11.0
//���ٱ�
#define RATIO 30.0
//��Ƶ��
#define BP    4.0

//��Щ�ǿ���ʹ�õĺ���
void go_4(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void speed_4(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void Clean_All_Motor_Pwm(void);
void Clean_Task_Encoder_val(void);
void Clear_All_Motor_Now_Encoder_val(void);
//�����ǿ��ƺ���
void speed_4_WT9011G4K(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
//������ƺ���
void speed_4_LaserX(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void speed_4_LaserY(int16_t motor_RF,int16_t motor_LF,int16_t motor_LB,int16_t motor_RB);
void Choice_Laser(int16_t Reality,uint16_t Target);


//����ǵײ��PWM��ֵ����
void Motor_Pwm_Set(uint8_t motor,int speed);

//����ĺ���������ĺ����е���
short Read_Speed_Encoder(uint8_t TIMX);
short Read_Position_Encoder(uint8_t TIMX);

#endif
