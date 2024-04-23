#ifndef __LASER_H_
#define __LASER_H_

#include "main.h"
#include "usart.h"
#include "control.h"

//���ݻ�������С
#define Laser_RXBUFFER_LEN  11

//���ݵĴ�С
#define Laser_Data_Buffer   7

//PWM�޷� 
#define MAX_PWM             600
typedef struct
{
		uint8_t Rx_flag;												        //������ɱ�־
		uint8_t Rx_len;													        //���ճ���
		uint8_t frame_head;											        //֡ͷ
		uint8_t RxBuffer[Laser_RXBUFFER_LEN];					  //���ݴ洢
}LASER;

extern LASER LaserX;
extern LASER LaserY;

extern int Final_LaserX_Data;
extern int Final_LaserY_Data;

void LaserX_Init(LASER *Data);
void LaserY_Init(LASER *Data);
void LaserX_Process(void);
void LaserY_Process(void);

#endif
