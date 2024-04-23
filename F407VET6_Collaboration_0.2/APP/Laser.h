#ifndef __LASER_H_
#define __LASER_H_

#include "main.h"
#include "usart.h"
#include "control.h"

//数据缓存区大小
#define Laser_RXBUFFER_LEN  11

//数据的大小
#define Laser_Data_Buffer   7

//PWM限幅 
#define MAX_PWM             600
typedef struct
{
		uint8_t Rx_flag;												        //接收完成标志
		uint8_t Rx_len;													        //接收长度
		uint8_t frame_head;											        //帧头
		uint8_t RxBuffer[Laser_RXBUFFER_LEN];					  //数据存储
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
