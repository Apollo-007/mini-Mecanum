#ifndef __WT9011G4K_H_
#define __WT9011G4K_H_

#include "main.h"
#include "usart.h"
#include "jy901.h"
#include "delay.h"

//数据缓存区大小
#define RXBUFFER_LEN 11

typedef struct
{
	float angle[3];
}Angle;

typedef struct
{
		uint8_t Rx_flag;												        //接收完成标志
		uint8_t Rx_len;													        //接收长度
		uint8_t frame_head;											        //帧头
		uint8_t RxBuffer[RXBUFFER_LEN];					        //数据存储
		Angle angle;                                    //角度
}WT9011G4K;

extern WT9011G4K WT9011G4K_Data;
extern float WT9011G4K_Angle;
extern float Target_WT9011G4K_Angle;

void WT9011G4K_Init(WT9011G4K *Data);
void WT9011G4K_Process(void);
void Get_Start_WT9011G4K_Angle(void);

#endif
