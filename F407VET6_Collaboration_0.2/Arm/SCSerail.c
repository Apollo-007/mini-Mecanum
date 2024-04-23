/*
 * SCServo.c
 * ���ض��Ӳ���ӿڲ����
 * ����: 2022.3.30
 * ����: 
 */

#include "stm32f4xx.h"
#include "main.h"

uint32_t IOTimeOut = 100;//���������ʱ
uint8_t wBuf[128];
uint8_t wLen = 0;

//UART �������ݽӿ�
int readSCS(unsigned char *nDat, int nLen)
{
	return Uart_Read(nDat, nLen, IOTimeOut);
}

//UART �������ݽӿ�
int writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		if(wLen<sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
		}
	}
	return wLen;
}

int writeByteSCS(unsigned char bDat)
{
	if(wLen<sizeof(wBuf)){
		wBuf[wLen] = bDat;
		wLen++;
	}
	return wLen;
}

//�ȴ���������л�(Լ8us)
void nopDelay(void)
{
	uint16_t i = 500;
	while(i--);
}

//���ջ�����ˢ��
void rFlushSCS()
{
	nopDelay();
}

void rFlushSCS_CL()
{
	Uart_Flush_CL();
}


//���ͻ�����ˢ��
void wFlushSCS()
{
	if(wLen){
		Uart_Send(wBuf, wLen);
		wLen = 0;
	}
}
