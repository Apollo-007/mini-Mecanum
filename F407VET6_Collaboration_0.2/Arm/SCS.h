/*
 * SCS.h
 * ���ش��ж��ͨ��Э�����
 * ����: 2022.3.30
 * ����: 
 */

#ifndef _SCS_H
#define _SCS_H

#include <stdint.h>

extern int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//��ͨдָ��
extern int genWriteCL(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//��ͨдָ��
extern int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//�첽дָ��
extern int regAction(uint8_t ID);//�첽дִ����
extern void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//ͬ��дָ��
extern int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);//д1���ֽ�
extern int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);//д2���ֽ�
extern int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);//��ָ��
extern int readByte(uint8_t ID, uint8_t MemAddr);//��1���ֽ�
extern int readWord(uint8_t ID, uint8_t MemAddr);//��2���ֽ�
extern int Ping(uint8_t ID);//Pingָ��
extern uint8_t getSCSErr(void);//���ض��״̬��
extern int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen);//ͬ����ָ�������
extern int syncReadPacketRx(uint8_t ID, uint8_t *nDat);//ͬ�������ذ����룬�ɹ������ڴ��ֽ�����ʧ�ܷ���0
extern int syncReadRxPacketToByte(void);//����һ���ֽ�
extern int syncReadRxPacketToWrod(uint8_t negBit);//���������ֽڣ�negBitΪ����Ϊ��negBit=0��ʾ�޷���
extern void syncReadBegin(uint8_t IDN, uint8_t rxLen);//ͬ������ʼ
extern void syncReadEnd(void);//ͬ��������

extern void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);
extern void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data);//1��16λ�����Ϊ2��8λ��
extern int SCS2Host(uint8_t DataL, uint8_t DataH);//2��8λ�����Ϊ1��16λ��
extern int Ack(uint8_t ID);//Ӧ��
extern int checkHead(void);//֡ͷ���

//Ӳ���ӿں���
extern int writeSCS(uint8_t *nDat, int nLen);
extern int writeByteSCS(unsigned char bDat);
extern int readSCS(uint8_t *nDat, int nLen);
extern void rFlushSCS(void);
extern void rFlushSCS_CL(void);
extern void wFlushSCS(void);

#endif
