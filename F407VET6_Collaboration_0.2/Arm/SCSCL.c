/*
 * SCSCL.c
 * 飞特SCSCL系列串行舵机应用层程序
 * 日期: 2019.7.31
 * 作者: 
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SCSCL.h"

static uint8_t MemCL[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L+1];
static int ErrCL = 0;

int getErrCL(void)
{
	return ErrCL;
}

int WritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return genWriteCL(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

int RegWritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return regWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

void RegWriteActionCL()
{
	regAction(0xfe);
}

void SyncWritePos(uint8_t ID[], uint8_t IDN, uint16_t Position[], uint16_t Time[], uint16_t Speed[])
{
  uint8_t offbuf[32*6];
	uint8_t i;
  for(i = 0; i<IDN; i++){
		uint16_t T, V;
		if(Time){
			T = Time[i];
		}else{
			T = 0;
		}
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
    Host2SCS(offbuf+i*6+0, offbuf+i*6+1, Position[i]);
    Host2SCS(offbuf+i*6+2, offbuf+i*6+3, T);
    Host2SCS(offbuf+i*6+4, offbuf+i*6+5, V);
  }
  syncWrite(ID, IDN, SCSCL_GOAL_POSITION_L, offbuf, 6);
}

int PWMMode(uint8_t ID)
{
	uint8_t bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);	
}

int WritePWM(uint8_t ID, int16_t pwmOut)
{
	uint8_t bBuf[2];
	if(pwmOut<0){
		pwmOut = -pwmOut;
		pwmOut |= (1<<10);
	}
	Host2SCS(bBuf+0, bBuf+1, pwmOut);

	return genWrite(ID, SCSCL_GOAL_TIME_L, bBuf, 2);
}
int EnableTorqueCL(uint8_t ID, uint8_t Enable)
{
	return writeByte(ID, SCSCL_TORQUE_ENABLE, Enable);
}

int unLockEpromCL(uint8_t ID)
{
	return writeByte(ID, SCSCL_LOCK, 0);
}

int LockEpromCL(uint8_t ID)
{
	return writeByte(ID, SCSCL_LOCK, 1);
}

int FeedBackCL(int ID)
{
	int nLen = Read(ID, SCSCL_PRESENT_POSITION_L, MemCL, sizeof(MemCL));
	if(nLen!=sizeof(MemCL)){
		ErrCL = 1;
		return -1;
	}
	ErrCL = 0;
	return nLen;
}
	
int ReadPosCL(int ID)
{
	int Pos = -1;
	if(ID==-1){
		Pos = MemCL[SCSCL_PRESENT_POSITION_L-SCSCL_PRESENT_POSITION_L];
		Pos <<= 8;
		Pos |= MemCL[SCSCL_PRESENT_POSITION_H-SCSCL_PRESENT_POSITION_L];
	}else{
		ErrCL = 0;
		Pos = readWord(ID, SCSCL_PRESENT_POSITION_L);
		if(Pos==-1){
			ErrCL = 1;
		}
	}
	return Pos;
}

int ReadSpeedCL(int ID)
{
	int Speed = -1;
	if(ID==-1){
		Speed = MemCL[SCSCL_PRESENT_SPEED_L-SCSCL_PRESENT_POSITION_L];
		Speed <<= 8;
		Speed |= MemCL[SCSCL_PRESENT_SPEED_H-SCSCL_PRESENT_POSITION_L];
	}else{
		ErrCL = 0;
		Speed = readWord(ID, SCSCL_PRESENT_SPEED_L);
		if(Speed==-1){
			ErrCL = 1;
			return -1;
		}
	}
	if(!ErrCL && Speed&(1<<15)){
		Speed = -(Speed&~(1<<15));
	}	
	return Speed;
}

int ReadLoadCL(int ID)
{
	int Load = -1;
	if(ID==-1){
		Load = MemCL[SCSCL_PRESENT_LOAD_L-SCSCL_PRESENT_POSITION_L];
		Load <<= 8;
		Load |= MemCL[SCSCL_PRESENT_LOAD_H-SCSCL_PRESENT_POSITION_L];
	}else{
		ErrCL = 0;
		Load = readWord(ID, SCSCL_PRESENT_LOAD_L);
		if(Load==-1){
			ErrCL = 1;
		}
	}
	if(!ErrCL && Load&(1<<10)){
		Load = -(Load&~(1<<10));
	}	
	return Load;
}

int ReadVoltageCL(int ID)
{
	int Voltage = -1;
	if(ID==-1){
		Voltage = MemCL[SCSCL_PRESENT_VOLTAGE-SCSCL_PRESENT_POSITION_L];	
	}else{
		ErrCL = 0;
		Voltage = readByte(ID, SCSCL_PRESENT_VOLTAGE);
		if(Voltage==-1){
			ErrCL = 1;
		}
	}
	return Voltage;
}

int ReadTemperCL(int ID)
{
	int Temper = -1;
	if(ID==-1){
		Temper = MemCL[SCSCL_PRESENT_TEMPERATURE-SCSCL_PRESENT_POSITION_L];	
	}else{
		ErrCL = 0;
		Temper = readByte(ID, SCSCL_PRESENT_TEMPERATURE);
		if(Temper==-1){
			ErrCL = 1;
		}
	}
	return Temper;
}

int ReadMoveCL(int ID)
{
	int Move = -1;
	if(ID==-1){
		Move = MemCL[SCSCL_MOVING-SCSCL_PRESENT_POSITION_L];	
	}else{
		ErrCL = 0;
		Move = readByte(ID, SCSCL_MOVING);
		if(Move==-1){
			ErrCL = 1;
		}
	}
	return Move;
}

int ReadCurrentCL(int ID)
{
	int Current = -1;
	if(ID==-1){
		Current = MemCL[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L];
		Current <<= 8;
		Current |= MemCL[SCSCL_PRESENT_CURRENT_L-SCSCL_PRESENT_POSITION_L];
	}else{
		ErrCL = 0;
		Current = readWord(ID, SCSCL_PRESENT_CURRENT_L);
		if(Current==-1){
			ErrCL = 1;
			return -1;
		}
	}
	if(!ErrCL && Current&(1<<15)){
		Current = -(Current&~(1<<15));
	}	
	return Current;
}
