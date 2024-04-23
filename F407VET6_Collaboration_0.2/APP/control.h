#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "main.h"
#include "motor.h"
#include "math.h"
#include "wt9011g4k.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//运动PID函数

//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB 3  4 RB

//RF
extern int Target_Velocity_RF,Reality_Velocity_RF;   
extern int Target_Position_RF,Reality_Position_RF;   
void  RF_Position_Incremental_PID(void);
int   RF_Position_PID(int reality,int target);
int   RF_Incremental_PID(int reality,int target);

//LF
extern int Target_Velocity_LF,Reality_Velocity_LF;   
extern int Target_Position_LF,Reality_Position_LF;   
void  LF_Position_Incremental_PID(void);
int   LF_Position_PID(int reality,int target);
int   LF_Incremental_PID(int reality,int target);

//LB
extern int Target_Velocity_LB,Reality_Velocity_LB;   
extern int Target_Position_LB,Reality_Position_LB;
void  LB_Position_Incremental_PID(void);
int   LB_Position_PID(int reality,int target);
int   LB_Incremental_PID(int reality,int target);

//RB
extern int Target_Velocity_RB,Reality_Velocity_RB;   
extern int Target_Position_RB,Reality_Position_RB;  
void  RB_Position_Incremental_PID(void);
int   RB_Position_PID(int reality,int target);
int   RB_Incremental_PID(int reality,int target);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//陀螺仪PID函数

//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB 3  4 RB

//RF
extern int Target_Velocity_RF_WT9011G4K,Reality_Velocity_RF_WT9011G4K;
void  RF_Position_Incremental_PID_WT9011G4K(void);
int   RF_Position_PID_WT9011G4K(float reality,float target);
int   RF_Incremental_PID_WT9011G4K(int reality,int target);

//LF
extern int Target_Velocity_LF_WT9011G4K,Reality_Velocity_LF_WT9011G4K;   
void  LF_Position_Incremental_PID_WT9011G4K(void);
int   LF_Position_PID_WT9011G4K(float reality,float target);
int   LF_Incremental_PID_WT9011G4K(int reality,int target);

//LB
extern int Target_Velocity_LB_WT9011G4K,Reality_Velocity_LB_WT9011G4K;
void  LB_Position_Incremental_PID_WT9011G4K(void);
int   LB_Position_PID_WT9011G4K(float reality,float target);
int   LB_Incremental_PID_WT9011G4K(int reality,int target);

//RB
extern int Target_Velocity_RB_WT9011G4K,Reality_Velocity_RB_WT9011G4K;   
void  RB_Position_Incremental_PID_WT9011G4K(void);
int   RB_Position_PID_WT9011G4K(float reality,float target);
int   RB_Incremental_PID_WT9011G4K(int reality,int target);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//激光测距PID函数

//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB 3  4 RB

//RF
extern int Target_Velocity_RF_LASER,Reality_Velocity_RF_LASER;
extern int Target_Position_RF_LASER,Reality_Position_RF_LASER;
void  RF_Position_Incremental_PID_LASER(void);
int   RF_Position_PID_LASER(float reality,float target);
int   RF_Incremental_PID_LASER(int reality,int target);

//LF
extern int Target_Velocity_LF_LASER,Reality_Velocity_LF_LASER;
extern int Target_Position_LF_LASER,Reality_Position_LF_LASER;
void  LF_Position_Incremental_PID_LASER(void);
int   LF_Position_PID_LASER(float reality,float target);
int   LF_Incremental_PID_LASER(int reality,int target);

//LB
extern int Target_Velocity_LB_LASER,Reality_Velocity_LB_LASER; 
extern int Target_Position_LB_LASER,Reality_Position_LB_LASER;
void LB_Position_Incremental_PID_LASER(void);
int LB_Position_PID_LASER(float reality,float target);
int LB_Incremental_PID_LASER(int reality,int target);

//RB
extern int Target_Velocity_RB_LASER,Reality_Velocity_RB_LASER;   
extern int Target_Position_RB_LASER,Reality_Position_RB_LASER;
void RB_Position_Incremental_PID_LASER(void);
int RB_Position_PID_LASER(float reality,float target);
int RB_Incremental_PID_LASER(int reality,int target);


//PID通用函数
int Xianfu(int value,int Amplitude);
int Abs(int a);

#endif
