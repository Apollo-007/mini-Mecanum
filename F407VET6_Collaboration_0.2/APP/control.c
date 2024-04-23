#include "control.h"

/* 系统统一转换为脉冲数进行处理，脉冲数分辨率高，控制精度高 */


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//4个轮子的PID参数，串级PID
float Position_KP_RF=1,Position_KI_RF=0.001,Position_KD_RF=7.7; 
float Incremental_KP_RF=80.0,Incremental_KI_RF=4,Incremental_KD_RF=0.0;   

float Position_KP_LF=1,Position_KI_LF=0.0009,Position_KD_LF=8; 
float Incremental_KP_LF=135.0,Incremental_KI_LF=1.5,Incremental_KD_LF=0.0; 

float Position_KP_LB=1,Position_KI_LB=0.001,Position_KD_LB=5.6; 
float Incremental_KP_LB=90.0,Incremental_KI_LB=3.0,Incremental_KD_LB=0.0;   

float Position_KP_RB=1,Position_KI_RB=0.001,Position_KD_RB=7; 
float Incremental_KP_RB=120.0,Incremental_KI_RB=2.3,Incremental_KD_RB=0.0;   

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//电机位置示意图
   //机械臂 
//LF 2  1->RF

//LB 3  4 RB
/* 目标速度，实际速度 */
int Target_Velocity_RF=0,Reality_Velocity_RF=0;   
/* 目标位置，实际位置 */
int Target_Position_RF=0,Reality_Position_RF=0;   

/**
  * @brief  串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void RF_Position_Incremental_PID(void)
{
  static int RF_Pwm = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_RF = Read_Speed_Encoder(5);
  /* 实际位置脉冲数 */
  Reality_Position_RF += Reality_Velocity_RF;
  /* 位置式位置控制 */    
  RF_Pwm = RF_Position_PID(Reality_Position_RF,Target_Position_RF);
  /* 位置环输出限幅 */
  RF_Pwm = Xianfu(RF_Pwm,Target_Velocity_RF);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_RF-Target_Position_RF)<5)             
  {
      /* 停止输出 */
      Motor_Pwm_Set(RF,0);                                         
  }
  else
  {   
      /* 增量式速度控制 */
//      RF_Pwm = RF_Incremental_PID(Reality_Velocity_RF,Target_Velocity_RF); 
      RF_Pwm = RF_Incremental_PID(Reality_Velocity_RF,RF_Pwm);      
      /* 赋值 */
      Motor_Pwm_Set(RF,RF_Pwm);                                      
  }
}

int RF_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                        /* 计算偏差 */
    err_sum += err;	                             /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;            /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RF*err)                   /* 比例环节 */
         +(Position_KI_RF*err_sum)               /* 积分环节 */
         +(Position_KD_RF*(err-err_1));          /* 微分环节 */
    err_1=err;                                   /* 保存上次偏差*/
    return Pwm;                                 /* 输出结果 */
}
int RF_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err=target-reality;                                /* 计算偏差 */
	 Pwm += (Incremental_KP_RF*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_RF*err)                   /* 积分环节 */
           +(Incremental_KD_RF*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                       /* 保存上上次偏差 */
	 err_1=err;	                                        /* 保存上一次偏差 */
	 return Pwm;                                       /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF->2  1 RF

//LB 3  4 RB
/* 目标速度，实际速度 */
int Target_Velocity_LF=0,Reality_Velocity_LF=0;   
/* 目标位置，实际位置 */
int Target_Position_LF=0,Reality_Position_LF=0;   

/**
  * @brief  串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void LF_Position_Incremental_PID(void)
{
  static int LF_Pwm = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_LF = Read_Speed_Encoder(3);
  /* 实际位置脉冲数 */
  Reality_Position_LF += Reality_Velocity_LF;
  /* 位置式位置控制 */    
  LF_Pwm = LF_Position_PID(Reality_Position_LF,Target_Position_LF);
  /* 位置环输出限幅 */
  LF_Pwm = Xianfu(LF_Pwm,Target_Velocity_LF);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_LF-Target_Position_LF)<5)             
  {
      /* 停止输出 */
      Motor_Pwm_Set(LF,0);                                         
  }
  else
  {   
      /* 增量式速度控制 */
//      LF_Pwm = LF_Incremental_PID(Reality_Velocity_LF,Target_Velocity_LF);
      LF_Pwm = LF_Incremental_PID(Reality_Velocity_LF,LF_Pwm);      
      /* 赋值 */
      Motor_Pwm_Set(LF,LF_Pwm);                                      
  }
}
 
int LF_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                        /* 计算偏差 */
    err_sum += err;	                             /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;            /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LF*err)                   /* 比例环节 */
         +(Position_KI_LF*err_sum)               /* 积分环节 */
         +(Position_KD_LF*(err-err_1));          /* 微分环节 */
    err_1=err;                                   /* 保存上次偏差*/
    return Pwm;                                 /* 输出结果 */
}

int LF_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err=target-reality;                                /* 计算偏差 */
	 Pwm += (Incremental_KP_LF*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_LF*err)                   /* 积分环节 */
           +(Incremental_KD_LF*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                       /* 保存上上次偏差 */
	 err_1=err;	                                        /* 保存上一次偏差 */
	 return Pwm;                                       /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//电机位置示意图
  //机械臂 
//LF 2  1 RF

//LB->3  4 RB

/* 目标速度，实际速度 */
int Target_Velocity_LB=0,Reality_Velocity_LB=0;   
/* 目标位置，实际位置 */
int Target_Position_LB=0,Reality_Position_LB=0;

/**
  * @brief  串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void LB_Position_Incremental_PID(void)
{
  static int LB_Pwm = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_LB = Read_Speed_Encoder(2);
  /* 实际位置脉冲数 */
  Reality_Position_LB += Reality_Velocity_LB;
  /* 位置式位置控制 */    
  LB_Pwm = LB_Position_PID(Reality_Position_LB,Target_Position_LB);
  /* 位置环输出限幅 */
  LB_Pwm = Xianfu(LB_Pwm,Target_Velocity_LB);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_LB-Target_Position_LB)<5)             
  {
      /* 停止输出 */
      Motor_Pwm_Set(LB,0);                                         
  }
  else
  {   
      /* 增量式速度控制 */
//        LB_Pwm = LB_Incremental_PID(Reality_Velocity_LB,Target_Velocity_LB);     
      LB_Pwm = LB_Incremental_PID(Reality_Velocity_LB,LB_Pwm);      
      /* 赋值 */
      Motor_Pwm_Set(LB,LB_Pwm);                                      
  }
  
}

int LB_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                      /* 计算偏差 */
    err_sum += err;	                             /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;            /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LB*err)                   /* 比例环节 */
         +(Position_KI_LB*err_sum)               /* 积分环节 */
         +(Position_KD_LB*(err-err_1));          /* 微分环节 */
    err_1=err;                                   /* 保存上次偏差*/
    return Pwm;                                 /* 输出结果 */
}

int LB_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                /* 计算偏差 */
	 Pwm += (Incremental_KP_LB*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_LB*err)                   /* 积分环节 */
           +(Incremental_KD_LB*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                       /* 保存上上次偏差 */
	 err_1=err;	                                        /* 保存上一次偏差 */
	 return Pwm;                                       /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB 3  4->RB
/* 目标速度，实际速度 */
int Target_Velocity_RB=0,Reality_Velocity_RB=0;   
/* 目标位置，实际位置 */
int Target_Position_RB=0,Reality_Position_RB=0;   

/**
  * @brief  串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void RB_Position_Incremental_PID(void)
{
  static int RB_Pwm = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_RB = Read_Speed_Encoder(4);
  /* 实际位置脉冲数 */
  Reality_Position_RB += Reality_Velocity_RB;
  /* 位置式位置控制 */    
  RB_Pwm = RB_Position_PID(Reality_Position_RB,Target_Position_RB);
  /* 位置环输出限幅 */
  RB_Pwm = Xianfu(RB_Pwm,Target_Velocity_RB);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_RB-Target_Position_RB)<5)             
  {
      /* 停止输出 */
      Motor_Pwm_Set(RB,0);                                         
  }
  else
  {   
      /* 增量式速度控制 */
//      RB_Pwm = RB_Incremental_PID(Reality_Velocity_RB,Target_Velocity_RB); 
      RB_Pwm = RB_Incremental_PID(Reality_Velocity_RB,RB_Pwm);      
      /* 赋值 */
      Motor_Pwm_Set(RB,RB_Pwm);                                      
  }
}

int RB_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                        /* 计算偏差 */
    err_sum += err;	                             /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;            /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RB*err)                   /* 比例环节 */
         +(Position_KI_RB*err_sum)               /* 积分环节 */
         +(Position_KD_RB*(err-err_1));          /* 微分环节 */
    err_1=err;                                   /* 保存上次偏差*/
    return Pwm;                                 /* 输出结果 */
}

int RB_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                            /* 计算偏差 */
	 Pwm += (Incremental_KP_RB*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_RB*err)                   /* 积分环节 */
           +(Incremental_KD_RB*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                       /* 保存上上次偏差 */
	 err_1=err;	                                        /* 保存上一次偏差 */
	 return Pwm;                                       /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//4个轮子的PID参数，陀螺仪PID

float Position_KP_RF_WT9011G4K=1.5,Position_KI_RF_WT9011G4K=0.0,Position_KD_RF_WT9011G4K=1; 
float Incremental_KP_RF_WT9011G4K=100.0,Incremental_KI_RF_WT9011G4K=15.0,Incremental_KD_RF_WT9011G4K=0.0;   

float Position_KP_LF_WT9011G4K=1.5,Position_KI_LF_WT9011G4K=0.0,Position_KD_LF_WT9011G4K=1; 
float Incremental_KP_LF_WT9011G4K=100.0,Incremental_KI_LF_WT9011G4K=15.0,Incremental_KD_LF_WT9011G4K=0.0; 

float Position_KP_LB_WT9011G4K=1.5,Position_KI_LB_WT9011G4K=0.0,Position_KD_LB_WT9011G4K=1; 
float Incremental_KP_LB_WT9011G4K=100.0,Incremental_KI_LB_WT9011G4K=15.0,Incremental_KD_LB_WT9011G4K=0.0;   

float Position_KP_RB_WT9011G4K=1.5,Position_KI_RB_WT9011G4K=0.0,Position_KD_RB_WT9011G4K=1; 
float Incremental_KP_RB_WT9011G4K=100.0,Incremental_KI_RB_WT9011G4K=15.0,Incremental_KD_RB_WT9011G4K=0.0;   

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//陀螺仪PID处理

//电机位置示意图
   //机械臂 
//LF 2  1->RF

//LB 3  4 RB

/* 目标速度，实际速度 */
int Target_Velocity_RF_WT9011G4K=0,Reality_Velocity_RF_WT9011G4K=0;

/**
  * @brief  陀螺仪串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void RF_Position_Incremental_PID_WT9011G4K(void)
{
  static int RF_Pwm_WT9011G4K = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_RF_WT9011G4K = Read_Speed_Encoder(5);
  /* 位置式位置控制 */ 
  RF_Pwm_WT9011G4K = RF_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* 位置环输出限幅 */
  RF_Pwm_WT9011G4K = Xianfu(RF_Pwm_WT9011G4K,Target_Velocity_RF_WT9011G4K);
  /* 滤除部分干扰 */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(RF,0);
  }
  else
  {
    /* 增量式速度控制 */
//    RF_Pwm_WT9011G4K = RF_Incremental_PID_WT9011G4K(Reality_Velocity_RF_WT9011G4K,Target_Velocity_RF_WT9011G4K); 
    RF_Pwm_WT9011G4K = RF_Incremental_PID_WT9011G4K(Reality_Velocity_RF_WT9011G4K,RF_Pwm_WT9011G4K);      
    /* 赋值 */
    Motor_Pwm_Set(RF,RF_Pwm_WT9011G4K);                                      
  }
}

int RF_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    
    err = target - reality;                                /* 计算偏差 */
    err_sum += err;	                                       /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                      /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RF_WT9011G4K*err)                   /* 比例环节 */
         +(Position_KI_RF_WT9011G4K*err_sum)               /* 积分环节 */
         +(Position_KD_RF_WT9011G4K*(err-err_1));          /* 微分环节 */
    err_1=err;                                             /* 保存上次偏差*/
    return Pwm;                                           /* 输出结果 */
}

int RF_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err= target - reality;                                       /* 计算偏差 */
	 Pwm += (Incremental_KP_RF_WT9011G4K*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_RF_WT9011G4K*err)                   /* 积分环节 */
           +(Incremental_KD_RF_WT9011G4K*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                                 /* 保存上上次偏差 */
	 err_1=err;	                                                  /* 保存上一次偏差 */
	 return Pwm;                                                 /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF->2  1 RF

//LB 3  4 RB

/* 目标速度，实际速度 */
int Target_Velocity_LF_WT9011G4K=0,Reality_Velocity_LF_WT9011G4K=0;   

/**
  * @brief  陀螺仪串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void LF_Position_Incremental_PID_WT9011G4K(void)
{
  static int LF_Pwm_WT9011G4K = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_LF_WT9011G4K = Read_Speed_Encoder(3);
  /* 位置式位置控制 */    
  LF_Pwm_WT9011G4K = LF_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* 位置环输出限幅 */
  LF_Pwm_WT9011G4K = Xianfu(LF_Pwm_WT9011G4K,Target_Velocity_LF_WT9011G4K);
  /* 滤除部分干扰 */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(LF,0);
  }
  else
  {
    /* 增量式速度控制 */
//    LF_Pwm_WT9011G4K = LF_Incremental_PID_WT9011G4K(Reality_Velocity_LF_WT9011G4K,Target_Velocity_LF_WT9011G4K);
    LF_Pwm_WT9011G4K = LF_Incremental_PID_WT9011G4K(Reality_Velocity_LF_WT9011G4K,LF_Pwm_WT9011G4K);      
    /* 赋值 */
    Motor_Pwm_Set(LF,LF_Pwm_WT9011G4K);                                      
  }

}
 
int LF_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                                  /* 计算偏差 */
    err_sum += err;	                                       /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                      /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LF_WT9011G4K*err)                   /* 比例环节 */
         +(Position_KI_LF_WT9011G4K*err_sum)               /* 积分环节 */
         +(Position_KD_LF_WT9011G4K*(err-err_1));          /* 微分环节 */
    err_1=err;                                             /* 保存上次偏差*/
    return Pwm;                                           /* 输出结果 */
}

int LF_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                          /* 计算偏差 */
	 Pwm += (Incremental_KP_LF_WT9011G4K*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_LF_WT9011G4K*err)                   /* 积分环节 */
           +(Incremental_KD_LF_WT9011G4K*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                                 /* 保存上上次偏差 */
	 err_1=err;	                                                  /* 保存上一次偏差 */
	 return Pwm;                                                 /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB->3  4 RB

/* 目标速度，实际速度 */
int Target_Velocity_LB_WT9011G4K=0,Reality_Velocity_LB_WT9011G4K=0;   

/**
  * @brief  陀螺仪串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */

void LB_Position_Incremental_PID_WT9011G4K(void)
{
  static int LB_Pwm_WT9011G4K = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_LB_WT9011G4K = Read_Speed_Encoder(2);
  /* 位置式位置控制 */    
  LB_Pwm_WT9011G4K = LB_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* 位置环输出限幅 */
  LB_Pwm_WT9011G4K = Xianfu(LB_Pwm_WT9011G4K,Target_Velocity_LB_WT9011G4K);
  /* 滤除部分干扰 */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(LB,0);      
  }
  else
  {
    /* 增量式速度控制 */
//    LB_Pwm_WT9011G4K = LB_Incremental_PID_WT9011G4K(Reality_Velocity_LB_WT9011G4K,Target_Velocity_LB_WT9011G4K);     
    LB_Pwm_WT9011G4K = LB_Incremental_PID_WT9011G4K(Reality_Velocity_LB_WT9011G4K,LB_Pwm_WT9011G4K);      
    /* 赋值 */
    Motor_Pwm_Set(LB,LB_Pwm_WT9011G4K);                                      
  }
}

int LB_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                                /* 计算偏差 */
    err_sum += err;	                                       /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                      /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LB_WT9011G4K*err)                   /* 比例环节 */
         +(Position_KI_LB_WT9011G4K*err_sum)               /* 积分环节 */
         +(Position_KD_LB_WT9011G4K*(err-err_1));          /* 微分环节 */
    err_1=err;                                             /* 保存上次偏差*/
    return Pwm;                                           /* 输出结果 */
}

int LB_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* 计算偏差 */
	 Pwm += (Incremental_KP_LB_WT9011G4K*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_LB_WT9011G4K*err)                   /* 积分环节 */
           +(Incremental_KD_LB_WT9011G4K*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                                 /* 保存上上次偏差 */
	 err_1=err;	                                                  /* 保存上一次偏差 */
	 return Pwm;                                                 /* 输出结果 */
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB 3  4->RB

/* 目标速度，实际速度 */
int Target_Velocity_RB_WT9011G4K=0,Reality_Velocity_RB_WT9011G4K=0;   
  
/**
  * @brief  陀螺仪串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void RB_Position_Incremental_PID_WT9011G4K(void)
{
  static int RB_Pwm_WT9011G4K = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_RB_WT9011G4K = Read_Speed_Encoder(4);
  /* 位置式位置控制 */   
  RB_Pwm_WT9011G4K = RB_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* 位置环输出限幅 */
  RB_Pwm_WT9011G4K = Xianfu(RB_Pwm_WT9011G4K,Target_Velocity_RB_WT9011G4K);
  /* 滤除部分干扰 */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(RB,0);
  }
  else
  {
    /* 增量式速度控制 */ 
//    RB_Pwm_WT9011G4K = RB_Incremental_PID_WT9011G4K(Reality_Velocity_RB_WT9011G4K,Target_Velocity_RB_WT9011G4K); 
    RB_Pwm_WT9011G4K = RB_Incremental_PID_WT9011G4K(Reality_Velocity_RB_WT9011G4K,RB_Pwm_WT9011G4K);      
    /* 赋值 */
    Motor_Pwm_Set(RB,RB_Pwm_WT9011G4K);     
  }                                
}

int RB_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                               /* 计算偏差 */
    err_sum += err;	                                      /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                     /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RB_WT9011G4K*err)                   /* 比例环节 */
         +(Position_KI_RB_WT9011G4K*err_sum)               /* 积分环节 */
         +(Position_KD_RB_WT9011G4K*(err-err_1));          /* 微分环节 */
    err_1=err;                                             /* 保存上次偏差*/
    return Pwm;                                           /* 输出结果 */
}

int RB_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* 计算偏差 */
	 Pwm += (Incremental_KP_RB_WT9011G4K*(err-err_1))             /* 比例环节 */
           +(Incremental_KI_RB_WT9011G4K*err)                   /* 积分环节 */
           +(Incremental_KD_RB_WT9011G4K*(err-2*err_1+err_2));  /* 微分环节 */ 
   err_2=err_1;                                                 /* 保存上上次偏差 */
	 err_1=err;	                                                  /* 保存上一次偏差 */
	 return Pwm;                                                 /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//4个轮子的PID参数，激光PID

float Position_KP_RF_LASER=10,Position_KI_RF_LASER=0.0,Position_KD_RF_LASER=200; 
float Incremental_KP_RF_LASER=100.0,Incremental_KI_RF_LASER=15,Incremental_KD_RF_LASER=0.0;   

float Position_KP_LF_LASER=10,Position_KI_LF_LASER=0.0,Position_KD_LF_LASER=200; 
float Incremental_KP_LF_LASER=100.0,Incremental_KI_LF_LASER=15,Incremental_KD_LF_LASER=0.0; 

float Position_KP_LB_LASER=10,Position_KI_LB_LASER=0.0,Position_KD_LB_LASER=200; 
float Incremental_KP_LB_LASER=100.0,Incremental_KI_LB_LASER=15,Incremental_KD_LB_LASER=0.0;   

float Position_KP_RB_LASER=10,Position_KI_RB_LASER=0.0,Position_KD_RB_LASER=200; 
float Incremental_KP_RB_LASER=100.0,Incremental_KI_RB_LASER=15,Incremental_KD_RB_LASER=0.0;   

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF 2  1->RF

//LB 3  4 RB

/* 目标速度，实际速度 */
int Target_Velocity_RF_LASER=0,Reality_Velocity_RF_LASER=0;
/* 目标位置，实际位置*/
int Target_Position_RF_LASER=0,Reality_Position_RF_LASER=0;

/**
  * @brief  激光串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void RF_Position_Incremental_PID_LASER(void)
{
  static int RF_Pwm_LASER = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_RF_LASER = Read_Speed_Encoder(5);
  /* 位置式位置控制 */ 
  RF_Pwm_LASER = RF_Position_PID_LASER(Reality_Position_RF_LASER,Target_Position_RF_LASER);
  /* 位置环输出限幅 */
  RF_Pwm_LASER = Xianfu(RF_Pwm_LASER,Target_Velocity_RF_LASER);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_RF_LASER - Target_Position_RF_LASER) < 6)
  {
    Motor_Pwm_Set(RF,0);
  }
  else
  {
    /* 增量式速度控制 */
//    RF_Pwm_LASER = RF_Incremental_PID_LASER(Reality_Velocity_RF_LASER,Target_Velocity_RF_LASER); 
    RF_Pwm_LASER = RF_Incremental_PID_LASER(Reality_Velocity_RF_LASER,RF_Pwm_LASER);      
    /* 赋值 */
    Motor_Pwm_Set(RF,RF_Pwm_LASER);                                      
  }
}

int RF_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    
    err = target - reality;                                 /* 计算偏差 */
    err_sum += err;	                                        /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                       /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RF_LASER*err)                        /* 比例环节 */
         +(Position_KI_RF_LASER*err_sum)                    /* 积分环节 */
         +(Position_KD_RF_LASER*(err-err_1));               /* 微分环节 */
    err_1=err;                                              /* 保存上次偏差*/
    return Pwm;                                            /* 输出结果 */
}

int RF_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err= target - reality;                                       /* 计算偏差 */
	 Pwm += (Incremental_KP_RF_LASER*(err-err_1))                 /* 比例环节 */
           +(Incremental_KI_RF_LASER*err)                       /* 积分环节 */
           +(Incremental_KD_RF_LASER*(err-2*err_1+err_2));      /* 微分环节 */ 
   err_2=err_1;                                                 /* 保存上上次偏差 */
	 err_1=err;	                                                  /* 保存上一次偏差 */
	 return Pwm;                                                 /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF->2  1 RF

//LB 3  4 RB

/* 目标速度，实际速度 */
int Target_Velocity_LF_LASER=0,Reality_Velocity_LF_LASER=0;
/* 目标位置，实际位置*/
int Target_Position_LF_LASER=0,Reality_Position_LF_LASER=0;

/**
  * @brief  陀螺仪串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void LF_Position_Incremental_PID_LASER(void)
{
  static int LF_Pwm_LASER = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_LF_LASER = Read_Speed_Encoder(3);
  /* 位置式位置控制 */    
  LF_Pwm_LASER = LF_Position_PID_LASER(Reality_Position_LF_LASER,Target_Position_LF_LASER);
  /* 位置环输出限幅 */
  LF_Pwm_LASER = Xianfu(LF_Pwm_LASER,Target_Velocity_LF_LASER);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_LF_LASER - Target_Position_LF_LASER) < 6)
  {
    Motor_Pwm_Set(LF,0);
  }
  else
  {
    /* 增量式速度控制 */
//    LF_Pwm_LASER = LF_Incremental_PID_LASER(Reality_Velocity_LF_LASER,Target_Velocity_LF_LASER);
    LF_Pwm_LASER = LF_Incremental_PID_LASER(Reality_Velocity_LF_LASER,LF_Pwm_LASER);      
    /* 赋值 */
    Motor_Pwm_Set(LF,LF_Pwm_LASER);                                      
  }

}
 
int LF_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                                  /* 计算偏差 */
    err_sum += err;	                                       /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                      /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LF_LASER*err)                       /* 比例环节 */
         +(Position_KI_LF_LASER*err_sum)                   /* 积分环节 */
         +(Position_KD_LF_LASER*(err-err_1));              /* 微分环节 */
    err_1=err;                                             /* 保存上次偏差*/
    return Pwm;                                           /* 输出结果 */
}

int LF_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                          /* 计算偏差 */
	 Pwm += (Incremental_KP_LF_LASER*(err-err_1))                     /* 比例环节 */
           +(Incremental_KI_LF_LASER*err)                           /* 积分环节 */
           +(Incremental_KD_LF_LASER*(err-2*err_1+err_2));          /* 微分环节 */ 
   err_2=err_1;                                                     /* 保存上上次偏差 */
	 err_1=err;	                                                      /* 保存上一次偏差 */
	 return Pwm;                                                     /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB->3  4 RB

/* 目标速度，实际速度 */
int Target_Velocity_LB_LASER=0,Reality_Velocity_LB_LASER=0; 
/* 目标位置，实际位置*/
int Target_Position_LB_LASER=0,Reality_Position_LB_LASER=0;
  
/**
  * @brief  陀螺仪串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */

void LB_Position_Incremental_PID_LASER(void)
{
  static int LB_Pwm_LASER = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_LB_LASER = Read_Speed_Encoder(2);
  /* 位置式位置控制 */    
  LB_Pwm_LASER = LB_Position_PID_LASER(Reality_Position_LB_LASER,Target_Position_LB_LASER);
  /* 位置环输出限幅 */
  LB_Pwm_LASER = Xianfu(LB_Pwm_LASER,Target_Velocity_LB_LASER);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_LB_LASER - Target_Position_LB_LASER) < 6)
  {
    Motor_Pwm_Set(LB,0);      
  }
  else
  {
    /* 增量式速度控制 */
//    LB_Pwm_LASER = LB_Incremental_PID_LASER(Reality_Velocity_LB_LASER,Target_Velocity_LB_LASER);     
    LB_Pwm_LASER = LB_Incremental_PID_LASER(Reality_Velocity_LB_LASER,LB_Pwm_LASER);      
    /* 赋值 */
    Motor_Pwm_Set(LB,LB_Pwm_LASER);                                      
  }
}

int LB_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                                /* 计算偏差 */
    err_sum += err;	                                       /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                      /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LB_LASER*err)                       /* 比例环节 */
         +(Position_KI_LB_LASER*err_sum)                   /* 积分环节 */
         +(Position_KD_LB_LASER*(err-err_1));              /* 微分环节 */
    err_1=err;                                             /* 保存上次偏差*/
    return Pwm;                                           /* 输出结果 */
}

int LB_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* 计算偏差 */
	 Pwm += (Incremental_KP_LB_LASER*(err-err_1))                 /* 比例环节 */
           +(Incremental_KI_LB_LASER*err)                       /* 积分环节 */
           +(Incremental_KD_LB_LASER*(err-2*err_1+err_2));      /* 微分环节 */ 
   err_2=err_1;                                                 /* 保存上上次偏差 */
	 err_1=err;	                                                  /* 保存上一次偏差 */
	 return Pwm;                                                 /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//电机位置示意图
   //机械臂 
//LF 2  1 RF

//LB 3  4->RB

/* 目标速度，实际速度 */
int Target_Velocity_RB_LASER=0,Reality_Velocity_RB_LASER=0;   
/* 目标位置，实际位置*/
int Target_Position_RB_LASER=0,Reality_Position_RB_LASER=0;

/**
  * @brief  陀螺仪串级PID算法实现
  * @param  无
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
void RB_Position_Incremental_PID_LASER(void)
{
  static int RB_Pwm_LASER = 0;
  /* 获取实际脉冲数 */  
  Reality_Velocity_RB_LASER = Read_Speed_Encoder(4);
  /* 位置式位置控制 */   
  RB_Pwm_LASER = RB_Position_PID_LASER(Reality_Position_RB_LASER,Target_Position_RB_LASER);
  /* 位置环输出限幅 */
  RB_Pwm_LASER = Xianfu(RB_Pwm_LASER,Target_Velocity_RB_LASER);
  /* 滤除部分干扰 */
  if(Abs(Reality_Position_RB_LASER - Target_Position_RB_LASER) < 6)
  {
    Motor_Pwm_Set(RB,0);
  }
  else
  {
    /* 增量式速度控制 */ 
//    RB_Pwm_LASER = RB_Incremental_PID_LASER(Reality_Velocity_RB_LASER,Target_Velocity_RB_LASER); 
    RB_Pwm_LASER = RB_Incremental_PID_LASER(Reality_Velocity_RB_LASER,RB_Pwm_LASER);      
    /* 赋值 */
    Motor_Pwm_Set(RB,RB_Pwm_LASER);     
  }                                
}

int RB_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                               /* 计算偏差 */
    err_sum += err;	                                      /* 偏差累积 */
    if(err_sum> 5000) err_sum = 5000;                     /* 积分限幅 */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RB_LASER*err)                      /* 比例环节 */
         +(Position_KI_RB_LASER*err_sum)                  /* 积分环节 */
         +(Position_KD_RB_LASER*(err-err_1));             /* 微分环节 */
    err_1=err;                                             /* 保存上次偏差*/
    return Pwm;                                           /* 输出结果 */
}

int RB_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* 计算偏差 */
	 Pwm += (Incremental_KP_RB_LASER*(err-err_1))                 /* 比例环节 */
           +(Incremental_KI_RB_LASER*err)                       /* 积分环节 */
           +(Incremental_KD_RB_LASER*(err-2*err_1+err_2));      /* 微分环节 */ 
   err_2=err_1;                                                 /* 保存上上次偏差 */
	 err_1=err;	                                                  /* 保存上一次偏差 */
	 return Pwm;                                                 /* 输出结果 */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：int value,int Amplitude
返回  值：无
**************************************************************************/
int Xianfu(int value,int Amplitude)
{	
	float temp;
	if(value>Amplitude) temp = Amplitude;
	else if(value<-Amplitude) temp = -Amplitude;
	else temp = value;
	return temp;
}

/**************************************************************************
函数功能：取绝对值
入口参数：int
返回  值：int
**************************************************************************/
int Abs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


