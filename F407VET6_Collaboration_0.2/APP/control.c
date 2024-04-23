#include "control.h"

/* ϵͳͳһת��Ϊ���������д����������ֱ��ʸߣ����ƾ��ȸ� */


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//4�����ӵ�PID����������PID
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

//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1->RF

//LB 3  4 RB
/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_RF=0,Reality_Velocity_RF=0;   
/* Ŀ��λ�ã�ʵ��λ�� */
int Target_Position_RF=0,Reality_Position_RF=0;   

/**
  * @brief  ����PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void RF_Position_Incremental_PID(void)
{
  static int RF_Pwm = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_RF = Read_Speed_Encoder(5);
  /* ʵ��λ�������� */
  Reality_Position_RF += Reality_Velocity_RF;
  /* λ��ʽλ�ÿ��� */    
  RF_Pwm = RF_Position_PID(Reality_Position_RF,Target_Position_RF);
  /* λ�û�����޷� */
  RF_Pwm = Xianfu(RF_Pwm,Target_Velocity_RF);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_RF-Target_Position_RF)<5)             
  {
      /* ֹͣ��� */
      Motor_Pwm_Set(RF,0);                                         
  }
  else
  {   
      /* ����ʽ�ٶȿ��� */
//      RF_Pwm = RF_Incremental_PID(Reality_Velocity_RF,Target_Velocity_RF); 
      RF_Pwm = RF_Incremental_PID(Reality_Velocity_RF,RF_Pwm);      
      /* ��ֵ */
      Motor_Pwm_Set(RF,RF_Pwm);                                      
  }
}

int RF_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                        /* ����ƫ�� */
    err_sum += err;	                             /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;            /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RF*err)                   /* �������� */
         +(Position_KI_RF*err_sum)               /* ���ֻ��� */
         +(Position_KD_RF*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                   /* �����ϴ�ƫ��*/
    return Pwm;                                 /* ������ */
}
int RF_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err=target-reality;                                /* ����ƫ�� */
	 Pwm += (Incremental_KP_RF*(err-err_1))             /* �������� */
           +(Incremental_KI_RF*err)                   /* ���ֻ��� */
           +(Incremental_KD_RF*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                       /* �������ϴ�ƫ�� */
	 err_1=err;	                                        /* ������һ��ƫ�� */
	 return Pwm;                                       /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF->2  1 RF

//LB 3  4 RB
/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_LF=0,Reality_Velocity_LF=0;   
/* Ŀ��λ�ã�ʵ��λ�� */
int Target_Position_LF=0,Reality_Position_LF=0;   

/**
  * @brief  ����PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void LF_Position_Incremental_PID(void)
{
  static int LF_Pwm = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_LF = Read_Speed_Encoder(3);
  /* ʵ��λ�������� */
  Reality_Position_LF += Reality_Velocity_LF;
  /* λ��ʽλ�ÿ��� */    
  LF_Pwm = LF_Position_PID(Reality_Position_LF,Target_Position_LF);
  /* λ�û�����޷� */
  LF_Pwm = Xianfu(LF_Pwm,Target_Velocity_LF);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_LF-Target_Position_LF)<5)             
  {
      /* ֹͣ��� */
      Motor_Pwm_Set(LF,0);                                         
  }
  else
  {   
      /* ����ʽ�ٶȿ��� */
//      LF_Pwm = LF_Incremental_PID(Reality_Velocity_LF,Target_Velocity_LF);
      LF_Pwm = LF_Incremental_PID(Reality_Velocity_LF,LF_Pwm);      
      /* ��ֵ */
      Motor_Pwm_Set(LF,LF_Pwm);                                      
  }
}
 
int LF_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                        /* ����ƫ�� */
    err_sum += err;	                             /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;            /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LF*err)                   /* �������� */
         +(Position_KI_LF*err_sum)               /* ���ֻ��� */
         +(Position_KD_LF*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                   /* �����ϴ�ƫ��*/
    return Pwm;                                 /* ������ */
}

int LF_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err=target-reality;                                /* ����ƫ�� */
	 Pwm += (Incremental_KP_LF*(err-err_1))             /* �������� */
           +(Incremental_KI_LF*err)                   /* ���ֻ��� */
           +(Incremental_KD_LF*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                       /* �������ϴ�ƫ�� */
	 err_1=err;	                                        /* ������һ��ƫ�� */
	 return Pwm;                                       /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//���λ��ʾ��ͼ
  //��е�� 
//LF 2  1 RF

//LB->3  4 RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_LB=0,Reality_Velocity_LB=0;   
/* Ŀ��λ�ã�ʵ��λ�� */
int Target_Position_LB=0,Reality_Position_LB=0;

/**
  * @brief  ����PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void LB_Position_Incremental_PID(void)
{
  static int LB_Pwm = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_LB = Read_Speed_Encoder(2);
  /* ʵ��λ�������� */
  Reality_Position_LB += Reality_Velocity_LB;
  /* λ��ʽλ�ÿ��� */    
  LB_Pwm = LB_Position_PID(Reality_Position_LB,Target_Position_LB);
  /* λ�û�����޷� */
  LB_Pwm = Xianfu(LB_Pwm,Target_Velocity_LB);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_LB-Target_Position_LB)<5)             
  {
      /* ֹͣ��� */
      Motor_Pwm_Set(LB,0);                                         
  }
  else
  {   
      /* ����ʽ�ٶȿ��� */
//        LB_Pwm = LB_Incremental_PID(Reality_Velocity_LB,Target_Velocity_LB);     
      LB_Pwm = LB_Incremental_PID(Reality_Velocity_LB,LB_Pwm);      
      /* ��ֵ */
      Motor_Pwm_Set(LB,LB_Pwm);                                      
  }
  
}

int LB_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                      /* ����ƫ�� */
    err_sum += err;	                             /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;            /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LB*err)                   /* �������� */
         +(Position_KI_LB*err_sum)               /* ���ֻ��� */
         +(Position_KD_LB*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                   /* �����ϴ�ƫ��*/
    return Pwm;                                 /* ������ */
}

int LB_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                /* ����ƫ�� */
	 Pwm += (Incremental_KP_LB*(err-err_1))             /* �������� */
           +(Incremental_KI_LB*err)                   /* ���ֻ��� */
           +(Incremental_KD_LB*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                       /* �������ϴ�ƫ�� */
	 err_1=err;	                                        /* ������һ��ƫ�� */
	 return Pwm;                                       /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1 RF

//LB 3  4->RB
/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_RB=0,Reality_Velocity_RB=0;   
/* Ŀ��λ�ã�ʵ��λ�� */
int Target_Position_RB=0,Reality_Position_RB=0;   

/**
  * @brief  ����PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void RB_Position_Incremental_PID(void)
{
  static int RB_Pwm = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_RB = Read_Speed_Encoder(4);
  /* ʵ��λ�������� */
  Reality_Position_RB += Reality_Velocity_RB;
  /* λ��ʽλ�ÿ��� */    
  RB_Pwm = RB_Position_PID(Reality_Position_RB,Target_Position_RB);
  /* λ�û�����޷� */
  RB_Pwm = Xianfu(RB_Pwm,Target_Velocity_RB);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_RB-Target_Position_RB)<5)             
  {
      /* ֹͣ��� */
      Motor_Pwm_Set(RB,0);                                         
  }
  else
  {   
      /* ����ʽ�ٶȿ��� */
//      RB_Pwm = RB_Incremental_PID(Reality_Velocity_RB,Target_Velocity_RB); 
      RB_Pwm = RB_Incremental_PID(Reality_Velocity_RB,RB_Pwm);      
      /* ��ֵ */
      Motor_Pwm_Set(RB,RB_Pwm);                                      
  }
}

int RB_Position_PID(int reality,int target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                        /* ����ƫ�� */
    err_sum += err;	                             /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;            /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RB*err)                   /* �������� */
         +(Position_KI_RB*err_sum)               /* ���ֻ��� */
         +(Position_KD_RB*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                   /* �����ϴ�ƫ��*/
    return Pwm;                                 /* ������ */
}

int RB_Incremental_PID(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                            /* ����ƫ�� */
	 Pwm += (Incremental_KP_RB*(err-err_1))             /* �������� */
           +(Incremental_KI_RB*err)                   /* ���ֻ��� */
           +(Incremental_KD_RB*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                       /* �������ϴ�ƫ�� */
	 err_1=err;	                                        /* ������һ��ƫ�� */
	 return Pwm;                                       /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//4�����ӵ�PID������������PID

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
//������PID����

//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1->RF

//LB 3  4 RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_RF_WT9011G4K=0,Reality_Velocity_RF_WT9011G4K=0;

/**
  * @brief  �����Ǵ���PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void RF_Position_Incremental_PID_WT9011G4K(void)
{
  static int RF_Pwm_WT9011G4K = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_RF_WT9011G4K = Read_Speed_Encoder(5);
  /* λ��ʽλ�ÿ��� */ 
  RF_Pwm_WT9011G4K = RF_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* λ�û�����޷� */
  RF_Pwm_WT9011G4K = Xianfu(RF_Pwm_WT9011G4K,Target_Velocity_RF_WT9011G4K);
  /* �˳����ָ��� */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(RF,0);
  }
  else
  {
    /* ����ʽ�ٶȿ��� */
//    RF_Pwm_WT9011G4K = RF_Incremental_PID_WT9011G4K(Reality_Velocity_RF_WT9011G4K,Target_Velocity_RF_WT9011G4K); 
    RF_Pwm_WT9011G4K = RF_Incremental_PID_WT9011G4K(Reality_Velocity_RF_WT9011G4K,RF_Pwm_WT9011G4K);      
    /* ��ֵ */
    Motor_Pwm_Set(RF,RF_Pwm_WT9011G4K);                                      
  }
}

int RF_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    
    err = target - reality;                                /* ����ƫ�� */
    err_sum += err;	                                       /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                      /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RF_WT9011G4K*err)                   /* �������� */
         +(Position_KI_RF_WT9011G4K*err_sum)               /* ���ֻ��� */
         +(Position_KD_RF_WT9011G4K*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                             /* �����ϴ�ƫ��*/
    return Pwm;                                           /* ������ */
}

int RF_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err= target - reality;                                       /* ����ƫ�� */
	 Pwm += (Incremental_KP_RF_WT9011G4K*(err-err_1))             /* �������� */
           +(Incremental_KI_RF_WT9011G4K*err)                   /* ���ֻ��� */
           +(Incremental_KD_RF_WT9011G4K*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                                 /* �������ϴ�ƫ�� */
	 err_1=err;	                                                  /* ������һ��ƫ�� */
	 return Pwm;                                                 /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF->2  1 RF

//LB 3  4 RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_LF_WT9011G4K=0,Reality_Velocity_LF_WT9011G4K=0;   

/**
  * @brief  �����Ǵ���PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void LF_Position_Incremental_PID_WT9011G4K(void)
{
  static int LF_Pwm_WT9011G4K = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_LF_WT9011G4K = Read_Speed_Encoder(3);
  /* λ��ʽλ�ÿ��� */    
  LF_Pwm_WT9011G4K = LF_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* λ�û�����޷� */
  LF_Pwm_WT9011G4K = Xianfu(LF_Pwm_WT9011G4K,Target_Velocity_LF_WT9011G4K);
  /* �˳����ָ��� */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(LF,0);
  }
  else
  {
    /* ����ʽ�ٶȿ��� */
//    LF_Pwm_WT9011G4K = LF_Incremental_PID_WT9011G4K(Reality_Velocity_LF_WT9011G4K,Target_Velocity_LF_WT9011G4K);
    LF_Pwm_WT9011G4K = LF_Incremental_PID_WT9011G4K(Reality_Velocity_LF_WT9011G4K,LF_Pwm_WT9011G4K);      
    /* ��ֵ */
    Motor_Pwm_Set(LF,LF_Pwm_WT9011G4K);                                      
  }

}
 
int LF_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                                  /* ����ƫ�� */
    err_sum += err;	                                       /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                      /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LF_WT9011G4K*err)                   /* �������� */
         +(Position_KI_LF_WT9011G4K*err_sum)               /* ���ֻ��� */
         +(Position_KD_LF_WT9011G4K*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                             /* �����ϴ�ƫ��*/
    return Pwm;                                           /* ������ */
}

int LF_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                          /* ����ƫ�� */
	 Pwm += (Incremental_KP_LF_WT9011G4K*(err-err_1))             /* �������� */
           +(Incremental_KI_LF_WT9011G4K*err)                   /* ���ֻ��� */
           +(Incremental_KD_LF_WT9011G4K*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                                 /* �������ϴ�ƫ�� */
	 err_1=err;	                                                  /* ������һ��ƫ�� */
	 return Pwm;                                                 /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1 RF

//LB->3  4 RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_LB_WT9011G4K=0,Reality_Velocity_LB_WT9011G4K=0;   

/**
  * @brief  �����Ǵ���PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */

void LB_Position_Incremental_PID_WT9011G4K(void)
{
  static int LB_Pwm_WT9011G4K = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_LB_WT9011G4K = Read_Speed_Encoder(2);
  /* λ��ʽλ�ÿ��� */    
  LB_Pwm_WT9011G4K = LB_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* λ�û�����޷� */
  LB_Pwm_WT9011G4K = Xianfu(LB_Pwm_WT9011G4K,Target_Velocity_LB_WT9011G4K);
  /* �˳����ָ��� */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(LB,0);      
  }
  else
  {
    /* ����ʽ�ٶȿ��� */
//    LB_Pwm_WT9011G4K = LB_Incremental_PID_WT9011G4K(Reality_Velocity_LB_WT9011G4K,Target_Velocity_LB_WT9011G4K);     
    LB_Pwm_WT9011G4K = LB_Incremental_PID_WT9011G4K(Reality_Velocity_LB_WT9011G4K,LB_Pwm_WT9011G4K);      
    /* ��ֵ */
    Motor_Pwm_Set(LB,LB_Pwm_WT9011G4K);                                      
  }
}

int LB_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                                /* ����ƫ�� */
    err_sum += err;	                                       /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                      /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LB_WT9011G4K*err)                   /* �������� */
         +(Position_KI_LB_WT9011G4K*err_sum)               /* ���ֻ��� */
         +(Position_KD_LB_WT9011G4K*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                             /* �����ϴ�ƫ��*/
    return Pwm;                                           /* ������ */
}

int LB_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* ����ƫ�� */
	 Pwm += (Incremental_KP_LB_WT9011G4K*(err-err_1))             /* �������� */
           +(Incremental_KI_LB_WT9011G4K*err)                   /* ���ֻ��� */
           +(Incremental_KD_LB_WT9011G4K*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                                 /* �������ϴ�ƫ�� */
	 err_1=err;	                                                  /* ������һ��ƫ�� */
	 return Pwm;                                                 /* ������ */
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1 RF

//LB 3  4->RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_RB_WT9011G4K=0,Reality_Velocity_RB_WT9011G4K=0;   
  
/**
  * @brief  �����Ǵ���PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void RB_Position_Incremental_PID_WT9011G4K(void)
{
  static int RB_Pwm_WT9011G4K = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_RB_WT9011G4K = Read_Speed_Encoder(4);
  /* λ��ʽλ�ÿ��� */   
  RB_Pwm_WT9011G4K = RB_Position_PID_WT9011G4K(WT9011G4K_Angle,Target_WT9011G4K_Angle);
  /* λ�û�����޷� */
  RB_Pwm_WT9011G4K = Xianfu(RB_Pwm_WT9011G4K,Target_Velocity_RB_WT9011G4K);
  /* �˳����ָ��� */
  if(fabs(WT9011G4K_Angle - Target_WT9011G4K_Angle) < 0.15)
  {
    Motor_Pwm_Set(RB,0);
  }
  else
  {
    /* ����ʽ�ٶȿ��� */ 
//    RB_Pwm_WT9011G4K = RB_Incremental_PID_WT9011G4K(Reality_Velocity_RB_WT9011G4K,Target_Velocity_RB_WT9011G4K); 
    RB_Pwm_WT9011G4K = RB_Incremental_PID_WT9011G4K(Reality_Velocity_RB_WT9011G4K,RB_Pwm_WT9011G4K);      
    /* ��ֵ */
    Motor_Pwm_Set(RB,RB_Pwm_WT9011G4K);     
  }                                
}

int RB_Position_PID_WT9011G4K(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                               /* ����ƫ�� */
    err_sum += err;	                                      /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                     /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RB_WT9011G4K*err)                   /* �������� */
         +(Position_KI_RB_WT9011G4K*err_sum)               /* ���ֻ��� */
         +(Position_KD_RB_WT9011G4K*(err-err_1));          /* ΢�ֻ��� */
    err_1=err;                                             /* �����ϴ�ƫ��*/
    return Pwm;                                           /* ������ */
}

int RB_Incremental_PID_WT9011G4K(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* ����ƫ�� */
	 Pwm += (Incremental_KP_RB_WT9011G4K*(err-err_1))             /* �������� */
           +(Incremental_KI_RB_WT9011G4K*err)                   /* ���ֻ��� */
           +(Incremental_KD_RB_WT9011G4K*(err-2*err_1+err_2));  /* ΢�ֻ��� */ 
   err_2=err_1;                                                 /* �������ϴ�ƫ�� */
	 err_1=err;	                                                  /* ������һ��ƫ�� */
	 return Pwm;                                                 /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//4�����ӵ�PID����������PID

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
//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1->RF

//LB 3  4 RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_RF_LASER=0,Reality_Velocity_RF_LASER=0;
/* Ŀ��λ�ã�ʵ��λ��*/
int Target_Position_RF_LASER=0,Reality_Position_RF_LASER=0;

/**
  * @brief  ���⴮��PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void RF_Position_Incremental_PID_LASER(void)
{
  static int RF_Pwm_LASER = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_RF_LASER = Read_Speed_Encoder(5);
  /* λ��ʽλ�ÿ��� */ 
  RF_Pwm_LASER = RF_Position_PID_LASER(Reality_Position_RF_LASER,Target_Position_RF_LASER);
  /* λ�û�����޷� */
  RF_Pwm_LASER = Xianfu(RF_Pwm_LASER,Target_Velocity_RF_LASER);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_RF_LASER - Target_Position_RF_LASER) < 6)
  {
    Motor_Pwm_Set(RF,0);
  }
  else
  {
    /* ����ʽ�ٶȿ��� */
//    RF_Pwm_LASER = RF_Incremental_PID_LASER(Reality_Velocity_RF_LASER,Target_Velocity_RF_LASER); 
    RF_Pwm_LASER = RF_Incremental_PID_LASER(Reality_Velocity_RF_LASER,RF_Pwm_LASER);      
    /* ��ֵ */
    Motor_Pwm_Set(RF,RF_Pwm_LASER);                                      
  }
}

int RF_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    
    err = target - reality;                                 /* ����ƫ�� */
    err_sum += err;	                                        /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                       /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RF_LASER*err)                        /* �������� */
         +(Position_KI_RF_LASER*err_sum)                    /* ���ֻ��� */
         +(Position_KD_RF_LASER*(err-err_1));               /* ΢�ֻ��� */
    err_1=err;                                              /* �����ϴ�ƫ��*/
    return Pwm;                                            /* ������ */
}

int RF_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err= target - reality;                                       /* ����ƫ�� */
	 Pwm += (Incremental_KP_RF_LASER*(err-err_1))                 /* �������� */
           +(Incremental_KI_RF_LASER*err)                       /* ���ֻ��� */
           +(Incremental_KD_RF_LASER*(err-2*err_1+err_2));      /* ΢�ֻ��� */ 
   err_2=err_1;                                                 /* �������ϴ�ƫ�� */
	 err_1=err;	                                                  /* ������һ��ƫ�� */
	 return Pwm;                                                 /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF->2  1 RF

//LB 3  4 RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_LF_LASER=0,Reality_Velocity_LF_LASER=0;
/* Ŀ��λ�ã�ʵ��λ��*/
int Target_Position_LF_LASER=0,Reality_Position_LF_LASER=0;

/**
  * @brief  �����Ǵ���PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void LF_Position_Incremental_PID_LASER(void)
{
  static int LF_Pwm_LASER = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_LF_LASER = Read_Speed_Encoder(3);
  /* λ��ʽλ�ÿ��� */    
  LF_Pwm_LASER = LF_Position_PID_LASER(Reality_Position_LF_LASER,Target_Position_LF_LASER);
  /* λ�û�����޷� */
  LF_Pwm_LASER = Xianfu(LF_Pwm_LASER,Target_Velocity_LF_LASER);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_LF_LASER - Target_Position_LF_LASER) < 6)
  {
    Motor_Pwm_Set(LF,0);
  }
  else
  {
    /* ����ʽ�ٶȿ��� */
//    LF_Pwm_LASER = LF_Incremental_PID_LASER(Reality_Velocity_LF_LASER,Target_Velocity_LF_LASER);
    LF_Pwm_LASER = LF_Incremental_PID_LASER(Reality_Velocity_LF_LASER,LF_Pwm_LASER);      
    /* ��ֵ */
    Motor_Pwm_Set(LF,LF_Pwm_LASER);                                      
  }

}
 
int LF_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target-reality;                                  /* ����ƫ�� */
    err_sum += err;	                                       /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                      /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LF_LASER*err)                       /* �������� */
         +(Position_KI_LF_LASER*err_sum)                   /* ���ֻ��� */
         +(Position_KD_LF_LASER*(err-err_1));              /* ΢�ֻ��� */
    err_1=err;                                             /* �����ϴ�ƫ��*/
    return Pwm;                                           /* ������ */
}

int LF_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                          /* ����ƫ�� */
	 Pwm += (Incremental_KP_LF_LASER*(err-err_1))                     /* �������� */
           +(Incremental_KI_LF_LASER*err)                           /* ���ֻ��� */
           +(Incremental_KD_LF_LASER*(err-2*err_1+err_2));          /* ΢�ֻ��� */ 
   err_2=err_1;                                                     /* �������ϴ�ƫ�� */
	 err_1=err;	                                                      /* ������һ��ƫ�� */
	 return Pwm;                                                     /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1 RF

//LB->3  4 RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_LB_LASER=0,Reality_Velocity_LB_LASER=0; 
/* Ŀ��λ�ã�ʵ��λ��*/
int Target_Position_LB_LASER=0,Reality_Position_LB_LASER=0;
  
/**
  * @brief  �����Ǵ���PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */

void LB_Position_Incremental_PID_LASER(void)
{
  static int LB_Pwm_LASER = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_LB_LASER = Read_Speed_Encoder(2);
  /* λ��ʽλ�ÿ��� */    
  LB_Pwm_LASER = LB_Position_PID_LASER(Reality_Position_LB_LASER,Target_Position_LB_LASER);
  /* λ�û�����޷� */
  LB_Pwm_LASER = Xianfu(LB_Pwm_LASER,Target_Velocity_LB_LASER);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_LB_LASER - Target_Position_LB_LASER) < 6)
  {
    Motor_Pwm_Set(LB,0);      
  }
  else
  {
    /* ����ʽ�ٶȿ��� */
//    LB_Pwm_LASER = LB_Incremental_PID_LASER(Reality_Velocity_LB_LASER,Target_Velocity_LB_LASER);     
    LB_Pwm_LASER = LB_Incremental_PID_LASER(Reality_Velocity_LB_LASER,LB_Pwm_LASER);      
    /* ��ֵ */
    Motor_Pwm_Set(LB,LB_Pwm_LASER);                                      
  }
}

int LB_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                                /* ����ƫ�� */
    err_sum += err;	                                       /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                      /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_LB_LASER*err)                       /* �������� */
         +(Position_KI_LB_LASER*err_sum)                   /* ���ֻ��� */
         +(Position_KD_LB_LASER*(err-err_1));              /* ΢�ֻ��� */
    err_1=err;                                             /* �����ϴ�ƫ��*/
    return Pwm;                                           /* ������ */
}

int LB_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* ����ƫ�� */
	 Pwm += (Incremental_KP_LB_LASER*(err-err_1))                 /* �������� */
           +(Incremental_KI_LB_LASER*err)                       /* ���ֻ��� */
           +(Incremental_KD_LB_LASER*(err-2*err_1+err_2));      /* ΢�ֻ��� */ 
   err_2=err_1;                                                 /* �������ϴ�ƫ�� */
	 err_1=err;	                                                  /* ������һ��ƫ�� */
	 return Pwm;                                                 /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���λ��ʾ��ͼ
   //��е�� 
//LF 2  1 RF

//LB 3  4->RB

/* Ŀ���ٶȣ�ʵ���ٶ� */
int Target_Velocity_RB_LASER=0,Reality_Velocity_RB_LASER=0;   
/* Ŀ��λ�ã�ʵ��λ��*/
int Target_Position_RB_LASER=0,Reality_Position_RB_LASER=0;

/**
  * @brief  �����Ǵ���PID�㷨ʵ��
  * @param  ��
	*	@note 	��
  * @retval ͨ��PID���������
  */
void RB_Position_Incremental_PID_LASER(void)
{
  static int RB_Pwm_LASER = 0;
  /* ��ȡʵ�������� */  
  Reality_Velocity_RB_LASER = Read_Speed_Encoder(4);
  /* λ��ʽλ�ÿ��� */   
  RB_Pwm_LASER = RB_Position_PID_LASER(Reality_Position_RB_LASER,Target_Position_RB_LASER);
  /* λ�û�����޷� */
  RB_Pwm_LASER = Xianfu(RB_Pwm_LASER,Target_Velocity_RB_LASER);
  /* �˳����ָ��� */
  if(Abs(Reality_Position_RB_LASER - Target_Position_RB_LASER) < 6)
  {
    Motor_Pwm_Set(RB,0);
  }
  else
  {
    /* ����ʽ�ٶȿ��� */ 
//    RB_Pwm_LASER = RB_Incremental_PID_LASER(Reality_Velocity_RB_LASER,Target_Velocity_RB_LASER); 
    RB_Pwm_LASER = RB_Incremental_PID_LASER(Reality_Velocity_RB_LASER,RB_Pwm_LASER);      
    /* ��ֵ */
    Motor_Pwm_Set(RB,RB_Pwm_LASER);     
  }                                
}

int RB_Position_PID_LASER(float reality,float target)
{ 	
    static float err,err_1,err_sum,Pwm=0;
    err = target - reality;                               /* ����ƫ�� */
    err_sum += err;	                                      /* ƫ���ۻ� */
    if(err_sum> 5000) err_sum = 5000;                     /* �����޷� */
    if(err_sum<-5000) err_sum =-5000;            
    Pwm = (Position_KP_RB_LASER*err)                      /* �������� */
         +(Position_KI_RB_LASER*err_sum)                  /* ���ֻ��� */
         +(Position_KD_RB_LASER*(err-err_1));             /* ΢�ֻ��� */
    err_1=err;                                             /* �����ϴ�ƫ��*/
    return Pwm;                                           /* ������ */
}

int RB_Incremental_PID_LASER(int reality,int target)
{ 	
	 static float err,err_1,err_2,Pwm;
    
	 err = target - reality;                                      /* ����ƫ�� */
	 Pwm += (Incremental_KP_RB_LASER*(err-err_1))                 /* �������� */
           +(Incremental_KI_RB_LASER*err)                       /* ���ֻ��� */
           +(Incremental_KD_RB_LASER*(err-2*err_1+err_2));      /* ΢�ֻ��� */ 
   err_2=err_1;                                                 /* �������ϴ�ƫ�� */
	 err_1=err;	                                                  /* ������һ��ƫ�� */
	 return Pwm;                                                 /* ������ */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�����int value,int Amplitude
����  ֵ����
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
�������ܣ�ȡ����ֵ
��ڲ�����int
����  ֵ��int
**************************************************************************/
int Abs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


