#include "wt9011g4k.h"

WT9011G4K WT9011G4K_Data;
struct SAngle stcAngle;

//将结构体中的数据取出
float WT9011G4K_Angle;

//用于存放开机时的目标数值
float Target_WT9011G4K_Angle;

//第一个数值始终为0，不知道为什么
float Temp_Arr[20];


//接收结构体初始化
void WT9011G4K_Init(WT9011G4K *Data)
{
  //清空接收缓冲区
  for(uint16_t i=0;i<RXBUFFER_LEN;i++)
  {
    Data->RxBuffer[i]=0;
  }
  //帧头为0x55
  Data->frame_head=0x55;
  //清除接收标志位
  Data->Rx_flag=0;
  //清除接收长度
  Data->Rx_len=0;
}

//WT9011G4K处理函数
void WT9011G4K_Process(void)
{
  //数据长度不对
  if(WT9011G4K_Data.Rx_len<RXBUFFER_LEN)
    return;
  //如果读出的是帧头0x55,进行处理
  if(WT9011G4K_Data.RxBuffer[0] == WT9011G4K_Data.frame_head)
  {
    //判读数据是否为角度数据帧
    if(WT9011G4K_Data.RxBuffer[1] == 0X53)
    {
      //将高8位与低8位合并
        memcpy(&stcAngle,&WT9011G4K_Data.RxBuffer[2],8);
        for(uint8_t i=0;i<3;i++)
        {
          //角度换算
          WT9011G4K_Data.angle.angle[i] =  (float)stcAngle.Angle[i]/32768*180;
          if(0 < fabs(WT9011G4K_Data.angle.angle[2]) && fabs(WT9011G4K_Data.angle.angle[2]) < 180)
          {
            //delay_ms(20);
            WT9011G4K_Angle = WT9011G4K_Data.angle.angle[2];
          }
        }        
    }
  }
}

//存储开机时的陀螺仪初始位置数值
void Get_Start_WT9011G4K_Angle(void)
{
  for(uint8_t i=0;i<20;i++)
  {
    WT9011G4K_Process();
    delay_ms(10);
    Temp_Arr[i] = WT9011G4K_Angle;
    delay_ms(1);
  }
  Target_WT9011G4K_Angle = Temp_Arr[19];
  Get_Start_WT9011G4K_Angle_Flag = 1;
}


