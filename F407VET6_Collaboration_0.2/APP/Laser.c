#include "laser.h"

//X轴激光结构体
LASER LaserX;
//Y轴激光结构体
LASER LaserY;

//X轴激光结构体初始化
void LaserX_Init(LASER *Data)
{
  //清空接收缓冲区
  for(uint16_t i=0;i<Laser_RXBUFFER_LEN;i++)
  {
    Data->RxBuffer[i]=0;
  }
  //帧头为0X80
  Data->frame_head=0x80;
  //清除接收标志位
  Data->Rx_flag=0;
  //清除接收长度
  Data->Rx_len=0;
}

//Y轴激光结构体初始化
void LaserY_Init(LASER *Data)
{
  //清空接收缓冲区
  for(uint16_t i=0;i<Laser_RXBUFFER_LEN;i++)
  {
    Data->RxBuffer[i]=0;
  }
  //帧头为0X80
  Data->frame_head=0x80;
  //清除接收标志位
  Data->Rx_flag=0;
  //清除接收长度
  Data->Rx_len=0;
}

uint8_t LaserX_Data[Laser_Data_Buffer];
uint8_t LaserY_Data[Laser_Data_Buffer];
int Final_LaserX_Data;
int Final_LaserY_Data;

//X轴激光处理函数
void LaserX_Process(void)
{
  if(LaserX.RxBuffer[0] == LaserX.frame_head)
  {
    LaserX_Data[0] = LaserX.RxBuffer[3];
    LaserX_Data[1] = LaserX.RxBuffer[4];
    LaserX_Data[2] = LaserX.RxBuffer[5];
    LaserX_Data[3] = LaserX.RxBuffer[7];
    LaserX_Data[4] = LaserX.RxBuffer[8];
    LaserX_Data[5] = LaserX.RxBuffer[9];
    LaserX_Data[6] = '\0';
    //数据处理
    Final_LaserX_Data = atoi((const char*)LaserX_Data);
    X = 1;
  }
}

//Y轴激光处理函数
void LaserY_Process(void)
{
  if(LaserY.RxBuffer[0] == LaserY.frame_head)
  {
    LaserY_Data[0] = LaserY.RxBuffer[3];
    LaserY_Data[1] = LaserY.RxBuffer[4];
    LaserY_Data[2] = LaserY.RxBuffer[5];
    LaserY_Data[3] = LaserY.RxBuffer[7];
    LaserY_Data[4] = LaserY.RxBuffer[8];
    LaserY_Data[5] = LaserY.RxBuffer[9];
    LaserY_Data[6] = '\0';
    //数据处理
    Final_LaserY_Data = atoi((const char*)LaserY_Data);
    Y = 1;
  }
}
