#include "wt9011g4k.h"

WT9011G4K WT9011G4K_Data;
struct SAngle stcAngle;

//���ṹ���е�����ȡ��
float WT9011G4K_Angle;

//���ڴ�ſ���ʱ��Ŀ����ֵ
float Target_WT9011G4K_Angle;

//��һ����ֵʼ��Ϊ0����֪��Ϊʲô
float Temp_Arr[20];


//���սṹ���ʼ��
void WT9011G4K_Init(WT9011G4K *Data)
{
  //��ս��ջ�����
  for(uint16_t i=0;i<RXBUFFER_LEN;i++)
  {
    Data->RxBuffer[i]=0;
  }
  //֡ͷΪ0x55
  Data->frame_head=0x55;
  //������ձ�־λ
  Data->Rx_flag=0;
  //������ճ���
  Data->Rx_len=0;
}

//WT9011G4K������
void WT9011G4K_Process(void)
{
  //���ݳ��Ȳ���
  if(WT9011G4K_Data.Rx_len<RXBUFFER_LEN)
    return;
  //�����������֡ͷ0x55,���д���
  if(WT9011G4K_Data.RxBuffer[0] == WT9011G4K_Data.frame_head)
  {
    //�ж������Ƿ�Ϊ�Ƕ�����֡
    if(WT9011G4K_Data.RxBuffer[1] == 0X53)
    {
      //����8λ���8λ�ϲ�
        memcpy(&stcAngle,&WT9011G4K_Data.RxBuffer[2],8);
        for(uint8_t i=0;i<3;i++)
        {
          //�ǶȻ���
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

//�洢����ʱ�������ǳ�ʼλ����ֵ
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


