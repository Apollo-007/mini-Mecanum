#include "task.h"

/*
* ������:Chassis_OFF
* �� �� :���̹ر�
* �� �� :��
* �� �� :��
*/
void Chassis_OFF(void)
{
  speed_4(0,0,0,0);
  go_4(0,0,0,0);
  Clean_Task_Encoder_val();
  Clear_All_Motor_Now_Encoder_val();
}



/*
* ������:First_One_X
* �� �� :��1�ֵ�1��ת�䣬X�����
* �� �� :��
* �� �� :��
*/
void First_One_X(void)
{
  Choice_Laser(Final_LaserX_Data,223);
  speed_4_LaserX(2,2,2,2);
}

/*
* ������:First_One_Y
* �� �� :��1�ֵ�1��ת�䣬Y�����
* �� �� :��
* �� �� :��
*/
void First_One_Y(void)
{
  Choice_Laser(Final_LaserY_Data,1950);
  speed_4_LaserX(2,2,2,2);
}
