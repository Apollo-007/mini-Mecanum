#include "task.h"

/*
* 函数名:Chassis_OFF
* 功 能 :底盘关闭
* 输 入 :无
* 输 出 :无
*/
void Chassis_OFF(void)
{
  speed_4(0,0,0,0);
  go_4(0,0,0,0);
  Clean_Task_Encoder_val();
  Clear_All_Motor_Now_Encoder_val();
}



/*
* 函数名:First_One_X
* 功 能 :第1轮第1个转弯，X轴调整
* 输 入 :无
* 输 出 :无
*/
void First_One_X(void)
{
  Choice_Laser(Final_LaserX_Data,223);
  speed_4_LaserX(2,2,2,2);
}

/*
* 函数名:First_One_Y
* 功 能 :第1轮第1个转弯，Y轴调整
* 输 入 :无
* 输 出 :无
*/
void First_One_Y(void)
{
  Choice_Laser(Final_LaserY_Data,1950);
  speed_4_LaserX(2,2,2,2);
}
