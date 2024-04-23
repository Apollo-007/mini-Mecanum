#include "delay.h"

//TIM14��ʱΪ1us

void delay_ms(uint16_t nms) 
{
    uint32_t cnt;
    cnt = nms * 1000;
    // ������ʱ��14
    HAL_TIM_Base_Start(&htim14); 
    while (cnt--)
    {
      while (__HAL_TIM_GET_FLAG(&htim14, TIM_FLAG_UPDATE) == RESET);
      __HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
    }
    // ֹͣ��ʱ��14
    HAL_TIM_Base_Stop(&htim14); 
}

void delay_us(uint32_t nus)
{
    uint32_t cnt;
    cnt = nus;
    // ������ʱ��14
    HAL_TIM_Base_Start(&htim4);
    while (cnt--)
    {
      while (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) == RESET);
      __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    }
    // ֹͣ��ʱ��14
    HAL_TIM_Base_Stop(&htim4); 
}
