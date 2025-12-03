/*
==============================================================================
文件作用：此文件为蜂鸣器相关函数的实现文件，主要包含了蜂鸣器的开关控制函数。
文件说明：2025赛季未使用，蜂鸣器可用来做故障提示或初始化成功提示，但2025赛季没用
==============================================================================
*/
#include "tim.h"
#include "bsp_buzzer.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
  __HAL_TIM_PRESCALER(&htim4, psc);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}
void buzzer_off(void)
{
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
