
#include "bsp_laser.h"
/*==============================================================================
文件作用：用PWM控制激光发射器
文件说明：2025赛季不能使用可见激光，故此文件弃用
  ==============================================================================*/
extern TIM_HandleTypeDef htim3;
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}
