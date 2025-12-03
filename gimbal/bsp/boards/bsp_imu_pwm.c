
#include "bsp_imu_pwm.h"
#include "tim.h"
  /*
  ==============================================================================
文件作用：此文件为使用PWM控制IMU的温度实现温控
文件说明：2025赛季使用了此函数，在gimbal_task中调用，使IMU的温度稳定在某个值
  ==============================================================================
  */
//extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
