/***********************************************
 * @file bsp_buzzer.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-25
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 *************************************************/

#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

void Buzzer_Init(void)
{
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void Buzzer_On(uint32_t freq, uint32_t delay)
{
    __HAL_TIM_PRESCALER(&htim4, 10000 / freq);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 20000); // 20%占空比
    HAL_Delay(delay);
    Buzzer_Off();
}

void Buzzer_Off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void Buzzer_Preset(void)
{
    Buzzer_On(200, 200);
    Buzzer_On(300, 200);
    Buzzer_On(400, 200);
}
