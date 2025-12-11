/************************
 * @file motor.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-29
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#include "motor.h"

#define MOTOR_TIMEOUT 500u
#define MOTOR_NUMBER 6

struct Motor_Status motor_status;

/************************
 * @brief Motor offline detection
 *
 * @param motor_status
 * @param dt
 ************************/
void Motor_Offline_Detection(struct Motor_Status *motor_status, uint32_t dt)
{
    uint8_t i = 0;

    motor_status->offline_flag = false;

    for (i = 0; i < MOTOR_NUMBER; i++)
    {
        if (motor_status->receive_flag[i] == true)
        {
            motor_status->count[i] = 0;
            motor_status->receive_flag[i] = false;
            motor_status->offline[i] = false;
        }
        else
        {
            if (motor_status->count[i] >= MOTOR_TIMEOUT)
            {
                motor_status->offline[i] = true;
            }
            else
            {
                motor_status->count[i] += dt;
            }
        }

        // 总离线 flag
        motor_status->offline_flag = motor_status->offline_flag || motor_status->offline[i];
    }
}
