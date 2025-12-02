/************************
 * @file motor.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-29
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#ifndef __MOTOR_H
#define __MOTOR_H

#include "user_lib.h"

struct Motor_Status
{
    uint8_t receive_flag[6];
    uint8_t offline[6];
    uint8_t offline_flag;
    uint16_t count[6];
};

#define MOTOR_IS_OFFLINE(motor_status) ((motor_status)->offline_flag == true)

extern struct Motor_Status motor_status;

void Motor_Offline_Detection(struct Motor_Status *motor_status, uint32_t dt);

#endif
