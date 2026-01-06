/************************
 * @file chassis.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief chassis class & methods
 * @version 0.1.0
 * @date 2025-12-01
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#include "robo_behavior.h"

typedef struct State_Wheel_Legged_Chassis
{
    float vyaw;
    float pitch;
    float x;
    float v;
    float theta;
} State_Wheel_Legged_Chassis_t;
