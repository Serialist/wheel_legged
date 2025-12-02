/************************
 * @file debug.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-22
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#ifndef __DEBUG_H
#define __DEBUG_H

#include "user_lib.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

struct Wheel_Leg_Debug
{
    float tpr;
    float tpl;
    float tr;
    float tl;
    uint8_t torque_flag;
    uint8_t no_g_fn_flag;
    uint8_t no_tp_flag;
    uint8_t no_t_flag;
    uint8_t no_yaw_flag;
    uint8_t no_above_det_flag;
    struct
    {
        float tpl, tpr;
        float f0l, f0r;
    } ground_det;
};

extern struct Wheel_Leg_Debug my_debug;

#endif
