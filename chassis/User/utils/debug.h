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

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef VALUE_LIMIT
#define VALUE_LIMIT(a, min, max) (MAX(min, MIN(a, max)))
#endif

#ifndef VALUE_LIMIT_ABS
#define VALUE_LIMIT_ABS(a, max) (VALUE_LIMIT(a, -max, max))
#endif

#ifndef VALUE_MAP
#define VALUE_MAP(a, inmin, intmax, outmin, outmax) (outmin + (a - inmin) * (outmax - outmin) / (intmax - inmin))
#endif

#ifndef RAMP
#define RAMP(prev_x, x, k_min, k_max, dt) (prev_x += VALUE_LIMIT(x - prev_x, k_min * dt, k_max * dt))
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
