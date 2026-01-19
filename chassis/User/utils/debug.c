/************************
 * @file debug.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-22
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "debug.h"

struct Wheel_Leg_Debug my_debug = {
    .tpr = false,
    .tpl = false,
    .tr = false,
    .tl = false,
    .torque_flag = false,
    .no_g_fn_flag = false,
    .no_tp_flag = false,
    .no_t_flag = false,
    .no_yaw_flag = false,
    .no_above_det_flag = false,
    .ground_det = {
        .tpl = false,
        .tpr = false,
        .f0l = false,
        .f0r = false}};

/* ================================================================ log ================================================================ */

/* ================================================================ utils ================================================================ */

/*
BOOL Log_ATorder_Parse(Log_Def_t *log, uint8_t *order)
{
    if (!(order[0] == 'A' && order[1] == 'T'))
    {
        return false;
    }

    if (String_Compare(&order[2], "DBG"))
    {
        log->Output_High("Log_ATorder_Parse");
        log->Output_Mid("Log_ATorder_Parse");
        log->Output_Low("Log_ATorder_Parse");
    }
}
*/
