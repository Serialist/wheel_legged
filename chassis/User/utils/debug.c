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

#ifndef BOOL
#define BOOL uint8_t
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef VALUE_LIMIT
#define VALUE_LIMIT(a, min, max) (MAX(min, MIN(a, max)))
#endif

#ifndef LIMIT_ABS
#define LIMIT_ABS(a, max) (VALUE_LIMIT(a, -max, max))
#endif

#ifndef VALUE_MAP
#define VALUE_MAP(a, inmin, intmax, outmin, outmax) (outmin + (a - inmin) * (outmax - outmin) / (intmax - inmin))
#endif

struct Wheel_Leg_Debug my_debug = {
    .tpr = FALSE,
    .tpl = FALSE,
    .tr = FALSE,
    .tl = FALSE,
    .torque_flag = FALSE,
    .no_g_fn_flag = FALSE,
    .no_tp_flag = FALSE,
    .no_t_flag = false,
    .no_yaw_flag = false,
    .no_above_det_flag = FALSE,
    .ground_det = {
        .tpl = FALSE,
        .tpr = FALSE,
        .f0l = FALSE,
        .f0r = FALSE}};

/* ================================================================ log ================================================================ */

/*

enum Log_Level
{
    LOG_LEVEL_ALERT,
    LOG_LEVEL_NOTICE,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_TRACE,
    LOG_LEVEL_PROFILE,
    LOG_LEVEL_LOG,
    LOG_LEVEL_MESSAGE,
    LOG_LEVEL_EVENT,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL,
};

typedef struct Log_Def_t
{
    uint8_t id;

    void (*Output_High)(char *msg);
    void (*Output_Mid)(char *msg);
    void (*Output_Low)(char *msg);

} Log_Def_t;

BOOL Log_Init(Log_Def_t *log,
              void(Output_High)(char *msg), void(Output_Mid)(char *msg), void(Output_Low)(char *msg))
{
    log->id = 0;

    if (Output_High == NULL || Output_Mid == NULL || Output_Low == NULL)
    {
        return FALSE;
    }

    log->Output_High = Output_High;
    log->Output_Mid = Output_Mid;
    log->Output_Low = Output_Low;

    Output_High("Log_Init");
    Output_Mid("Log_Init");
    Output_Low("Log_Init");
}

BOOL Log_Trace(Log_Def_t *log, char *msg)
{
}

void Log_Debug(Log_Def_t *log, char *msg)
{
}

void Log_Info(Log_Def_t *log, char *msg)
{
}

void Log_Warn(Log_Def_t *log, char *msg)
{
}

void Log_Error(Log_Def_t *log, char *msg)
{
}

void Log_Fatal(Log_Def_t *log, char *msg)
{
}

*/

/* ================================================================ utils ================================================================ */

/*

BOOL String_Compare(char *str1, char *str2)
{
    uint8_t i = 0;
    for (i = 0; str1[i] != '\0' && str2[i] != '\0'; i++)
    {
        if (str1[i] != str2[i])
        {
            return FALSE;
            break;
        }
    }
    return TRUE;
}

BOOL Log_ATorder_Parse(Log_Def_t *log, uint8_t *order)
{
    if (!(order[0] == 'A' && order[1] == 'T'))
    {
        return FALSE;
    }

    if (String_Compare(&order[2], "DBG"))
    {
        log->Output_High("Log_ATorder_Parse");
        log->Output_Mid("Log_ATorder_Parse");
        log->Output_Low("Log_ATorder_Parse");
    }
}

*/
