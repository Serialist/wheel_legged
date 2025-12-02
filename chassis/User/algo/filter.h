/************************
 * @file filter.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-29
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#ifndef __FILTER_H
#define __FILTER_H

#include "user_lib.h"

/* ================================================ filter average ================================================ */

#define FILTER_AVERAGE_FLOAT_WINDOW_MAX_SIZE 32

#define FILTER_AVERAGE_UPDATA_WEIGHTED(filter, calibrate, weight) Filter_Average_Update((filter), ((calibrate) * (weight)))

struct Filter_Average
{
    float window[FILTER_AVERAGE_FLOAT_WINDOW_MAX_SIZE];
    float average;
    uint8_t width;
    uint8_t index;
};

bool Filter_Average_Init(struct Filter_Average *filter, uint8_t width);

float Filter_Average_Update(struct Filter_Average *filter, float calibrate);

#endif
