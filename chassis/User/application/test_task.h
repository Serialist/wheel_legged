/************************************************
 * @file test_task.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-09-10
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 ********************************/

#ifndef __TEST_TASK_H
#define __TEST_TASK_H

/**************************************************************** include ****************************************************************/

#include "struct_typedef.h"
#include "stdint.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "chassismotor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "vmc-dm.h"

/**************************************************************** define ****************************************************************/

/**************************************************************** struct ****************************************************************/

/************************************************
 * @brief 速度计算器状态结构体
 *
 ********************************/
typedef struct
{
    float prev_position;
    uint8_t initialized;
    uint32_t prev_tick;
    float filtered_velocity; // 滤波后的速度值(rad/s)
    float zero_drift_offset; // 零漂补偿值
} VelocityCalculator;

/**************************************************************** extren ****************************************************************/

/**************************************************************** proto ****************************************************************/

#endif
