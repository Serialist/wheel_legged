/************************
 * @file chassis.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-12-01
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "user_lib.h"

struct Robo_Attitude
{
    float x, y, z;
    float vx, vy, vz;
    float ax, ay, az;

    float roll, pitch, yaw;
    float total_roll, total_pitch, total_yaw;
    float vroll, vpitch, vyaw;
    float aroll, apitch, ayaw;
};

enum Robo_State
{
    ROBO_STATE_NONE = 0,
    ROBO_STATE_INIT,      // 初始化
    ROBO_STATE_RUN,       // 运行
    ROBO_STATE_STOP,      // 停止
    ROBO_STATE_EMERGENCY, // 急停
    ROBO_STATE_STAND,
};

enum Robo_Behavior
{
    ROBO_BX_NONE = 0,
    ROBO_BX_OFF,
    ROBO_BX_STOP,
    ROBO_BX_NORMAL,
    ROBO_BX_JUMP,
    ROBO_BX_STAND,
};

struct Wheel_Leg_Flag
{
    uint8_t above_left;
    uint8_t above_right;
    uint8_t above;
    uint8_t jump_v_reach;
};

struct Robo_Status
{
    enum Robo_State status;

    enum Robo_Behavior last_behavior;
    enum Robo_Behavior behavior;

    struct Wheel_Leg_Flag flag;
};

#endif
