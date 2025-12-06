
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "referee_system.h"
#include "remote_control.h"
#include "bsp_can.h"

// m3508转化成底盘速度(m/s)的比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f

#define REMOTE_CHANNLE_TO_CHASSIS_SPEED 0.0009 // 最大速度3.5m/s

#define Pi 3.14159265358979323846f

#define CHASSIS_Y 0
#define CHASSIS_Z 1
#define torque_k 5.050
#define Y_VELOCITY_PID 0   // Y轴速度环
#define Y_LEAN_ANGLE_PID 1 // Y轴直立环
#define SPEED_DEC_1 2      // 减速1
#define SPEED_DEC_2 3      // 减速2

#define Y_VELOCITY 0   // Y轴速度环输出值， 将轮子向前滚动的速度作为正（+），下同
#define Y_LEAN_ANGLE 1 // Y轴倾斜角度输出值
#define Y_CURRENT 2    // Y轴总电流输出
#define rpm_radk 0.105
// Y轴直立环PID参数
#define Y_BALANCE_PID_KP 0.22 // 100
#define Y_BALANCE_PID_KI 0
#define Y_BALANCE_PID_KD 7 // 20000
#define Y_BALANCE_PID_MAX_IOUT 0
#define Y_BALANCE_PID_MAX_OUT 12
// Y轴速度环PID参数
#define Y_VELOCITY_PID_KP 0   //
#define Y_VELOCITY_PID_KI 0.2 //-0.001
#define Y_VELOCITY_PID_KD 0
#define Y_VELOCITY_PID_MAX_IOUT 0.75
#define Y_VELOCITY_PID_MAX_OUT 0.75
// 转向环
#define Turn_PID_KP -0.08 //-300
#define Turn_PID_KI -0.0002
#define Turn_PID_KD -0
#define Turn_PID_MAX_IOUT 0.25
#define Turn_PID_MAX_OUT 4
// l0PID参数
#define L0_BALANCE_PID_KP 230 // 100
#define L0_BALANCE_PID_KI 0
#define L0_BALANCE_PID_KD 62000 // 20000
#define L0_BALANCE_PID_MAX_IOUT 0
#define L0_BALANCE_PID_MAX_OUT 20000
// l02PID参数
#define L0_BALANCE_PID_KP 230 // 100
#define L0_BALANCE_PID_KI 0
#define L0_BALANCE_PID_KD 62000 // 20000
#define L0_BALANCE_PID_MAX_IOUT 0
#define L0_BALANCE_PID_MAX_OUT 20000

#define PITCH 0
#define YAW 1
#define ROLL 2

// 低压保护警告值
#define LOW_VOT 13 // 低于12V可能触发620电调低压保护
#define YAW_PID 4
#define ROLL_PID 5
#define TP_PID 6

#define S_L 1
#define S_R 0

#define UP 1
#define MID 3
#define DOWN 2

#define R_X 0
#define R_Y 1
#define L_X 2
#define L_Y 3
#define L_Z 4
////定义OPEN_SUPER_POWER打开超级电容
// #define OPEN_SUPER_POWER

#endif
