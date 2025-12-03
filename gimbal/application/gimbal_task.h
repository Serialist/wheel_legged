/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
/********* 锟斤拷台PID锟斤拷锟斤拷锟叫癸拷 锟斤拷始 *******************/

#include "struct_typedef.h"
#include "vision.h"
#include "remote_control.h"
#include "bsp_can.h"
// 2025.10.1,大底盘的叫ROBOT_Sentry，小底盘的有雷达的步改哨叫ROBOT_Infantry
// 其实电机ID可以自己改，但是现在机械装了不好改了，主要还是PID参数有差别
//**********************步兵模式所有pid设置*****************************//

/**
 * @brief 定义机器人模式
 * @date 2024-10-06 19:11
 * @version 1.1.1
 * @author Serialist
 * @note
 * ROBOT_ID 定义的是机器人的硬件配置，比如电机ID，零点位置等
 * ROBOT_MODE 定义的是机器人的软件功能，比如通信协议，交互，遥控器等
 * 同样使用 ROBOT_Infantry 或 ROBOT_Sentry
*/
#define ROBOT_MODE ROBOT_Sentry

#if (ROBOT_ID == ROBOT_Infantry)
// pid位置环
#define PITCH_SHELL_PID_KP 80
#define PITCH_SHELL_PID_KI 0.9
#define PITCH_SHELL_PID_KD 15

#define PITCH_SHELL_PID_MAX_IOUT 50
#define PITCH_SHELL_PID_MAX_OUT 16384

// YAW轴位置环
#define YAW_SHELL_PID_KP 15
#define YAW_SHELL_PID_KI 0.0
#define YAW_SHELL_PID_KD 2
#define YAW_SHELL_PID_MAX_IOUT 20
#define YAW_SHELL_PID_MAX_OUT 5000

// YAW轴自瞄位置环
#define YAW_VISION_SHELL_PID_KP -10 // 13(1.5)
#define YAW_VISION_SHELL_PID_KI -0.1
#define YAW_VISION_SHELL_PID_KD -2000 // 0.75(1.5)
#define YAW_VISION_SHELL_PID_KF 1
#define YAW_VISION_SHELL_PID_MAX_IOUT 100
#define YAW_VISION_SHELL_PID_MAX_OUT 200

// pitch轴自瞄位置环
#define PITCH_VISION_SHELL_PID_KP 15
#define PITCH_VISION_SHELL_PID_KI 0.15
#define PITCH_VISION_SHELL_PID_KD 2000
#define PITCH_VISION_SHELL_PID_MAX_IOUT 100
#define PITCH_VISION_SHELL_PID_MAX_OUT 15000

// pitch轴速度环
#define PITCH_CORE_PID_KP -200
#define PITCH_CORE_PID_KI 0
#define PITCH_CORE_PID_KD -50
#define PITCH_CORE_PID_MAX_IOUT 30
#define PITCH_CORE_PID_MAX_OUT 15000
// yaw轴速度环
#define YAW_CORE_PID_KP 250 // 800
#define YAW_CORE_PID_KI 0
#define YAW_CORE_PID_KD 15 // 150
#define YAW_CORE_PID_MAX_IOUT 10001
#define YAW_CORE_PID_MAX_OUT 25000

// pitch轴自瞄位置环
#define VISION_YAW_CORE_PID_KP 800 // 170//270
#define VISION_YAW_CORE_PID_KI 0
#define VISION_YAW_CORE_PID_KD 250 // 70//100
#define VISION_YAW_CORE_PID_KF 1
#define VISION_YAW_CORE_PID_MAX_IOUT 0
#define VISION_YAW_CORE_PID_MAX_OUT 25000

// pitch轴自瞄速度环
#define PITCH_VISION_CORE_PID_KP -870
#define PITCH_VISION_CORE_PID_KI 0
#define PITCH_VISION_CORE_PID_KD -150
#define PITCH_VISION_CORE_PID_MAX_IOUT 0
#define PITCH_VISION_CORE_PID_MAX_OUT 15000

// 云底联动pid
#define FOLLOW_PID_KP 75
#define FOLLOW_PID_KI 0.1
#define FOLLOW_PID_KD 1500
#define FOLLOW_PID_MAX_IOUT 20
#define FOLLOW_PID_MAX_OUT 8000
//**********************步兵模式所有pid设置*****************************//
#elif (ROBOT_ID == ROBOT_Sentry)
//**********************哨兵模式所有pid设置*****************************//
// pid位置环
#define PITCH_SHELL_PID_KP 30
#define PITCH_SHELL_PID_KI 1
#define PITCH_SHELL_PID_KD 5

#define PITCH_SHELL_PID_MAX_IOUT 50
#define PITCH_SHELL_PID_MAX_OUT 16384

// YAW轴位置环
#define YAW_SHELL_PID_KP 15
#define YAW_SHELL_PID_KI 0.0
#define YAW_SHELL_PID_KD 2
#define YAW_SHELL_PID_MAX_IOUT 20
#define YAW_SHELL_PID_MAX_OUT 5000

// YAW轴自瞄位置环
#define YAW_VISION_SHELL_PID_KP -10 // 13(1.5)
#define YAW_VISION_SHELL_PID_KI -0.1
#define YAW_VISION_SHELL_PID_KD -2000 // 0.75(1.5)
#define YAW_VISION_SHELL_PID_KF 1
#define YAW_VISION_SHELL_PID_MAX_IOUT 100
#define YAW_VISION_SHELL_PID_MAX_OUT 200

// pitch轴自瞄位置环
#define PITCH_VISION_SHELL_PID_KP 15
#define PITCH_VISION_SHELL_PID_KI 0.15
#define PITCH_VISION_SHELL_PID_KD 2000
#define PITCH_VISION_SHELL_PID_MAX_IOUT 100
#define PITCH_VISION_SHELL_PID_MAX_OUT 15000

// pitch轴速度环
#define PITCH_CORE_PID_KP 120
#define PITCH_CORE_PID_KI 0
#define PITCH_CORE_PID_KD 30
#define PITCH_CORE_PID_MAX_IOUT 30
#define PITCH_CORE_PID_MAX_OUT 15000
// yaw轴速度环
#define YAW_CORE_PID_KP 250 // 800
#define YAW_CORE_PID_KI 0
#define YAW_CORE_PID_KD 15 // 150
#define YAW_CORE_PID_MAX_IOUT 10001
#define YAW_CORE_PID_MAX_OUT 25000

// pitch轴自瞄位置环
#define VISION_YAW_CORE_PID_KP 800 // 170//270
#define VISION_YAW_CORE_PID_KI 0
#define VISION_YAW_CORE_PID_KD 250 // 70//100
#define VISION_YAW_CORE_PID_KF 1
#define VISION_YAW_CORE_PID_MAX_IOUT 0
#define VISION_YAW_CORE_PID_MAX_OUT 25000

// pitch轴自瞄速度环
#define PITCH_VISION_CORE_PID_KP -870
#define PITCH_VISION_CORE_PID_KI 0
#define PITCH_VISION_CORE_PID_KD -150
#define PITCH_VISION_CORE_PID_MAX_IOUT 0
#define PITCH_VISION_CORE_PID_MAX_OUT 15000

// 云底联动pid
#define FOLLOW_PID_KP 100
#define FOLLOW_PID_KI 0.5
#define FOLLOW_PID_KD 1500
#define FOLLOW_PID_MAX_IOUT 20
#define FOLLOW_PID_MAX_OUT 8000
//**********************哨兵模式所有pid设置*****************************//

#endif

#define average 1
#define kalman 2
#define normal 0

#define CHANNEL_SCALE_FACTOR 3000.0f

#define YAW_RELATIVE_MAX_ANGLE 179
#define YAW_RELATIVE_MIN_ANGLE -179
#define PITCH_RELATIVE_MAX_ANGLE 30
#define PITCH_RELATIVE_MIN_ANGLE -15

#define SPEED_X 0
#define SPEED_Y 1
#define SPEED_W 2

#define max_pitch_ecd 3380
#define min_pitch_ecd 2500

typedef enum
{
	HEAD,
	TAIL,
	RIGHT,
	LEFT
} Head_fdb;

typedef enum
{

	OBLIQUE
} oblique_t;
typedef enum
{
	PITCHREL,
	YAWREL,
	ROLLREL
} Axis_motorkm;

// 按键类型重定义
#define key_E 0
#define key_V 1
#define key_C 2
#define key_F 11
#define key_SHIFT 3
#define key_CTRL 4
#define key_B 5
#define fr_1 6
#define fr_2 7
#define Pause 8
#define trigger 9
#define key_X 10
#define key_Q 11
#define key_G 12
// 按键类型重定义

typedef enum
{
	IMUPitchAngle,
	IMUPitchGyro,
	IMUPitchAccel,
	IMUYawAngle,
	IMUYawGyro,
	IMUYawAccel
} Axis_imukm;

typedef enum
{
	INIT_MODE,
	ZERO_FORCE_MODE,
	NORMAL_MODE,
	VISION_FOLLOW,
	ROTATION_MODE
} Robot_Behavior_e;
typedef enum
{
	pitch_move_mode,
	pitch_stab_mode
} pitch_mode;
typedef enum
{
	CHASSIS_NO_FOLLOW_GIMBAL_MODE,
	CHASSIS_FOLLOW_GIMBAL_MODE,
	CHASSIS_ROTATE_MODE,
} Chassis_Behavior_e;

typedef enum
{
	RELATIVE_MODE,
	ABSOLUTE_MODE,
	VISION_MODE
} Motor_Behavior_e;

typedef enum
{
	INIT_START_BIT = 0,
	INIT_GYRO_START_BIT,
	INIT_GYRO_SUCCESSFUL_BIT,
	INIT_SUCCESSFUL_BIT
} Init_Flag_Bit_e;

typedef struct
{
	uint32_t model;

	uint32_t rc;
	uint32_t imu;
	uint32_t gimbal[3];
	uint32_t vision;
} Gimbal_Time_t;

typedef struct
{
	float fdb[3];
	float set[3];
	float filt[3]; //

} Gyro_t;

typedef struct
{
	float fdb[3];
	float set[3];
	float filt[3]; //

} Accel_t;

typedef struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} remote_control;

typedef struct
{
	float absolute_fdb[3];
	float absolute_set[3];

	float relative_fdb[3];
	float relative_set[3];

	float absolute_filt[3];
	float relative_filt[3];

	float ecd_angle_set[3];
	float ecd_angle_fdb[3];

	float one_head_angle;
} Angle_t;

typedef struct
{
	Motor_Feedback_t motor_fdb[2];

	Motor_Set_t motor_set[2];

	Robot_Behavior_e robot_mode_set;
	Robot_Behavior_e robot_last_mode;

	Chassis_Behavior_e chassis_mode;
	Chassis_Behavior_e chassis_last_mode;

	Motor_Behavior_e motor_mode[2];
	Motor_Behavior_e motor_last_mode[2];

	Gimbal_Time_t time_new;
	Gimbal_Time_t time_last;
	Gimbal_Time_t time_err;

	Vision_t vision;

	Angle_t angle;
	Gyro_t gyro;
	Accel_t accel;

	remote_control ref_rc_data;

	float AGV_set[3];

	int8_t init_flag;

	RC_ctrl_t rc_data;

	oblique_t oblique;

	remote_data_t new_rc_data;

	uint8_t robo_ID;

	uint8_t key_press_flag[20];
	uint8_t key_state[20];

	uint8_t new_rc_sw_flag[20];
	uint8_t new_rc_sw_state[20];
	uint8_t Cali_flag;
} Gimbal_t;

extern Gimbal_t gimbal;
extern void gimbal_task(void const *pvParameters);
void robot_iwdg_reset(void);

static void key_mode_press_fr1(remote_data_t *New_Rc_Data);
static void key_mode_press_fr2(remote_data_t *New_Rc_Data);
static void key_mode_press_pause(remote_data_t *New_Rc_Data);
static void key_mode_press_trigger(remote_data_t *New_Rc_Data);

static void key_mode_press_X(Gimbal_t *key_p);
static void key_mode_press_F(Gimbal_t *key_p);
static void key_mode_press_Q(Gimbal_t *key_p);
static void key_mode_press_G(Gimbal_t *key_p);
static void key_mode_press_V(Gimbal_t *key_p);

void key_mode_press(uint16_t Key_label, uint8_t key_press_flag, uint8_t key_press_state);
void new_rc_sw_press(uint64_t sw_label, uint8_t new_rc_sw_flag, uint8_t new_rc_sw_state);

void Get_FR_State(uint8_t *FR_State);
void Get_New_rc_FR_State(uint8_t *FR_State);
void Cali_buzzer(Gimbal_t *buzzer_det);
#endif
