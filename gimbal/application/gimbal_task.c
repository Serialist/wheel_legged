/************************
 * @file gimbal_task.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-12-05
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

/* ================================================================ include ================================================================*/

// 系统层
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "arm_math.h"
#include "AHRS_MiddleWare.h"
#include "cmsis_os.h"

// 外设层
#include "iwdg.h"
#include "usart.h"
#include "can.h"

// 数学运算层
#include "user_lib.h"
#include "constrain_calc.h"
#include "data_transform.h"
#include "average_filter.h"
#include "kalman_filter.h"
#include "pid.h"

// 应用硬件层
#include "bsp_buzzer.h"
#include "bsp_delay.h"
#include "bsp_can.h"
#include "referee.h"
#include "bsp_referee.h"
#include "usbd_cdc_if.h"
#include "remote_control.h"

// 任务层
#include "vision.h"
#include "INS_task.h"
#include "shoot_task.h"
#include "INS_task.h"
#include "gimbal_task.h"

/* ================================================================ macro ================================================================*/

/* ================================================================ typedef ================================================================*/

/* ================================================================ variable ================================================================*/

Gimbal_t gimbal;		   // 云台数据大结构体
remote_data_t New_Rc_Data; // 2025赛季RM官方提供的新遥控器数据结构体
Referee_info_t Gimbal_REF; // 云台裁判系统结构体
// uint8_t Cali_flag;//陀螺仪校准标志位，陀螺仪校准完之前都是无力模式防止疯转
// 测试变量，所有测试变量用det开头
float det_yaw_angle;
// 测试变量，所有测试变量用det开头

// PID结构体初始化,因为调pid要检测全局变量，所以所有pid的结构体都定义为全局变量
Gimbal_PID_t gimbal_shell_pid[2];
pid_type_def gimbal_core_pid[2];
pid_type_def vision_shell_pid[2];
pid_type_def vision_core_pid[2];
pid_type_def follow_pid;
// PID结构体初始化,因为调pid要检测全局变量，所以所有pid的结构体都定义为全局变量

// 经验系数值
float MOUSE_SCALE_FACTOR = 2500.f;
float New_Rc_FACTOR = 0.f;
float DT7_RC_FACTOR = 15.f;
// 经验系数值

uint8_t head_flag = HEAD;

// 各种均值和均值中位数滤波初始化
Average_type_def_t PITCH_Ave;
Average_type_def_t stab_Ave;
Average_type_def_t vision_yaw_Ave;
Average_type_def_t shell_yaw_Ave;
median_Ave_type_def_t M_Ave;
median_Ave_type_def_t M_pitch_stab_Ave;
// 各种均值和均值中位数滤波初始化

/* ================================================================ prototype ================================================================*/

static void GimbalInit(Gimbal_t *gimbal_init);
static void GetDataFdb(Gimbal_t *gimbal_fdb);
static void GimbalModeSet(Gimbal_t *gimbal_mode_set);
static void PitchAngleRefSet(Gimbal_t *pitch_angle_set);
static void YawAngleRefSet(Gimbal_t *yaw_angle_set);
static void PitchPidCalc(Gimbal_t *pitch_pid_calc);
static void YawPidCalc(Gimbal_t *yaw_pid_calc);
static void GimbalModeChangeControlTransit(Gimbal_t *gimbal_mode_change);
static void addPitchAngle(Gimbal_t *pitch, float add);
static void addYawAngle(Gimbal_t *yaw, float add);
static void pidCalc(Gimbal_t *pid_calc, Axis_e axis);
static void sentChassis(void);
void AGV_speed_set(Gimbal_t *speed_set);
void Key_Press(void);

/* ================================================================ function ================================================================*/

void gimbal_task(void const *pvParameters)
{
	GimbalInit(&gimbal);
	remote_control_init();

	while (1)
	{

		GetDataFdb(&gimbal);

		Cali_buzzer(&gimbal);

		Key_Press();

		GimbalModeSet(&gimbal);

		GimbalModeChangeControlTransit(&gimbal);

		// 给设定值的函数，其内嵌了一个下层函数
		PitchAngleRefSet(&gimbal);

		YawAngleRefSet(&gimbal);
		// 给设定值的函数，其内嵌了一个下层函数

		// 具体pid计算的函数，其内嵌了一个下层函数
		PitchPidCalc(&gimbal);

		YawPidCalc(&gimbal);
		// 具体pid计算的函数，其内嵌了一个下层函数

		sentGimbal(gimbal.motor_set[PITCH].current_set);
		// sentGimbal(0);

		AGV_speed_set(&gimbal);

		sentAGVspeed(gimbal.AGV_set[SPEED_X], gimbal.AGV_set[SPEED_Y], gimbal.AGV_set[SPEED_W]);

		sentChassis();

		osDelay(1);
	}
}

/************************
 * @brief 云台初始化
 *
 * @param gimbal_init
 ************************/
static void GimbalInit(Gimbal_t *gimbal_init)
{
	fp32 PID[3];

	PITCH_Ave.Aver_tot_num = 18;
	stab_Ave.Aver_tot_num = 50;
	vision_yaw_Ave.Aver_tot_num = 15;
	shell_yaw_Ave.Aver_tot_num = 10;
	gimbal_init->robot_mode_set = INIT_MODE;
	gimbal_init->init_flag = 0;

	gimbal_shell_pid[PITCH].kp = PITCH_SHELL_PID_KP;
	gimbal_shell_pid[PITCH].ki = PITCH_SHELL_PID_KI;
	gimbal_shell_pid[PITCH].kd = PITCH_SHELL_PID_KD;
	gimbal_shell_pid[PITCH].max_out = PITCH_SHELL_PID_MAX_OUT;
	gimbal_shell_pid[PITCH].max_iout = PITCH_SHELL_PID_MAX_IOUT;

	gimbal_shell_pid[YAW].kp = YAW_SHELL_PID_KP;
	gimbal_shell_pid[YAW].ki = YAW_SHELL_PID_KI;
	gimbal_shell_pid[YAW].kd = YAW_SHELL_PID_KD;
	gimbal_shell_pid[YAW].max_out = YAW_SHELL_PID_MAX_OUT;
	gimbal_shell_pid[YAW].max_iout = YAW_SHELL_PID_MAX_IOUT;

	vision_shell_pid[PITCH].Kp = PITCH_VISION_SHELL_PID_KP;
	vision_shell_pid[PITCH].Ki = PITCH_VISION_SHELL_PID_KI;
	vision_shell_pid[PITCH].Kd = PITCH_VISION_SHELL_PID_KD;
	vision_shell_pid[PITCH].max_out = PITCH_VISION_SHELL_PID_MAX_OUT;
	vision_shell_pid[PITCH].max_iout = PITCH_VISION_SHELL_PID_MAX_IOUT;

	vision_shell_pid[YAW].Kp = YAW_VISION_SHELL_PID_KP;
	vision_shell_pid[YAW].Ki = YAW_VISION_SHELL_PID_KI;
	vision_shell_pid[YAW].Kd = YAW_VISION_SHELL_PID_KD;

	vision_shell_pid[YAW].kf = YAW_VISION_SHELL_PID_KF;

	vision_shell_pid[YAW].max_out = YAW_VISION_SHELL_PID_MAX_OUT;
	vision_shell_pid[YAW].max_iout = YAW_VISION_SHELL_PID_MAX_IOUT;

	PID[0] = PITCH_CORE_PID_KP;
	PID[1] = PITCH_CORE_PID_KI;
	PID[2] = PITCH_CORE_PID_KD;
	PID_Init(&gimbal_core_pid[PITCH], PID_POSITION, PID, PITCH_CORE_PID_MAX_OUT, PITCH_CORE_PID_MAX_IOUT);

	PID[0] = YAW_CORE_PID_KP;
	PID[1] = YAW_CORE_PID_KI;
	PID[2] = YAW_CORE_PID_KD;
	PID_Init(&gimbal_core_pid[YAW], PID_POSITION, PID, YAW_CORE_PID_MAX_OUT, YAW_CORE_PID_MAX_IOUT);

	PID[0] = VISION_YAW_CORE_PID_KP;
	PID[1] = VISION_YAW_CORE_PID_KI;
	PID[2] = VISION_YAW_CORE_PID_KD;
	vision_core_pid[YAW].kf = VISION_YAW_CORE_PID_KF;
	PID_Init(&vision_core_pid[YAW], PID_POSITION, PID, VISION_YAW_CORE_PID_MAX_OUT, VISION_YAW_CORE_PID_MAX_IOUT);

	PID[0] = PITCH_VISION_CORE_PID_KP;
	PID[1] = PITCH_VISION_CORE_PID_KI;
	PID[2] = PITCH_VISION_CORE_PID_KD;
	PID_Init(&vision_core_pid[PITCH], PID_POSITION, PID, PITCH_VISION_CORE_PID_MAX_OUT, PITCH_VISION_CORE_PID_MAX_IOUT);

	PID[0] = FOLLOW_PID_KP;
	PID[1] = FOLLOW_PID_KI;
	PID[2] = FOLLOW_PID_KD;
	PID_Init(&follow_pid, PID_POSITION, PID, FOLLOW_PID_MAX_OUT, FOLLOW_PID_MAX_IOUT);
}

/************************
 * @brief 获取各种数据
 *
 * @param gimbal_fdb
 ************************/
static void GetDataFdb(Gimbal_t *gimbal_fdb)
{

	HAL_UART_GetState(&huart1);
	GetRCData(&(gimbal_fdb->rc_data));				   // 获取遥控器数据
	GetPitchMotorFdb(&(gimbal_fdb->motor_fdb[PITCH])); // 获取pitch电机数据
	GetYawMotorFdb(&(gimbal_fdb->motor_fdb[YAW]));	   // 获取yaw电机数据

	GetMotorPitchRelativeAngle(&(gimbal_fdb->angle.relative_fdb[PITCH])); // 获取pitch相对角度
	GetMotorYawRelativeAngle(&(gimbal_fdb->angle.relative_fdb[YAW]));
	Get_one_head_angle(&(gimbal_fdb->angle.one_head_angle));
	Get_Referee_Data(&Gimbal_REF);
	Get_Calibrate_flag(&gimbal_fdb->Cali_flag);
	// 四个头判断头尾
	//	if(gimbal_fdb->angle.relative_fdb[YAW]<0&&gimbal_fdb->angle.relative_fdb[YAW]>-90)//90??
	//		head_flag=HEAD;
	//	else if(gimbal_fdb->angle.relative_fdb[YAW]<180&&gimbal_fdb->angle.relative_fdb[YAW]>90)//-90??
	//		head_flag=TAIL, mode_state = 5;
	//	else if(gimbal_fdb->angle.relative_fdb[YAW]>-180&&gimbal_fdb->angle.relative_fdb[YAW]<-90)//-90??
	//		head_flag=LEFT;
	//	else if(gimbal_fdb->angle.relative_fdb[YAW]<90&&gimbal_fdb->angle.relative_fdb[YAW]>0)//-90??
	//		head_flag=RIGHT;
	// 四个头判断头尾

	// 两个头判断头尾
	if (gimbal_fdb->angle.one_head_angle > 0 && gimbal_fdb->angle.one_head_angle <= 180)
	{
		head_flag = HEAD;
	}
	else if (gimbal_fdb->angle.one_head_angle > 180 && gimbal_fdb->angle.one_head_angle <= 360)
	{
		head_flag = TAIL;
	}
	// 两个头判断头尾

	GetIMUYawAngleFdb(&(gimbal_fdb->angle.absolute_fdb[YAW]));
	GetIMUPitchAngleFdb(&(gimbal_fdb->angle.absolute_fdb[PITCH]));
	GetIMUPitchGyroFdb(&(gimbal_fdb->gyro.fdb[PITCH]));
	GetIMUYawGyroFdb(&(gimbal_fdb->gyro.fdb[YAW]));
	GetIMUPitchAccelFdb(&(gimbal_fdb->accel.fdb[PITCH]));
	GetIMUYawAccelFdb(&(gimbal_fdb->accel.fdb[YAW]));
	Get_New_Rc_data(&New_Rc_Data);
	Get_vision_Data(&(gimbal_fdb->vision));
}

/************************
 * @brief 获取模式
 *
 * @param gimbal_mode_set
 ************************/
static void GimbalModeSet(Gimbal_t *gimbal_mode_set)
{
	// P轴和Y轴模式切换和模式赋值
	gimbal_mode_set->robot_last_mode = gimbal_mode_set->robot_mode_set;
	gimbal_mode_set->motor_last_mode[PITCH] = gimbal_mode_set->motor_mode[PITCH];
	gimbal_mode_set->motor_last_mode[YAW] = gimbal_mode_set->motor_mode[YAW];

	// 当前操控模式选择DT7操控
	// 无力模式
	// if(gimbal_mode_set->rc_data.rc.s[0] == 1||New_Rc_Data.mode_sw==0||huart1.RxState==HAL_UART_STATE_READY)
	if (gimbal_mode_set->rc_data.rc.s[0] == 1 || huart3.RxState == HAL_UART_STATE_RESET || !(gimbal_mode_set->Cali_flag))
	{
		gimbal_mode_set->robot_mode_set = ZERO_FORCE_MODE;
		gimbal_mode_set->motor_mode[PITCH] = RELATIVE_MODE; // RELATIVE_MODE就是无力模式
		gimbal_mode_set->motor_mode[YAW] = RELATIVE_MODE;	// RELATIVE_MODE就是无力模式
	}

	// 正常模式
	else if (gimbal_mode_set->rc_data.rc.s[0] == 3 || New_Rc_Data.mode_sw == 1)
	{
		gimbal_mode_set->robot_mode_set = NORMAL_MODE;
		gimbal_mode_set->motor_mode[PITCH] = ABSOLUTE_MODE; //
		gimbal_mode_set->motor_mode[YAW] = ABSOLUTE_MODE;	//
															// 小陀螺模式
		if ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_SHIFT) > 0 || gimbal_mode_set->rc_data.rc.s[1] == 2 || gimbal.new_rc_sw_state[Pause] == 1)
		{
			gimbal_mode_set->chassis_mode = CHASSIS_ROTATE_MODE;
		}
		else
		{
			gimbal_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;
		}
	}
	//?????????????

	// 自瞄模式//
	else if (gimbal_mode_set->rc_data.rc.s[0] == 2 || (gimbal_mode_set->rc_data.mouse.press_r > 0) || (REF.RemoteControl.right_button_down > 0) || New_Rc_Data.mode_sw == 2)
	{
		gimbal_mode_set->robot_mode_set = VISION_FOLLOW;
		gimbal_mode_set->motor_mode[PITCH] = VISION_MODE;
		gimbal_mode_set->motor_mode[YAW] = VISION_MODE;

		// 自瞄下的小陀螺模式//
		if ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_SHIFT) > 0 || gimbal_mode_set->rc_data.rc.s[1] == 2 || gimbal.new_rc_sw_state[Pause] == 1)
		{
			gimbal_mode_set->chassis_mode = CHASSIS_ROTATE_MODE;
		}
		else
		{
			gimbal_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;
		}
	}
}

float absolte_angle_zero[3] = {0};
/************************
 * @brief 控制整车模式切换
 *
 * @param mode_change
 ************************/
static void GimbalModeChangeControlTransit(Gimbal_t *mode_change)
{

	if (mode_change->motor_last_mode[PITCH] != RELATIVE_MODE && mode_change->motor_mode[PITCH] == RELATIVE_MODE)
	{
		mode_change->angle.relative_set[PITCH] = mode_change->angle.relative_fdb[PITCH];
	}
	else if (mode_change->motor_last_mode[PITCH] != ABSOLUTE_MODE && mode_change->motor_mode[PITCH] == ABSOLUTE_MODE)
	{
		mode_change->angle.absolute_set[PITCH] = mode_change->angle.absolute_fdb[PITCH];
	}
	else if (mode_change->motor_last_mode[PITCH] != VISION_MODE && mode_change->motor_mode[PITCH] == VISION_MODE)
	{
		mode_change->angle.absolute_set[PITCH] = mode_change->angle.absolute_fdb[PITCH];
	}

	// yaw轴模式切换
	if (mode_change->motor_last_mode[YAW] != RELATIVE_MODE && mode_change->motor_mode[YAW] == RELATIVE_MODE)
	{
		mode_change->angle.relative_set[YAW] = mode_change->angle.relative_fdb[YAW];
	}
	else if (mode_change->motor_last_mode[YAW] != ABSOLUTE_MODE && mode_change->motor_mode[YAW] == ABSOLUTE_MODE)
	{
		mode_change->angle.absolute_set[YAW] = mode_change->angle.absolute_fdb[YAW];
	}
	else if (mode_change->motor_last_mode[YAW] != VISION_MODE && mode_change->motor_mode[YAW] == VISION_MODE)
	{
		mode_change->angle.absolute_set[YAW] = mode_change->angle.absolute_fdb[YAW];
	}

	if (mode_change->robot_last_mode == INIT_MODE && mode_change->robot_mode_set == ZERO_FORCE_MODE)
	{

		absolte_angle_zero[YAW] = mode_change->angle.absolute_fdb[YAW];
		absolte_angle_zero[PITCH] = mode_change->angle.absolute_fdb[PITCH];
	}
	// PITCH轴模式切换
	if (mode_change->robot_last_mode == ZERO_FORCE_MODE && mode_change->robot_mode_set != ZERO_FORCE_MODE)
	{
		mode_change->angle.relative_set[PITCH] = mode_change->angle.relative_fdb[PITCH];
		mode_change->angle.relative_set[YAW] = mode_change->angle.relative_fdb[YAW];

		mode_change->angle.absolute_set[PITCH] = mode_change->angle.absolute_fdb[PITCH];
		mode_change->angle.absolute_set[YAW] = mode_change->angle.absolute_fdb[YAW];
	}
}

/************************
 * @brief 根据模式的不同，在不同模式下选择不同增量值
 *
 * @param angle_set
 ************************/
static void PitchAngleRefSet(Gimbal_t *angle_set)
{
	float add_angle = 0;

	if (angle_set->robot_mode_set == ZERO_FORCE_MODE)
	{
		return;
	}
	else if (angle_set->robot_mode_set == INIT_MODE)
	{
		add_angle = 0.0f;
		addPitchAngle(angle_set, add_angle);
	}
	else if (angle_set->robot_mode_set == NORMAL_MODE)
	{
		add_angle = (float)(-(angle_set->rc_data.rc.ch[1]) / (-CHANNEL_SCALE_FACTOR)) - (float)angle_set->rc_data.mouse.y / MOUSE_SCALE_FACTOR - (float)Gimbal_REF.RemoteControl.mouse_y / 4000.f - 0 * ((float)((New_Rc_Data.ch_1 - 1024))) / (CHANNEL_SCALE_FACTOR);

		addPitchAngle(angle_set, add_angle);
	}
	else if (angle_set->robot_mode_set == VISION_FOLLOW)

	{
		add_angle = -gimbal.vision.angle_set_err[PITCH];
		addPitchAngle(angle_set, add_angle);
	}
}

/************************
 * @brief 根据模式的不同，在不同模式下选择不同增量值
 *
 * @param angle_set
 ************************/
static void YawAngleRefSet(Gimbal_t *angle_set)
{
	float add_angle = 0.0f;
	if (angle_set->robot_mode_set == ZERO_FORCE_MODE)
	{
		return;
	}
	else if (angle_set->robot_mode_set == INIT_MODE)
	{
		add_angle = 0.0f;
		addYawAngle(angle_set, add_angle);
	}
	else if (angle_set->robot_mode_set == NORMAL_MODE)
	{
		add_angle = (float)(-(angle_set->rc_data.rc.ch[0])) / (CHANNEL_SCALE_FACTOR) - (float)angle_set->rc_data.mouse.x / MOUSE_SCALE_FACTOR + (float)Gimbal_REF.RemoteControl.mouse_x / MOUSE_SCALE_FACTOR;
		//+((float)(-(New_Rc_Data.ch_0-1024)))/(CHANNEL_SCALE_FACTOR);
		addYawAngle(angle_set, add_angle);
	}
	else if (angle_set->robot_mode_set == VISION_FOLLOW)
	{
		add_angle = gimbal.vision.angle_set_err[YAW];

		addYawAngle(angle_set, add_angle);
	}
}

/************************
 * @brief 在不同模式下选择的计算方式不同，但其实具体的选择在其下层函数pidCalc(pitch_pid_calc, PITCH)已有体现
 *
 * @param pitch_pid_calc
 ************************/

static void PitchPidCalc(Gimbal_t *pitch_pid_calc)
{
	if (pitch_pid_calc->robot_mode_set == ZERO_FORCE_MODE)
	{
		pitch_pid_calc->motor_set[PITCH].current_set = 0;
	}
	else if (pitch_pid_calc->robot_mode_set == INIT_MODE)
	{
		pidCalc(pitch_pid_calc, PITCH);
	}
	else if (pitch_pid_calc->robot_mode_set == NORMAL_MODE)
	{
		pidCalc(pitch_pid_calc, PITCH);
	}
	else if (pitch_pid_calc->robot_mode_set == VISION_FOLLOW)
	{
		pidCalc(pitch_pid_calc, PITCH);
	}
}

/************************
 * @brief 在不同模式下选择的计算方式不同，但其实具体的选择在其下层函数pidCalc(pitch_pid_calc, PITCH)已有体现
 *
 * @param yaw_pid_calc
 ************************/

static void YawPidCalc(Gimbal_t *yaw_pid_calc)
{
	if (yaw_pid_calc->robot_mode_set == ZERO_FORCE_MODE)
	{
		yaw_pid_calc->motor_set[YAW].current_set = 0;
	}
	// 在pidCalc(yaw_pid_calc, YAW)函数中做选择，所以这里的三个函数是一模一样的
	else if (yaw_pid_calc->robot_mode_set == INIT_MODE)
	{
		pidCalc(yaw_pid_calc, YAW);
	}
	else if (yaw_pid_calc->robot_mode_set == NORMAL_MODE)
	{
		pidCalc(yaw_pid_calc, YAW);
	}
	else if (yaw_pid_calc->robot_mode_set == VISION_FOLLOW)
	{
		pidCalc(yaw_pid_calc, YAW);
	}
}

/************************
 * @brief 给PITCH设定值增量的函数,具体的增量是什么依赖上层的函数，增量式的设定值，加到absolute_set里面
 *
 * @param pitch
 * @param add
 ************************/
static void addPitchAngle(Gimbal_t *pitch, float add)
{
	// 用IMU解算的pitch轴角度值做电控限位
	//>0 17
	//<0 -10
	if (pitch->motor_mode[PITCH] == ABSOLUTE_MODE)
	{

#if (ROBOT_ID == ROBOT_Infantry)
		if (pitch->angle.absolute_fdb[PITCH] <= -25.f && ((pitch->rc_data.rc.ch[1] < 0) || Gimbal_REF.RemoteControl.mouse_y > 0))
		{
			pitch->angle.absolute_set[PITCH] -= add;
			;
		}
		else if (pitch->angle.absolute_fdb[PITCH] >= 8.f && ((pitch->rc_data.rc.ch[1] > 0) || Gimbal_REF.RemoteControl.mouse_y < 0))
		{
			pitch->angle.absolute_set[PITCH] -= add;
			;
		}
#elif (ROBOT_ID == ROBOT_Sentry)
		if (pitch->angle.absolute_fdb[PITCH] <= -10.f && ((pitch->rc_data.rc.ch[1] < 0) || Gimbal_REF.RemoteControl.mouse_y > 0))
		{
			pitch->angle.absolute_set[PITCH] -= add;
			;
		}
		else if (pitch->angle.absolute_fdb[PITCH] >= 17.f && ((pitch->rc_data.rc.ch[1] > 0) || Gimbal_REF.RemoteControl.mouse_y < 0))
		{
			pitch->angle.absolute_set[PITCH] -= add;
			;
		}
#endif
		pitch->angle.absolute_set[PITCH] += add;
	}
	// 用IMU解算的pitch轴角度值做电控限位

	else if (pitch->motor_mode[PITCH] == RELATIVE_MODE)
	{
		pitch->angle.relative_set[PITCH] += add;
		MinMaxConstrain(&pitch->angle.relative_set[PITCH], PITCH_RELATIVE_MIN_ANGLE, PITCH_RELATIVE_MAX_ANGLE);
	}

	else if (pitch->motor_mode[PITCH] == VISION_MODE)
	{
		pitch->angle.absolute_set[PITCH] += add;
		if (pitch->motor_fdb[PITCH].ecd_fdb >= max_pitch_ecd && gimbal.vision.angle_set_err[1] > 0)
		{
			pitch->angle.absolute_set[PITCH] -= add;
		}
		if (pitch->motor_fdb[PITCH].ecd_fdb <= min_pitch_ecd && gimbal.vision.angle_set_err[1] < 0)
		{
			pitch->angle.absolute_set[PITCH] -= add;
		}
		// 其实这里不太用过零检测，因为pitch的角度变动范围很小
		ConstrainLoop(&(pitch->angle.absolute_set[PITCH]), pitch->angle.absolute_set[PITCH], ANGLE_MIN, ANGLE_MAX);
	}
}

/************************
 * @brief 给YAW设定值的函数，具体的增量是什么依赖上层的函数，增量式的设定值，加到absolute_set里面
 *
 * @param yaw
 * @param add
 ************************/
static void addYawAngle(Gimbal_t *yaw, float add)
{
	if (yaw->motor_mode[YAW] == ABSOLUTE_MODE)
	{
		yaw->angle.absolute_set[YAW] += add;
		// 确保yaw轴的值从-180到+180
		ConstrainLoop(&(yaw->angle.absolute_set[YAW]), yaw->angle.absolute_set[YAW], ANGLE_MIN, ANGLE_MAX);
	}
	else if (yaw->motor_mode[YAW] == RELATIVE_MODE)
	{
		yaw->angle.relative_set[YAW] += add;
		MinMaxConstrain(&yaw->angle.relative_set[YAW], YAW_RELATIVE_MIN_ANGLE, YAW_RELATIVE_MAX_ANGLE);
	}
	else if (yaw->motor_mode[YAW] == VISION_MODE)
	{
		yaw->angle.absolute_set[YAW] += add;
		// 确保yaw轴的值从-180到+180
		ConstrainLoop(&(yaw->angle.absolute_set[YAW]), yaw->angle.absolute_set[YAW], ANGLE_MIN, ANGLE_MAX);
	}
}

uint8_t turn_round_flag;
/************************
 * @brief pid计算函数
 *
 * @param pid_calc
 * @param axis
 ************************/
static void pidCalc(Gimbal_t *pid_calc, Axis_e axis)
{
	float angle_set = 0.0f;
	// 正常模式
	if (pid_calc->motor_mode[axis] == ABSOLUTE_MODE)
	{

		if (axis == PITCH)
		{
			angle_set = PreventFullCircleConstrain(pid_calc->angle.absolute_set[PITCH], pid_calc->angle.absolute_fdb[PITCH]);
			// 位置环
			// 这个GIMBAL_PID_Calc和普通pid是一样使用的
			pid_calc->gyro.set[PITCH] = GIMBAL_PID_Calc(&gimbal_shell_pid[PITCH], pid_calc->angle.absolute_fdb[PITCH], angle_set, pid_calc->accel.fdb[PITCH]);
			// 速度环
			pid_calc->motor_set[PITCH].current_set = PID_calc(&gimbal_core_pid[PITCH], pid_calc->gyro.set[PITCH], pid_calc->gyro.fdb[PITCH]);
		}

		else if (axis == YAW)
		{
			if (gimbal.robot_mode_set == INIT_MODE)
			{
				angle_set = 0;
			}
			angle_set = PreventFullCircleConstrain(pid_calc->angle.absolute_set[YAW], pid_calc->angle.absolute_fdb[YAW]);
			det_yaw_angle = PreventFullCircleConstrain(pid_calc->angle.absolute_set[YAW], pid_calc->angle.absolute_fdb[YAW]);
			// 积分分离，后续可封装成额外的函数
			if (ABS(gimbal_shell_pid[YAW].err[0]) > 0.2)
			{
				gimbal_shell_pid[YAW].Iout = 0;
				pid_calc->gyro.set[YAW] = GIMBAL_PID_Calc(&gimbal_shell_pid[YAW], pid_calc->angle.absolute_fdb[YAW], det_yaw_angle, 0);
			}
			// 积分分离，后续可封装成额外的函数
			else
			{
				// 位置环
				// 这个GIMBAL_PID_Calc和普通pid是一样使用的
				pid_calc->gyro.set[YAW] = GIMBAL_PID_Calc(&gimbal_shell_pid[YAW], pid_calc->angle.absolute_fdb[YAW], det_yaw_angle, 0);
			}
			// 速度环
			pid_calc->motor_set[YAW].current_set = PID_calc(&gimbal_core_pid[YAW], pid_calc->gyro.set[YAW], pid_calc->gyro.fdb[YAW]);
		}
	}

	// 无力模式
	else if (pid_calc->motor_mode[axis] == RELATIVE_MODE)
	{
		pid_calc->gyro.set[axis] = GIMBAL_PID_Calc(&gimbal_shell_pid[axis], pid_calc->angle.relative_fdb[axis], angle_set, pid_calc->accel.fdb[axis]);
		pid_calc->motor_set[axis].current_set = PID_calc(&gimbal_core_pid[axis], pid_calc->gyro.set[axis], pid_calc->gyro.fdb[axis]);
	}
	// 无力模式

	// 自瞄模式，将控制权交给视觉系统
	else if (pid_calc->motor_mode[axis] == VISION_MODE)
	{
		if (axis == PITCH)
		{
			pid_calc->gyro.set[PITCH] = PID_calc(&vision_shell_pid[PITCH], 0, gimbal.vision.angle_set_err[1]);
			/// 积分分离
			if (ABS(vision_shell_pid[PITCH].error[0]) > 0.1)
			{
				vision_shell_pid[PITCH].Iout = 0;
			}
			/// 积分分离
			pid_calc->motor_set[PITCH].current_set = PID_calc(&vision_core_pid[PITCH], pid_calc->gyro.set[PITCH], pid_calc->gyro.fdb[PITCH]);

			// 当自瞄传来数据为0时，即未检测到目标，电流给0
			if (gimbal.vision.angle_set_err[1] == 0)
			{
				pid_calc->motor_set[PITCH].current_set = 0;
			}
		}
		if (axis == YAW)
		{
			angle_set = PreventFullCircleConstrain(pid_calc->angle.absolute_set[YAW], pid_calc->angle.absolute_fdb[YAW]);
			// 之前测试当小陀螺时自瞄会有一个向左还是向右的静态误差，这个误差跟转速有关，
			// 原因可能是小陀螺转时对云台有一个持续的力所以在小陀螺模式时加补偿
			if (gimbal.chassis_mode == CHASSIS_ROTATE_MODE)
			{
				// 位置环
				pid_calc->gyro.set[YAW] = PID_calc(&vision_shell_pid[YAW], -gimbal.vision.angle_set_err[0] - 1.8f * 1.3f * (float)(gimbal.motor_fdb[YAW].speed_rpm_fdb) / 95.f, 0);
			}
			else
			{ // 位置环
				pid_calc->gyro.set[YAW] = PID_calc(&vision_shell_pid[YAW], -gimbal.vision.angle_set_err[0], 0);
			}

			/// 积分分离
			if (ABS(vision_shell_pid[YAW].error[0]) > 0.1)
			{
				vision_shell_pid[YAW].Iout = 0;
			}
			/// 积分分离
			// 速度环
			pid_calc->motor_set[YAW].current_set = PID_calc(&vision_core_pid[YAW], pid_calc->gyro.set[YAW], pid_calc->gyro.fdb[YAW]);
		}
	}
}

float angle_kff = 8;
/************************
 * @brief 根据云台模式和遥控器或键盘的控制信息解算出底盘此时三个方向（XYW）的速度值
 *
 * @param speed_set
 *
 * @note
 * 开小陀螺的时候旋转矩阵不断变化,会有以下影响
 *
 * 1. 斜坡函数的加速度要给大，如果给太小还没加速完旋转矩阵已经变了，运动起来就会和正方向有很大的偏差
 * 2. 要给用编码器解算的角度加一个前馈量，因为编码器给的值传输速率低，如果小陀螺转速大，
 *    当前解算的角度到传给电机时角度已经又增大了，所以加一个固定值补偿，该值应与转速和前进速度成某种关系，
 *    2025赛季并未仔细测量，只是给一个固定值。
 ************************/
void AGV_speed_set(Gimbal_t *speed_set)
{

	// 键盘操控时移动系数，因为键盘控制只有0和1的区别，所以要放大，系数跟操作手的习惯和斜坡函数有关
	static float KEY_SCALE_FACTOR = 0.f;
	// 小陀螺转速
	static float spin_speed = 0;
	// 遥控器值直接赋给云底联动pid输出的前馈系数（可能不太能叫前馈把）
	// 此项可称为一个前馈值，首先遥控器控制云台，底盘再跟着云台转,底盘跟着云台转通过一个云底联动的pid实现
	// 所以直接把遥控器的值乘一个系数付给W速度。

	// 小陀螺旋转时因为旋转矩阵滞后的前馈系数
	static float spin_cut = 0.002f;
	// 小陀螺旋转速度，速度要随等级线性增加
	spin_speed = 5300 + shoot.referee.robot_level * 250;

	// 无力模式，所有数据均为0
	if (speed_set->robot_mode_set == ZERO_FORCE_MODE)
	{
		speed_set->AGV_set[SPEED_X] = 0;
		speed_set->AGV_set[SPEED_Y] = 0;
		speed_set->AGV_set[SPEED_W] = 0;
	}
	// 云底跟随正常模式
	else if (speed_set->chassis_mode == CHASSIS_FOLLOW_GIMBAL_MODE)
	{
		// 键盘操控时移动系数，以为键盘控制只有0和1的区别，所以要放大，系数给操作手的习惯和斜坡函数有关
		KEY_SCALE_FACTOR = 10000.f;
		int16_t x_temp;
		int16_t y_temp;
		if (speed_set->rc_data.rc.s[1] == 1) // 导航模式，先这么写
		{
			x_temp = speed_set->vision.x_go + speed_set->rc_data.rc.ch[2] - ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR -
					 ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR; //??????

			y_temp = speed_set->rc_data.rc.ch[3] + ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR + ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR;
		}
		else
		{
			x_temp = speed_set->vision.y_go + speed_set->rc_data.rc.ch[2] - ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR -
					 ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR; //??????

			y_temp = speed_set->rc_data.rc.ch[3] + ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR + ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR;
		}
		speed_set->AGV_set[SPEED_X] = DT7_RC_FACTOR * (int16_t)(y_temp * arm_sin_f32(speed_set->angle.relative_fdb[YAW] * ANGLE_TO_RAD) + x_temp * arm_cos_f32(speed_set->angle.relative_fdb[YAW] * ANGLE_TO_RAD));
		speed_set->AGV_set[SPEED_Y] = DT7_RC_FACTOR * (int16_t)(y_temp * arm_cos_f32(speed_set->angle.relative_fdb[YAW] * ANGLE_TO_RAD) - x_temp * arm_sin_f32(speed_set->angle.relative_fdb[YAW] * ANGLE_TO_RAD));

		if (head_flag == HEAD)
		{
			// 当目标和当前值相差0.5以外时积分输出为0，防止积分累积过大而超调
			// 即积分分离
			if (ABS(follow_pid.error[0]) > 0.5)
			{
				follow_pid.Iout = 0;
			}
			// 添加0.5度的死区
			speed_set->AGV_set[SPEED_W] = PID_dead_zone_calc(&follow_pid, 90, gimbal.angle.one_head_angle, 0.1) +
										  angle_kff * gimbal_shell_pid[YAW].out;
			// angle_kff*new_yaw_shell_pid.out,此项可称为一个前馈值，首先遥控器控制云台，底盘再跟着云台转                                                                    //所以直接把遥控器的值乘一个系数付给W速度。
		}
		if (head_flag == TAIL)
		{
			// 当目标和当前值相差0.5以外时积分输出为0，防止积分累积过大而超调
			// 即积分分离
			if (ABS(follow_pid.error[0]) > 0.5)
			{
				follow_pid.Iout = 0;
			}
			// 因为飞坡的时候必须是90度的头为正方向，所以设置按键F，当此时正方向为270时按下F则切换到90度的头
			if ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_F) > 0)
			{
				// 添加0.5度的死区
				speed_set->AGV_set[SPEED_W] = PID_dead_zone_calc(&follow_pid, 90, gimbal.angle.one_head_angle, 0.1) +
											  angle_kff * gimbal_shell_pid[YAW].out;
			}
			else
			{
				// 添加0.5度的死区
				speed_set->AGV_set[SPEED_W] = PID_dead_zone_calc(&follow_pid, 270, gimbal.angle.one_head_angle, 0.1) +
											  angle_kff * gimbal_shell_pid[YAW].out;
			}
		}
	}
	// 小陀螺模式
	else if (speed_set->chassis_mode == CHASSIS_ROTATE_MODE)
	{
		KEY_SCALE_FACTOR = 3000;
		// 开启超电时小陀螺转速拉高，相应前馈系数也拉高
		if (gimbal.key_state[key_C])
		{
			spin_cut = 0.002f;
			spin_speed = 7500;
		}
		else
		{
			spin_cut = 0.001f;
		}

		int16_t x_temp = (New_Rc_Data.ch_3 - 1024) * New_Rc_FACTOR + speed_set->rc_data.rc.ch[2] * DT7_RC_FACTOR - ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR - ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR;

		int16_t y_temp = (New_Rc_Data.ch_2 - 1024) * New_Rc_FACTOR + speed_set->rc_data.rc.ch[3] * DT7_RC_FACTOR + ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((speed_set->rc_data.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR + ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR;
		// 小陀螺反转，键盘未设置此功能
		if (speed_set->rc_data.rc.ch[4] <= -660 || (New_Rc_Data.wheel - 1024) <= -660)
		{
			speed_set->AGV_set[SPEED_W] = -spin_speed;
		}
		else
		{
			speed_set->AGV_set[SPEED_W] = spin_speed;
		}
		// 开小陀螺后，底盘直走时会出现一点程度的歪斜，原因是底盘XY速度的解算要依赖旋转矩阵
		// 开小陀螺旋转矩阵变化过快，会导致XY速度解算出现误差，所以要给XY速度加一个前馈系数，
		// 该系数与转速和前进速度成某种关系，2025赛季并未仔细测量，只是给一个固定值。
		float feeforward_angle = speed_set->angle.relative_fdb[YAW] + spin_cut * speed_set->AGV_set[SPEED_W];

		speed_set->AGV_set[SPEED_X] = (int16_t)(y_temp * arm_sin_f32(feeforward_angle * ANGLE_TO_RAD) + x_temp * arm_cos_f32(feeforward_angle * ANGLE_TO_RAD));
		speed_set->AGV_set[SPEED_Y] = (int16_t)(y_temp * arm_cos_f32(feeforward_angle * ANGLE_TO_RAD) - x_temp * arm_sin_f32(feeforward_angle * ANGLE_TO_RAD));
	}
}

/************************
 * @brief 新遥控器模式
 *
 * @param sw_label
 * @param new_rc_sw_flag
 * @param new_rc_sw_state
 ************************/
void new_rc_sw_press(uint64_t sw_label, uint8_t new_rc_sw_flag, uint8_t new_rc_sw_state)
{
	if (sw_label == 1)
	{
		new_rc_sw_flag = 1;
	}
	if (new_rc_sw_flag && (sw_label == 0))
	{
		if (new_rc_sw_state == 0)
		{
			new_rc_sw_state = 1;
		}
		else
		{
			new_rc_sw_state = 0;
		}
		new_rc_sw_flag = 0;
	}
}

/************************
 * @brief 按键模式
 *
 * @param Key_label
 * @param key_press_flag
 * @param key_press_state
 ************************/
void key_mode_press(uint16_t Key_label, uint8_t key_press_flag, uint8_t key_press_state)
{
	if (((gimbal.rc_data.key.v & Key_label) > 0) || ((Gimbal_REF.RemoteControl.keyboard_value & Key_label) > 0))
	{
		key_press_flag = 1;
	}
	if (key_press_flag == 1 && (((gimbal.rc_data.key.v & Key_label) < 0) || ((Gimbal_REF.RemoteControl.keyboard_value & Key_label) <= 0)))
	{
		if (key_press_state == 0)
		{
			key_press_state = 1;
		}
		else
		{
			key_press_state = 0;
		}
		key_press_flag = 0;
	}
}

/************************
 * @brief 类别只有两种，第一个是DT7映射的键鼠，另一个是裁判系统映射的键鼠，新图传上的按键是和上面两个类别分开的
 *
 ************************/
void Key_Press(void)
{
	key_mode_press(KEY_PRESSED_OFFSET_Q, gimbal.key_press_flag[key_Q], gimbal.key_state[key_Q]); // 枪口热量控制
	key_mode_press(KEY_PRESSED_OFFSET_B, gimbal.key_press_flag[key_B], gimbal.key_state[key_B]); // UI刷新
	key_mode_press(KEY_PRESSED_OFFSET_C, gimbal.key_press_flag[key_C], gimbal.key_state[key_C]); // 超级电容
	key_mode_press(KEY_PRESSED_OFFSET_X, gimbal.key_press_flag[key_X], gimbal.key_state[key_X]); // 一键掉头
	key_mode_press(KEY_PRESSED_OFFSET_E, gimbal.key_press_flag[key_E], gimbal.key_state[key_E]); // 摩擦轮

	//	  new_rc_sw_press(New_Rc_Data.trigger,(gimbal.new_rc_sw_flag[trigger]),(gimbal.new_rc_sw_state[trigger]));
	//	  new_rc_sw_press(New_Rc_Data.fn_1,gimbal.new_rc_sw_flag[fr_1],gimbal.new_rc_sw_state[fr_1]);
	//	  new_rc_sw_press(New_Rc_Data.fn_2,gimbal.new_rc_sw_flag[fr_2],gimbal.new_rc_sw_state[fr_2]);
	//    new_rc_sw_press(New_Rc_Data.Pause,gimbal.new_rc_sw_flag[Pause],gimbal.new_rc_sw_state[Pause]);
}

/************************
 * @brief 刷新看门狗，2025赛季已配置但未使用
 *
 ************************/
void robot_iwdg_reset(void)
{
	HAL_IWDG_Refresh(&hiwdg);
}

/************************
 * @brief 发送底盘控制数据，包括各种标志位和控制标志位，YAW轴电流和YAW轴和PITCH轴编码器角度值
 *
 * @note
 * 发送所有标志位只用一个八位的数据，因为标志位只有零和一两种状态，则用位运算将他们合在一个八位数据发送
 ************************/
static void sentChassis()
{
	static fp32 temp_gimbal_angle[2];
	CAN_TxHeaderTypeDef tx_message;
	uint8_t gimbal_data[8];
	uint8_t reserve = 0;
	uint32_t send_mail_box;
	tx_message.StdId = 0x306;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	// 通过位运算给底盘传各种标志位用于UI显示
	if (Gimbal_REF.RemoteControl.right_button_down > 0 || New_Rc_Data.mode_sw == 2) // 自瞄标志位
	{
		gimbal_data[2] |= (1 << 0);
	}
	if ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_SHIFT) > 0 || gimbal.new_rc_sw_state[Pause]) // 小陀螺标志位
	{
		gimbal_data[2] |= (1 << 1);
	}
	if (gimbal.key_state[key_E]) // 摩擦轮开关标志位
	{
		gimbal_data[2] |= (1 << 2);
	}
	if ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_B) > 0) // UI刷新控制网
	{
		gimbal_data[2] |= (1 << 3);
	}
	if (gimbal.key_state[key_C]) // 超级电容标志位
	{
		gimbal_data[2] |= (1 << 4);
	}
	if ((Gimbal_REF.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_X) > 0) // X一键掉头
	{
		gimbal_data[2] |= (1 << 5);
	}
	if (gimbal.key_state[key_Q]) // 启不启用枪口热量控制
	{
		gimbal_data[2] |= (1 << 6);
	}
	// 发送YAW轴电机控制电流
	gimbal_data[0] = (gimbal.motor_set[YAW].current_set) >> 8;
	gimbal_data[1] = gimbal.motor_set[YAW].current_set;

	// 处理PITCH轴和YAW轴编码器角度值，使其范围变为0-360度

	if (gimbal.angle.absolute_fdb[PITCH] < 0)
	{
		temp_gimbal_angle[PITCH] = -gimbal.angle.absolute_fdb[PITCH] + 180;
	}
	else
	{
		temp_gimbal_angle[PITCH] = gimbal.angle.absolute_fdb[PITCH];
	}
	if (gimbal.angle.absolute_fdb[YAW] < 0)
	{
		temp_gimbal_angle[YAW] = -gimbal.angle.absolute_fdb[YAW] + 180;
	}
	else
	{
		temp_gimbal_angle[YAW] = gimbal.angle.absolute_fdb[YAW];
	}
	// 处理PITCH轴和YAW轴编码器角度值，使其范围变为0-360度

	// 发送YAW轴和PITCH轴编码器角度值
	gimbal_data[3] = (FLOAT_TO_INT16(temp_gimbal_angle[PITCH] * 100)) >> 8;
	gimbal_data[4] = (FLOAT_TO_INT16(temp_gimbal_angle[PITCH] * 100));
	gimbal_data[5] = (FLOAT_TO_INT16(temp_gimbal_angle[YAW] * 100)) >> 8;
	gimbal_data[6] = (FLOAT_TO_INT16(temp_gimbal_angle[YAW] * 100));
	gimbal_data[7] = reserve;
	HAL_CAN_AddTxMessage(&hcan2, &tx_message, gimbal_data, &send_mail_box);
}

/************************
 * @brief 获取按键E是否按下，按键E控制摩擦轮的状态
 *
 * @param FR_State
 ************************/
void Get_FR_State(uint8_t *FR_State)
{
	*FR_State = (gimbal.key_state[key_E]);
}

/************************
 * @brief 新图传上的按键E是否按下，按键E控制摩擦轮的状态
 *
 * @param FR_State
 ************************/
void Get_New_rc_FR_State(uint8_t *FR_State)
{
	*FR_State = (gimbal.new_rc_sw_state[fr_1]);
}

/************************
 * @brief 蜂鸣器
 *
 * @param buzzer_det
 ************************/
void Cali_buzzer(Gimbal_t *buzzer_det)
{
	if (!buzzer_det->Cali_flag)
	{
		buzzer_on(84 - 1, 8000);
	}
	else
	{
		buzzer_off();
	}
}
