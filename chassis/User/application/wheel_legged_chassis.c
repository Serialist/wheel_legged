/***********************************************
 * @file wheel_legged_chassis.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-16
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 * oooooo     oooo   .oooooo.    oooooooooo.
 *  `888.     .8'   d8P'  `Y8b   `888'   `Y8b
 *   `888.   .8'   888            888      888
 *    `888. .8'    888            888      888
 *     `888.8'     888     ooooo  888      888
 *      `888'      `88.    .88'   888     d88'
 *       `8'        `Y8bood8P'   o888bood8P'
 *
 *************************************************/

#include "wheel_legged_chassis.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "pid.h"
#include "INS_task.h"
#include <math.h>
#include "bsp_dwt.h"
#include "dt7.h"
#include "user_lib.h"
#include "vmc-dm.h"
#include "can.h"
#include "debug.h"
#include "filter.h"
#include "motor.h"

extern AK_motor_ctrl_fdb_t motorAK10[6];

Chassis_t chassis;

PID_Typedef pid_tpl = {0},
			pid_tpr = {0},
			leglength_pid_l = {0},
			leglength_pid_r = {0},
			yaw_pid = {0},
			roll_pid = {0},
			tp_pid = {0};

Wheel_Leg_Target_t set;
VMC_t leg_l, leg_r;

/* ======================== he ======================== */

float turn_t; // yaw轴补偿

float tplqrl;
float tplqrr;
float tlqrl;
float tlqrr;

float lqr_k_l[2][6], lqr_k_r[2][6], xl[6], xr[6];

/// @brief 离地检测滤波
Filter_Average_t ground_detection_filter_l, ground_detection_filter_r;

static void ChassisInit(void);
void Chassis_Motor_Transmit(Chassis_t *ch);
void LQR_Control(Chassis_t *ch);
void Control_Get(Chassis_t *ch);
void Clamp(float *in, float min, float max);
void LQR_K_Calc(float k[2][6], float coe[12][4], float len);
void Motor_Enable(void);

void Chassis_Task(void const *argument)
{
	ChassisInit();
	VMC_Init(&leg_l);
	VMC_Init(&leg_r);

	for (;;)
	{
		/* ================================ 状态更新 ================================ */

		Control_Get(&chassis);
		LQR_Control(&chassis);

		/* ================================ 电机控制指令 ================================ */

		Chassis_Motor_Transmit(&chassis);
	}
}

static void ChassisInit(void)
{
	PID_init(&leglength_pid_l, PID_POSITION, 500, 0, 9000, 120, 0); // 腿长 left
	PID_init(&leglength_pid_r, PID_POSITION, 500, 0, 9000, 120, 0); // 腿长 right
	PID_init(&yaw_pid, PID_POSITION, 0.1f, 0, 0.5f, 1.5f, 0);		// yaw
	PID_init(&roll_pid, PID_POSITION, 0.8f, 0, 0, 30.0f, 0);		// roll
	PID_init(&tp_pid, PID_POSITION, 1.3, 0, 3, 1.5, 0);				// 劈叉

	// 腿摆角扭矩pid，用于板凳模型
	PID_init(&pid_tpl, PID_POSITION, 80, 0, 400, 10, 0);
	PID_init(&pid_tpr, PID_POSITION, 80, 0, 400, 10, 0);

	// 离地检测滤波
	Filter_Average_Init(&ground_detection_filter_l, 10);
	Filter_Average_Init(&ground_detection_filter_r, 10);

	set.left_length = set.right_length = 0.15f;

	Motor_Enable();
}

/************************
 * @brief 初始化电机控制
 *
 ************************/
void Motor_Enable(void)
{
	for (int a = 1; a <= 4; a++)
	{
		controller_init(a);
		osDelay(1);
	}
}

void chassis_sys_calc(Chassis_t *ch)
{
	VMC_calc_1(&leg_l, &chassis, 3.0f / 1000.0f);
	leg_l.phi1 = pi / 2.0f - motorAK10[2].angle;
	leg_l.phi4 = pi / 2.0f - motorAK10[3].angle;

	VMC_calc_1(&leg_r, &chassis, 3.0f / 1000.0f);
	leg_r.phi1 = pi / 2.0f - motorAK10[0].angle;
	leg_r.phi4 = pi / 2.0f - motorAK10[1].angle;
}

void Chassis_Motor_Transmit(Chassis_t *ch)
{
	if (rc_ctrl.rc.s[S_R] != DOWN)
	{
		for (int i = 0; i < 6; i++)
			ch->ak_set[i].torset = 0;
	}

	/// @brief 用 3508
	RM_Motor_Transmit(&hcan1, M3508_TX_ID_2,
					  0,
					  HEXROLL_TORQUE_TO_CURRENT(ch->ak_set[4].torset),
					  HEXROLL_TORQUE_TO_CURRENT(ch->ak_set[5].torset),
					  0);
	osDelay(1);

	// MIT模式下发送
	AK_MIT_Transmit(1, 0, 0, 0, 0, ch->ak_set[0].torset);
	AK_MIT_Transmit(3, 0, 0, 0, 0, ch->ak_set[3].torset);
	osDelay(1);
	AK_MIT_Transmit(2, 0, 0, 0, 0, ch->ak_set[1].torset);
	AK_MIT_Transmit(4, 0, 0, 0, 0, ch->ak_set[2].torset);
	osDelay(1);
}

void Control_Get(Chassis_t *ch)
{
	ch->robo_status.last_behavior = ch->robo_status.behavior;

	if (rc_ctrl.rc.s[S_R] == DOWN) // 正常行驶
	{
		set.v = rc_ctrl.rc.ch[L_Y] * 1.0f / 660.0f;
		set.yaw -= rc_ctrl.rc.ch[L_X] * REMOTE_CHANNLE_TO_CHASSIS_SPEED * 2.0f;
		set.left_length = set.right_length = (rc_ctrl.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;
		set.roll = -rc_ctrl.rc.ch[R_X] * 45.0f / 660.0f;

		set.x = (set.v != 0) ? ch->state.x_filter : 0;
	}
	else
	{
		set.v = 0;
		set.yaw = ch->IMU_DATA.toatalyaw;
		set.roll = 0;
		set.left_length = set.right_length = 0.2f;
		set.x = ch->state.x_filter;
	}
}

/// @brief 限幅
void Clamp(float *in, float min, float max)
{
	if (*in < min)
	{
		*in = min;
	}
	else if (*in > max)
	{
		*in = max;
	}
}

// 支持力前馈
float fn_feedforward = 0;

float lqr_coe[12][4] = {
	{-269.302161650351081, 665.997864129000050, -684.856663249451572, -4.200874736219789},
	{0.202747631972482, -390.940145397614401, 683.134886658680216, 35.759371905157778},
	{-16.516861525373891, -0.942134815925538, 3.974845965259497, 0.265827713709572},
	{18.375954258041411, -68.491264611158840, 81.298928448795564, 0.726732173939417},
	{-76.112409385061156, 220.240793342385388, -233.015175620739200, -0.658140709766848},
	{-19.375627199363102, -82.822537693160456, 185.120778564919590, 14.323121345883600},
	{-58.281246706424803, 169.280863067695009, -184.276455227029288, -2.706827504553039},
	{-62.752923129447581, 75.534969412225422, -6.207037353844792, 17.977268348363861},
	{-369.000277243575681, 611.569111135693220, -347.327805218386516, 92.806369486766101},
	{766.796342216858989, -2211.317977874311964, 2298.843592794516098, 78.491909876644897},
	{-24.855149986034782, 44.390123129392293, -31.389305835818501, 6.678577363723454},
	{40.558751559830981, -111.267255137396901, 111.505935959337293, 3.487975843682994}};

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void LQR_Control(Chassis_t *ch)
{
	LQR_K_Calc(lqr_k_l, lqr_coe, leg_l.L0);
	LQR_K_Calc(lqr_k_r, lqr_coe, leg_r.L0);

	xl[0] = (leg_l.theta);
	xl[1] = (leg_l.d_theta);
	xl[2] = (ch->state.x_filter - set.x);
	xl[3] = (ch->state.v_filter - set.v);
	xl[4] = (ch->IMU_DATA.pitch);
	xl[5] = (ch->IMU_DATA.pitchspd);

	xr[0] = (leg_r.theta);
	xr[1] = (leg_r.d_theta);
	xr[2] = (ch->state.x_filter - set.x);
	xr[3] = (ch->state.v_filter - set.v);
	xr[4] = (ch->IMU_DATA.pitch);
	xr[5] = (ch->IMU_DATA.pitchspd);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/// @brief 核心 LQR 计算，公式 u = - K * x。分左右腿
	tlqrl = tplqrl = tlqrr = tplqrr = 0;
	for (int i = 0; i < 6; i++)
	{
		tlqrl += xl[i] * lqr_k_l[0][i];
		tplqrl += xl[i] * lqr_k_l[1][i];
		tlqrr += xr[i] * lqr_k_r[0][i];
		tplqrr += xr[i] * lqr_k_r[1][i];
	}

	/* ================================ 轮 解算 ================================ */
	turn_t = yaw_pid.Kp * (set.yaw - ch->IMU_DATA.toatalyaw) - yaw_pid.Kd * ch->IMU_DATA.yawspd; // 这样计算更稳一点
	set.torque[5] = tlqrl + turn_t;
	set.torque[4] = tlqrr - turn_t;

	/* ================================ 腿 解算 ================================ */

	/// @brief 腿推力 PID
	fn_feedforward = 55.0f * arm_cos_f32(leg_r.theta);

	leg_l.F0 = fn_feedforward +
			   PID_Calc(&leglength_pid_l, set.left_length, leg_l.L0);
	leg_r.F0 = fn_feedforward +
			   PID_Calc(&leglength_pid_r, set.right_length, leg_r.L0);

	leg_l.Tp = tplqrl;
	leg_r.Tp = tplqrr;

	/// @brief 正 VMC
	VMC_calc_2(&leg_l);
	VMC_calc_2(&leg_r);

	/// @brief 发送 buf
	set.torque[0] = leg_r.torque_set[0];
	set.torque[1] = leg_r.torque_set[1];
	set.torque[3] = leg_l.torque_set[0];
	set.torque[2] = leg_l.torque_set[1];

/* ================================ 发送 ================================ */

/// @brief 限幅
#define HIP_TORQUE_MAX 40.0f
#define HUB_TORQUE_MAX 2.5f
	Clamp(&set.torque[0], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[1], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[2], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[3], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[4], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	Clamp(&set.torque[5], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);

	/// @brief 发送
	ch->ak_set[0].torset = -set.torque[0];
	ch->ak_set[1].torset = -set.torque[1];
	ch->ak_set[2].torset = set.torque[2];
	ch->ak_set[3].torset = set.torque[3];
	ch->ak_set[4].torset = -set.torque[4];
	ch->ak_set[5].torset = set.torque[5];
}

/************************
 * @brief 多项式拟合，腿长对应 K 矩阵
 *
 * @param[out] k 输出的 LQR K 矩阵
 * @param coe 多项式拟合系数
 * @param len 腿长
 * @return float
 ************************/
void LQR_K_Calc(float k[2][6], float coe[12][4], float len)
{
	int i, j, n;
	float k_[6][2];

	// 计算 k
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 2; j++)
		{
			n = i * 2 + j;
			k_[i][j] = coe[n][0] * len + coe[n][1] * len * len + coe[n][2] * len * len * len + coe[n][3];
		}
	}

	// 转置
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 2; j++)
		{
			k[j][i] = k_[i][j];
		}
	}
}
