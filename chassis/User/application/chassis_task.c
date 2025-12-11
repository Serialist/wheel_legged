/***********************************************
 * @file chassis_task.c
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

/* ================================================================ include ================================================================ */

#include "chassis_task.h"
#include "update_task.h"
#include "bsp_delay.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "constrain_calc.h"
#include "data_transform.h"
#include "pid.h"
#include "INS_task.h"
#include "kalman.h"
#include <math.h>
#include "bsp_dwt.h"
#include "remote_control.h"
#include "user_lib.h"
#include "vmc.h"
#include "can.h"
#include "debug.h"
#include "filter.h"
#include "motor.h"
#include "gimbal_chassis_comm.h"
#include "misc.h"
#include "fsm.h"

/* ================================================================ macro ================================================================ */

/* ================================================================ typedef ================================================================ */

enum Jump_State
{
	JUMP_STATE_NONE = 0,
	JUMP_STATE_RETRACT_GND,
	JUMP_STATE_TAKEOFF,
	JUMP_STATE_RETRACT_AIR,
	JUMP_STATE_LAND
};

/* ================================================================ variable ================================================================ */

struct Chassis_State chassis;
struct PID_Def chassis_motor_pid[10];

/**
 * @brief VMC腿摆角扭矩pid，用于板凳模型
 * @date 2025-10-16
 */
struct PID_Def pid_tpl, pid_tpr;

struct PID_Def chassis_motor_pid[10];

float T_AK_set_left[2];
float T_AK_set_right[2];

struct Wheel_Leg_Target set;
struct VMC_Leg leg_l, leg_r;

float turn_t; // yaw轴补偿
float leg_tp; // 防劈叉补偿

float tplqrl;
float tplqrr;
float tlqrl;
float tlqrr;

float lqr_k_l[2][6], lqr_k_r[2][6], xl[6], xr[6];

float t_ratiol;
float tp_ratiol;
float t_ratior;
float tp_ratior;

/// @brief 离地检测滤波
struct Filter_Average ground_detection_filter_l, ground_detection_filter_r;

struct PID_Def motor_test_pid;
float motor_test_speed = 0;

enum Jump_State jump_state = JUMP_STATE_NONE;

/* ================================================================ prototype ================================================================ */

static void Chassis_Init(void);

static void Fdb_Update(struct Chassis_State *ch);
static void chassis_torque_sent(struct Chassis_State *ch);

static void Balance_Produce(struct Chassis_State *ch);
void Jump_Phase(struct Chassis_State *ch);

static bool Ground_Detection_L(struct Chassis_State *ch, struct VMC_Leg *leg);
static bool Ground_Detection_R(struct Chassis_State *ch, struct VMC_Leg *leg);
static void Safe_Control(struct Chassis_State *ch);
static void Get_Order(struct Chassis_State *ch);
static void LQR_K_Calc(float k[2][6], float coe[12][4], float len);
static void Motor_Enable(void);

/* ================================================================ function ================================================================*/

/************************
 * @brief 底盘任务
 *
 * @param argument
 ************************/
void chassis_task(void const *argument)
{
	/* ================================ 初始化 ================================ */

	Chassis_Init();

	VMC_Init(&leg_l);
	VMC_Init(&leg_r);

	// 等待整车初始化完成
	while (chassis.robo_status.status == ROBO_STATE_INIT)
	{
		osDelay(1);
	}

	for (;;)
	{
		/* ================================ 更新 ================================ */

		Fdb_Update(&chassis); // 反馈数据

		/* ================================ 控制 ================================ */

		Get_Order(&chassis); // 获得控制指令

		Balance_Produce(&chassis); // 平衡处理

		Safe_Control(&chassis); // 安全控制

		/* ================================ 发送 ================================  */

		chassis_torque_sent(&chassis); // 底盘电机发送

		// Yaw_Send(&yaw_data);

		C2G_Transmit(&yaw_data); // 云底通信

		osDelay(1);
	}
}

//************************基础控制器*********************************/
// 系统初始化
static void Chassis_Init(void)
{
	// 腿部运动PID初始化
	// 腿长PID左
	chassis_motor_pid[0].Kp = 500;
	chassis_motor_pid[0].Ki = 0;
	chassis_motor_pid[0].Kd = 6000;
	chassis_motor_pid[0].max_out = 80;
	chassis_motor_pid[0].max_iout = 0;
	// 右
	chassis_motor_pid[1].Kp = 500;
	chassis_motor_pid[1].Ki = 0;
	chassis_motor_pid[1].Kd = 6000;
	chassis_motor_pid[1].max_out = 80;
	chassis_motor_pid[1].max_iout = 0;

	// 角度PID左
	chassis_motor_pid[2].Kp = 1.65;
	chassis_motor_pid[2].Ki = 0;
	chassis_motor_pid[2].Kd = 11;
	chassis_motor_pid[2].max_out = 2;
	chassis_motor_pid[2].max_iout = 0;
	// 右
	chassis_motor_pid[3].Kp = 1.65;
	chassis_motor_pid[3].Ki = 0;
	chassis_motor_pid[3].Kd = 11;
	chassis_motor_pid[3].max_out = 2;
	chassis_motor_pid[3].max_iout = 0;

	// 方向PID
	chassis_motor_pid[YAW_PID].Kp = 0.1f;
	chassis_motor_pid[YAW_PID].Ki = 0;
	chassis_motor_pid[YAW_PID].Kd = 0.5f;
	chassis_motor_pid[YAW_PID].max_out = 1.5f;
	chassis_motor_pid[YAW_PID].max_iout = 0;

	// 滚转PID
	chassis_motor_pid[ROLL_PID].Kp = 0.8;
	chassis_motor_pid[ROLL_PID].Ki = 0;
	chassis_motor_pid[ROLL_PID].Kd = 0;
	chassis_motor_pid[ROLL_PID].max_out = 30;
	chassis_motor_pid[ROLL_PID].max_iout = 0;

	// 力矩PID
	chassis_motor_pid[TP_PID].Kp = 1.3;
	chassis_motor_pid[TP_PID].Ki = 0;
	chassis_motor_pid[TP_PID].Kd = 3;
	chassis_motor_pid[TP_PID].max_out = 1.5;
	chassis_motor_pid[TP_PID].max_iout = 0;

	chassis_motor_pid[7].Kp = 21.5;
	chassis_motor_pid[7].Ki = 0;
	chassis_motor_pid[7].Kd = 4;
	chassis_motor_pid[7].max_out = 2400;
	chassis_motor_pid[7].max_iout = 2400;

	motor_test_pid.Kp = 5;
	motor_test_pid.Ki = 0;
	motor_test_pid.Kd = 0.1;
	motor_test_pid.max_out = 3000;
	motor_test_pid.max_iout = 0;

	// ================================================================
	/**
	 * @brief VMC腿摆角扭矩pid，用于板凳模型
	 * @date 2025-10-16
	 */
	// left
	pid_tpl.Kp = 80;
	pid_tpl.Ki = 0;
	pid_tpl.Kd = 400;
	pid_tpl.max_out = 15;
	pid_tpl.max_iout = 0;
	// right
	pid_tpr.Kp = 80;
	pid_tpr.Ki = 0;
	pid_tpr.Kd = 400;
	pid_tpr.max_out = 15;
	pid_tpr.max_iout = 0;

	set.left_length = set.right_length = 0.12f;

	tp_ratiol = 1.0f;
	tp_ratior = 1.0f;
	t_ratiol = 1.0f;
	t_ratior = 1.0f;

	Filter_Average_Init(&ground_detection_filter_l, 10);
	Filter_Average_Init(&ground_detection_filter_r, 10);

	Motor_Enable();

	// Reg_Add();
}

/************************
 * @brief 使能电机
 *
 * @note
 * 轮毂的 3508 不用初始化
 * 髋电机需要使能
 ************************/
void Motor_Enable(void)
{
	uint8_t i;

	for (i = 0; i < 4; i++)
	{
		controller_init(i + 1);
		osDelay(1);
	}
}

/************************
 * @brief 状态更新
 *
 * @param ch
 ************************/
static void Fdb_Update(struct Chassis_State *ch)
{
	ch->time_last = ch->time_now;
	// 获取电机数据
	GetAKMotor1Fdb(&ch->AK_fdb[0], &ch->ak_fdb_ctrl[0]);
	GetAKMotor2Fdb(&ch->AK_fdb[1], &ch->ak_fdb_ctrl[1]);
	GetAKMotor3Fdb(&ch->AK_fdb[2], &ch->ak_fdb_ctrl[2]);
	GetAKMotor4Fdb(&ch->AK_fdb[3], &ch->ak_fdb_ctrl[3]);
	GetAKMotor5Fdb(&ch->AK_fdb[4], &ch->ak_fdb_ctrl[4]);
	GetAKMotor6Fdb(&ch->AK_fdb[5], &ch->ak_fdb_ctrl[5]);

	// 获取遥控器数据
	ch->time_now.rc = GetRCData(&(ch->rc_data));

	ch->robo_status.flag.above_left = Ground_Detection_L(ch, &leg_l);
	ch->robo_status.flag.above_right = Ground_Detection_R(ch, &leg_r);

	ch->robo_status.flag.above = (ch->robo_status.flag.above_left == true) && (ch->robo_status.flag.above_right == true);
}

void chassis_sys_calc(struct Chassis_State *ch)
{
	leg_l.phi1 = pi / 2.0f - ch->ak_fdb_ctrl[3].motor_ctrlpos;
	leg_l.phi4 = pi / 2.0f - ch->ak_fdb_ctrl[2].motor_ctrlpos + 0.229f;
	leg_r.phi1 = pi / 2.0f - ch->ak_fdb_ctrl[0].motor_ctrlpos;
	leg_r.phi4 = pi / 2.0f - ch->ak_fdb_ctrl[1].motor_ctrlpos - 0.9f;

	VMC_calc_1(&leg_l, &chassis, 3.0f / 1000.0f);
	VMC_calc_1(&leg_r, &chassis, 3.0f / 1000.0f);
}

void Phase_Update(struct Chassis_State *ch)
{
	ch->state.phi = ch->IMU_DATA.pitch;
	ch->state.dPhi = ch->IMU_DATA.vpitch;

	ch->state.xr = ch->state.x_filter;
	ch->state.vr = ch->state.v_filter;

	ch->state.xl = ch->state.x_filter;
	ch->state.vl = ch->state.v_filter;

	ch->state.thetal = leg_l.theta;
	ch->state.thetar = leg_r.theta;

	ch->state.alphal = leg_l.alpha;
	ch->state.alphar = leg_r.alpha;

	ch->state.angle_err = leg_l.phi0 - leg_r.phi0;

	ch->state.dThetal = leg_l.d_theta;
	ch->state.dThetar = leg_r.d_theta;

	ch->state.d_alphal = leg_l.d_alpha;
	ch->state.d_alphar = leg_r.d_alpha;
}

// 扭矩发送

/************************
 * @brief oria
 *
 * @param ch
 ************************/
void chassis_torque_sent(struct Chassis_State *ch)
{

	if (ch->robo_status.status != ROBO_STATE_RUN)
	{
		for (int i = 0; i < 6; i++)
			ch->ak_set[i].torset = 0;
	}

	// MIT模式下发送
	pack_cmd(1, 0, 0, 0, 0, ch->ak_set[0].torset);
	pack_cmd(4, 0, 0, 0, 0, ch->ak_set[3].torset);
	delay_ms(1);
	pack_cmd(2, 0, 0, 0, 0, ch->ak_set[1].torset);
	pack_cmd(3, 0, 0, 0, 0, ch->ak_set[2].torset);
	delay_ms(1);

	/// @brief 用 3508
	DJI_Motor_Transmit(&hcan1, M3508_TX_ID_2,
					   0,
					   HEXROLL_TORQUE_TO_CURRENT(ch->ak_set[4].torset),
					   //    PID_Update(&motor_test_pid, motor_test_speed, m3508[1].speed),
					   HEXROLL_TORQUE_TO_CURRENT(ch->ak_set[5].torset),
					   0);

	// 伺服模式下电流发送

	//	comm_can_set_current(AK_ID_11,ch->current_set[4]);
	//	comm_can_set_current(AK_ID_21,ch->current_set[5]);
	//	vTaskDelay(10);
	//	comm_can_set_current(AK_ID_31,ch->current_set[0]);
	//	comm_can_set_current(AK_ID_61,ch->current_set[1]);
	//	vTaskDelay(10);
	//	comm_can_set_current(AK_ID_41,ch->current_set[2]);
	//	comm_can_set_current(AK_ID_51,ch->current_set[3]);
}

//************************运动控制器*********************************/
// 遥控器指令
void Get_Order(struct Chassis_State *ch)
{
	ch->robo_status.last_behavior = ch->robo_status.behavior;

	// 无力模式
	if (ch->rc_data.rc.s[S_R] == UP)
	{
		ch->robo_status.behavior = ROBO_BX_STOP;
	}
	// 一阶倒立摆
	else if (ch->rc_data.rc.s[S_R] == MID) // 停止模式
	{
		set.speed_cmd = ch->rc_data.rc.ch[L_Y] * RC_RATIO;												 // 目标速度
		set.yaw += -ch->rc_data.rc.ch[L_X] * RC_RATIO;													 // 目标 yaw
		set.left_length = set.right_length = VAULE_MAP(ch->rc_data.rc.ch[R_Y], -660, 660, 0.10f, 0.35f); // 目标腿长

		// 行进时不关注位置
		if (set.speed_cmd != 0)
		{
			set.position_set = ch->state.x_filter;
		}

		set.roll = ch->rc_data.rc.ch[R_X] * PI / 2 / 660 / 4;

		my_debug.no_tp_flag = true;

		ch->robo_status.behavior = ROBO_BX_NORMAL;
	}
	// 二阶倒立摆
	else if (ch->rc_data.rc.s[S_R] == DOWN)
	{
		// 正常运行
		if (ch->rc_data.rc.ch[R_Y] != -660)
		{
			my_debug.no_tp_flag = false;

			ch->robo_status.behavior = ROBO_BX_NORMAL;

			set.speed_cmd = ch->rc_data.rc.ch[L_Y] * RC_RATIO;												 // 目标速度
			set.yaw += -ch->rc_data.rc.ch[L_X] * YAW_RATIO;													 // 目标 yaw
			set.left_length = set.right_length = VAULE_MAP(ch->rc_data.rc.ch[R_Y], -660, 660, 0.10f, 0.30f); // 目标腿长

			// 行进时不关注位置
			if (set.speed_cmd != 0)
			{
				set.position_set = ch->state.x_filter;
			}

			set.roll = ch->rc_data.rc.ch[R_X] * PI / 2 / 660 / 4;
		}
		// 跳跃
		else
		{
			ch->robo_status.behavior = ROBO_BX_JUMP;
		}
	}

	/// @todo 改写成行为树
}

// 保险
void Safe_Control(struct Chassis_State *ch)
{
	// 如果停止，输出 0
	if (ch->robo_status.behavior == ROBO_BX_STOP || ch->robo_status.behavior == ROBO_BX_OFF)
	{
		ch->ak_set[0].torset = 0.0f;
		ch->ak_set[1].torset = 0.0f;
		ch->ak_set[2].torset = 0.0f;
		ch->ak_set[3].torset = 0.0f;
		ch->ak_set[4].torset = 0.0f;
		ch->ak_set[5].torset = 0.0f;
	}

	// 如果电机离线，尝试重新使能电机
	if (ch->robo_status.status == ROBO_STATE_EMERGENCY && MOTOR_IS_OFFLINE(&motor_status))
	{
		Motor_Enable();
	}
}

#define OFFGROUND_DETECTION_ACCEL_RATIO 0.6f

#define GROUND_DETECTION_THRESHOLD 15.0f // threshold 阈值
/************************
 * @brief 触地检测器 right
 *
 * @param ch
 * @return uint8_t 离地 1 触地 0
 ************************/
bool Ground_Detection_R(struct Chassis_State *ch, struct VMC_Leg *leg)
{
	ch->state.FNr = leg->F0 * arm_cos_f32(leg->theta) +
					leg->Tp * arm_sin_f32(leg->theta) / leg->L0 +
					OFFGROUND_DETECTION_ACCEL_RATIO * (ch->IMU_DATA.az +
													   (-leg->dd_L0 * arm_cos_f32(leg->theta)) +
													   2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
													   leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
													   leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

	ch->state.FNr = Filter_Average_Update(&ground_detection_filter_r, ch->state.FNr);

	return (ch->state.FNr < GROUND_DETECTION_THRESHOLD) ? true : false;
}

/************************
 * @brief 触地检测器 left
 *
 * @param ch
 * @return uint8_t
 ************************/
bool Ground_Detection_L(struct Chassis_State *ch, struct VMC_Leg *leg)
{
	my_debug.ground_det.f0l = leg->F0 * arm_cos_f32(leg->theta);
	my_debug.ground_det.tpl = leg->Tp * arm_sin_f32(leg->theta) / leg->L0;

	ch->state.FNl = my_debug.ground_det.f0l +
					my_debug.ground_det.tpl +
					OFFGROUND_DETECTION_ACCEL_RATIO * (ch->IMU_DATA.az +
													   (-leg->dd_L0 * arm_cos_f32(leg->theta)) +
													   2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
													   leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
													   leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

	ch->state.FNl = Filter_Average_Update(&ground_detection_filter_l, ch->state.FNl);

	return (ch->state.FNl < GROUND_DETECTION_THRESHOLD) ? true : false;
}

float xl_target[6] = {0},
	  xr_target[6] = {0};

float lqr_r_[12] = {0};
float lqr_l_[12] = {0};

float kx[12] = {1, 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1};

float KK[6] = {1, 1, 1, 1, 1, 1};

// 推力前馈
float fn_forward_r = 0;
float fn_forward_l = 0;

/// @date 2025-11-08 22:23
/// @date 2025-11-09 20:12 20:24
/// @date 2025-11-14 15:07 17:27
/// @date 2025-11-15 19:04 23:51
/// @date 2025-11-16 15:56 18:57
/// @date 2025-11-18 22:12 22:56
/// @date 2025-11-19 19:24
/// @date 2025-11-21 23:04 23:39
/// @date 2025-11-22 18:05 19:45
/// @date 2025-11-26 19:22
/// @date 2025-11-27 21:42 23:01
/// @date 2025-11-28 11:55 20:12
/// @date 2025-11-29 23:10
/// @date 2025-12-06 16:30 22:42
/// @date 2025-12-07 17:26 18:54
float lqr_coe[12][4] = {
	{-72.020750067070409, -133.442052393281386, 314.103896441336076, -1.291623246816448},
	{176.276906683049987, -383.972935317621193, 270.419166730327674, -5.585725556191087},
	{-6.371826973681856, -52.843832380562873, 71.648714252828455, -0.141254334901730},
	{14.487886509303429, -13.249594795107299, -19.420107363462410, -0.573766034435564},
	{-13.975495331470929, -49.462498281463347, 162.367335689228895, -4.224253743416839},
	{149.863609149629298, -625.023094578263340, 869.764932041754264, -7.355428842314836},
	{-11.999381932340690, -72.685447361650162, 194.075631668482714, -4.739919999837185},
	{144.595997681058606, -594.618278708500156, 820.728866112368792, -7.032377829242853},
	{-157.511582645597599, 277.810196525614117, -248.259570796877000, 62.973022759557168},
	{323.246939837904222, -766.447127212351234, 771.406338302571157, 66.953913181349819},
	{-7.744776446875450, 38.881103715924972, -70.124285703505819, 4.927318262858634},
	{-20.639530367016270, 114.807776885591807, -170.729118893160290, 6.033005512437716}};

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void Balance_Produce(struct Chassis_State *ch)
{
	/* ================================ 补充控制 ================================ */

	if (my_debug.no_yaw_flag)
	{
		set.yaw = chassis.IMU_DATA.total_yaw;
		turn_t = 0;
	}
	else
	{
		turn_t = chassis_motor_pid[YAW_PID].Kp * (set.yaw - chassis.IMU_DATA.total_yaw) - chassis_motor_pid[YAW_PID].Kd * ch->IMU_DATA.vyaw; // 这样计算更稳一点
	}
	// leg_tp = PID_Update(&chassis_motor_pid[TP_PID], 0.0f, ch->state.angle_err);					// 防劈叉pid计算
	set.roll_set_now = PID_Update(&chassis_motor_pid[ROLL_PID], set.roll, ch->IMU_DATA.roll); // roll 补偿

	/* ================================ LQR 控制 ================================ */

	LQR_K_Calc(lqr_k_l, lqr_coe, leg_l.L0);
	LQR_K_Calc(lqr_k_r, lqr_coe, leg_r.L0);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	xl[0] = (ch->state.thetal);
	xl[1] = (ch->state.dThetal);
	xl[2] = (ch->state.xl - set.position_set);
	xl[3] = (ch->state.vl - set.speed_cmd);
	xl[4] = (ch->state.phi);
	xl[5] = (ch->state.dPhi);

	xr[0] = (ch->state.thetar);
	xr[1] = (ch->state.dThetar);
	xr[2] = (ch->state.xr - set.position_set);
	xr[3] = (ch->state.vr - set.speed_cmd);
	xr[4] = (ch->state.phi);
	xr[5] = (ch->state.dPhi);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/// @brief 核心 LQR 计算，公式 u = - K * x。分左右腿

	// left

	lqr_l_[0] = kx[0] * xl[0] * lqr_k_l[0][0];
	lqr_l_[1] = kx[1] * xl[1] * lqr_k_l[0][1];
	lqr_l_[2] = kx[2] * xl[2] * lqr_k_l[0][2];
	lqr_l_[3] = kx[3] * xl[3] * lqr_k_l[0][3];
	lqr_l_[4] = kx[4] * xl[4] * lqr_k_l[0][4];
	lqr_l_[5] = kx[5] * xl[5] * lqr_k_l[0][5];

	tlqrl = lqr_l_[0] + lqr_l_[1] + lqr_l_[2] + lqr_l_[3] + lqr_l_[4] + lqr_l_[5];

	lqr_l_[6] = kx[6] * xl[0] * lqr_k_l[1][0];
	lqr_l_[7] = kx[7] * xl[1] * lqr_k_l[1][1];
	lqr_l_[8] = kx[8] * xl[2] * lqr_k_l[1][2];
	lqr_l_[9] = kx[9] * xl[3] * lqr_k_l[1][3];
	lqr_l_[10] = kx[10] * xl[4] * lqr_k_l[1][4];
	lqr_l_[11] = kx[11] * xl[5] * lqr_k_l[1][5];

	tplqrl = lqr_l_[6] + lqr_l_[7] + lqr_l_[8] + lqr_l_[9] + lqr_l_[10] + lqr_l_[11];

	// right

	lqr_r_[0] = kx[0] * xr[0] * lqr_k_r[0][0];
	lqr_r_[1] = kx[1] * xr[1] * lqr_k_r[0][1];
	lqr_r_[2] = kx[2] * xr[2] * lqr_k_r[0][2];
	lqr_r_[3] = kx[3] * xr[3] * lqr_k_r[0][3];
	lqr_r_[4] = kx[4] * xr[4] * lqr_k_r[0][4];
	lqr_r_[5] = kx[5] * xr[5] * lqr_k_r[0][5];

	tlqrr = lqr_r_[0] + lqr_r_[1] + lqr_r_[2] + lqr_r_[3] + lqr_r_[4] + lqr_r_[5];

	lqr_r_[6] = kx[6] * xr[0] * lqr_k_r[1][0];
	lqr_r_[7] = kx[7] * xr[1] * lqr_k_r[1][1];
	lqr_r_[8] = kx[8] * xr[2] * lqr_k_r[1][2];
	lqr_r_[9] = kx[9] * xr[3] * lqr_k_r[1][3];
	lqr_r_[10] = kx[10] * xr[4] * lqr_k_r[1][4];
	lqr_r_[11] = kx[11] * xr[5] * lqr_k_r[1][5];

	tplqrr = lqr_r_[6] + lqr_r_[7] + lqr_r_[8] + lqr_r_[9] + lqr_r_[10] + lqr_r_[11];

	// if (my_debug.torque_flag)
	// {
	// 	tplqrl = my_debug.tpl;
	// 	tplqrr = my_debug.tpr;
	// 	tlqrl = my_debug.tl;
	// 	tlqrr = my_debug.tr;
	// }
	if (my_debug.no_above_det_flag == false && ch->robo_status.flag.above == true)
	{
		// 不控制x和yaw
		set.position_set = chassis.state.x_filter = 0;
		set.yaw = chassis.IMU_DATA.total_yaw;

		// lqr 仅控制腿 theta
		tplqrl = lqr_l_[6] + lqr_l_[7];
		tplqrr = lqr_r_[6] + lqr_r_[7];
	}

	/// @brief 限幅
#define TPLQR_MAX 15.0f
#define TLQR_MAX 2.5f
	tplqrl = LIMIT(tplqrl, -TPLQR_MAX, TPLQR_MAX);
	tplqrr = LIMIT(tplqrr, -TPLQR_MAX, TPLQR_MAX);
	tlqrl = LIMIT(tlqrl, -TLQR_MAX, TLQR_MAX);
	tlqrr = LIMIT(tlqrr, -TLQR_MAX, TLQR_MAX);

	/* ================================ 轮 解算 ================================ */

	/// @brief 轮毂力矩计算
	if (my_debug.no_t_flag)
	{
		set.set_cal_real[5] = 0;
		set.set_cal_real[4] = 0;
	}
	else
	{
		set.set_cal_real[5] = tlqrl + turn_t;
		set.set_cal_real[4] = tlqrr - turn_t;
	}

	/* ================================ 腿推力 解算 ================================ */

	// 重力前馈
	if (my_debug.no_g_fn_flag == true)
	{
		fn_forward_l = 0;
		fn_forward_r = 0;
	}
	// 离地
	else if (ch->robo_status.flag.above == true)
	{
		// 提腿前馈
		fn_forward_l = -15.0f;
		fn_forward_r = -15.0f;
	}
	else
	{
		fn_forward_l = 55.0f * arm_cos_f32(leg_l.theta);
		fn_forward_r = 55.0f * arm_cos_f32(leg_r.theta);
	}

	// 跳跃状态机
	Jump_Phase(ch);

	// 推力 pid
	// 不用 yaw
	if (jump_state == JUMP_STATE_NONE || ch->robo_status.flag.above == false)
	{
		set.left_length += set.roll_set_now;
		set.right_length -= set.roll_set_now;
	}
	// 在空中，预留落地缓冲腿长
	else if (jump_state == JUMP_STATE_NONE && ch->robo_status.flag.above == true)
	{
		set.left_length = set.left_length = 0.15f;
	}

	// 推力计算
	leg_l.F0 = fn_forward_l +
			   PID_Update(&chassis_motor_pid[0], set.left_length, leg_l.L0);
	leg_r.F0 = fn_forward_r +
			   PID_Update(&chassis_motor_pid[1], set.right_length, leg_r.L0);

	/* ================================ 腿扭矩 解算 ================================ */

	if (my_debug.no_tp_flag)
	{
		tplqrl = PID_Update(&pid_tpl, PI / 2.0f, leg_l.phi0);
		tplqrr = PID_Update(&pid_tpr, PI / 2.0f, leg_r.phi0);
	}

	leg_l.Tp = tplqrl * tp_ratiol;
	leg_r.Tp = tplqrr * tp_ratior;

	// 正 VMC
	VMC_calc_2(&leg_l);
	VMC_calc_2(&leg_r);

	// 发送 buf
	set.set_cal_real[0] = leg_r.torque_set[0];
	set.set_cal_real[1] = leg_r.torque_set[1];
	set.set_cal_real[3] = leg_l.torque_set[0];
	set.set_cal_real[2] = leg_l.torque_set[1];

	// 软件限位
	// 当超过一定限定角度时，扭矩设为反向

	// if (leg_l.phi1 < PI / 2)
	// {
	// 	set.set_cal_real[3] = -10.0f;
	// }
	// if (leg_l.phi4 > PI / 2)
	// {
	// 	set.set_cal_real[2] = 10.0f;
	// }
	// if (leg_r.phi1 < PI / 2)
	// {
	// 	set.set_cal_real[0] = -10.0f;
	// }
	// if (leg_r.phi4 > PI / 2)
	// {
	// 	set.set_cal_real[1] = 10.0f;
	// }

	/* ================================ 发送 ================================ */

	/// @brief 限幅
#define HIP_TORQUE_MAX 15.0f
#define HUB_TORQUE_MAX 2.5f
	set.set_cal_real[0] = LIMIT(set.set_cal_real[0], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	set.set_cal_real[1] = LIMIT(set.set_cal_real[1], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	set.set_cal_real[2] = LIMIT(set.set_cal_real[2], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	set.set_cal_real[3] = LIMIT(set.set_cal_real[3], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	set.set_cal_real[4] = LIMIT(set.set_cal_real[4], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	set.set_cal_real[5] = LIMIT(set.set_cal_real[5], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);

	/// @brief 发送
	ch->ak_set[0].torset = -set.set_cal_real[0];
	ch->ak_set[1].torset = -set.set_cal_real[1];
	ch->ak_set[2].torset = set.set_cal_real[2];
	ch->ak_set[3].torset = set.set_cal_real[3];
	ch->ak_set[4].torset = -set.set_cal_real[4];
	ch->ak_set[5].torset = set.set_cal_real[5];
}

uint32_t jump_time = 0;

#ifndef CLEAR
#define CLEAR(var) ((var) = 0)
#endif

#define JUMP_RETRACT_GND_LEG_LENGTH 0.12f
#define JUMP_TAKEOFF_LEG_LENGTH 0.35f
#define JUMP_LAND_LEG_LENGTH 0.15f

/************************
 * @brief 跳跃过程
 *
 * @param ch
 ************************/
void Jump_Phase(struct Chassis_State *ch)
{
	// 不跳，检测，触发跳
	if (ch->robo_status.last_behavior != ROBO_BX_JUMP && ch->robo_status.behavior == ROBO_BX_JUMP && jump_state == JUMP_STATE_NONE)
	{
		jump_state = JUMP_STATE_RETRACT_GND;

		CLEAR(jump_time);
	}
	//  地面收腿
	else if (jump_state == JUMP_STATE_RETRACT_GND)
	{
		set.left_length = set.right_length = JUMP_RETRACT_GND_LEG_LENGTH;

		if (jump_time >= 50)
		{
			my_debug.no_yaw_flag = true;
			jump_state = JUMP_STATE_TAKEOFF; // 压缩完毕进入上升加速阶段
			CLEAR(jump_time);
		}
	}
	// 起跳
	else if (jump_state == JUMP_STATE_TAKEOFF)
	{
		set.left_length = set.right_length = JUMP_TAKEOFF_LEG_LENGTH;

		// 方式空中不离地
		// lqr 仅控制腿 theta
		tplqrl = lqr_l_[6] + lqr_l_[7];
		tplqrr = lqr_r_[6] + lqr_r_[7];

		fn_forward_l = fn_forward_r = 200.0f;

		if (leg_l.L0 > JUMP_TAKEOFF_LEG_LENGTH)
		{
			jump_state = JUMP_STATE_RETRACT_AIR; // 上升完毕进入缩腿阶段
			CLEAR(jump_time);
		}
	}
	// 空中收腿
	else if (jump_state == JUMP_STATE_RETRACT_AIR)
	{
		my_debug.no_t_flag = true;
		fn_forward_l = fn_forward_r = -200.0f;

		if (leg_l.L0 < JUMP_LAND_LEG_LENGTH)
		{
			jump_state = JUMP_STATE_LAND;
			CLEAR(jump_time);
		}
	}
	// 等待落地
	else if (jump_state == JUMP_STATE_LAND)
	{
		fn_forward_l = fn_forward_r = -15.0f;
		set.left_length = set.right_length = JUMP_LAND_LEG_LENGTH;

		if (chassis.robo_status.flag.above == false)
		{
			my_debug.no_t_flag = false;
			my_debug.no_yaw_flag = false;

			ch->robo_status.behavior = ROBO_BX_NORMAL;

			jump_state = JUMP_STATE_NONE;
			CLEAR(jump_time);
		}
	}

	jump_time++;
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
