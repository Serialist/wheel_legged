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

/* ================================================================ macro ================================================================ */

/* ================================================================ variable ================================================================ */

Chassis_t chassis;
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
float total_yaw;

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

/* ================================================================ prototype ================================================================ */

static void Chassis_Init(void);

static void Fdb_Update(Chassis_t *ch);
static void chassis_torque_sent(Chassis_t *ch);

static void standphase(Chassis_t *ch);
static void balancephase(Chassis_t *ch);
// static void balancephase_one_rod(Chassis_t *ch);
static void Jump_Phase(Chassis_t *ch);

static uint8_t Ground_Detection_L(Chassis_t *ch, struct VMC_Leg *leg);
static uint8_t Ground_Detection_R(Chassis_t *ch, struct VMC_Leg *leg);
static void Safe_Control(Chassis_t *ch);
static void Get_Order(Chassis_t *ch);
static void LQR_K_Calc(float k[2][6], float coe[12][4], float len);
static void Motor_Init(void);

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

		/* ================================ 状态更新 ================================ */

		Fdb_Update(&chassis); // 反馈数据
		Get_Order(&chassis);  // 获得控制指令

		/* ================================ 状态控制 ================================ */

		if (chassis.robo_status.status == ROBO_STATE_STAND)
		{
			standphase(&chassis); // 起立过程
		}

		balancephase(&chassis); // 平衡处理

		Safe_Control(&chassis); // 安全控制

		/* ================================ 发送数据 ================================  */

		chassis_torque_sent(&chassis); // 底盘电机发送
		C2G_Transmit(&yaw_data);	   // 云底通信

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

	Motor_Init();

	// Reg_Add();
}

/************************
 * @brief 初始化电机控制
 *
 ************************/
void Motor_Init(void)
{
	for (int a = 0; a < 4; a++)
	{
		controller_init(a + 1);
		vTaskDelay(2);
	}
	for (int bb = 4; bb < 6; bb++)
	{
		controller_init(bb + 1);
		controller_setorigin(bb + 1);
		vTaskDelay(2);
	}
}

/************************
 * @brief 状态更新
 *
 * @param ch
 ************************/
static void Fdb_Update(Chassis_t *ch)
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

void chassis_sys_calc(Chassis_t *ch)
{
	leg_l.phi1 = pi / 2.0f - ch->ak_fdb_ctrl[3].motor_ctrlpos;
	leg_l.phi4 = pi / 2.0f - ch->ak_fdb_ctrl[2].motor_ctrlpos;
	leg_r.phi1 = pi / 2.0f - ch->ak_fdb_ctrl[0].motor_ctrlpos;
	leg_r.phi4 = pi / 2.0f - ch->ak_fdb_ctrl[1].motor_ctrlpos;

	VMC_calc_1(&leg_l, &chassis, 3.0f / 1000.0f);
	VMC_calc_1(&leg_r, &chassis, 3.0f / 1000.0f);
}

void Phase_Update(Chassis_t *ch)
{
	ch->st.phi = ch->IMU_DATA.pitch;
	ch->st.dPhi = ch->IMU_DATA.pitchspd;

	ch->st.xr = ch->st.x_filter - set.position_set;
	ch->st.vr = ch->st.v_filter - set.speed_cmd;

	ch->st.xl = ch->st.x_filter - set.position_set;
	ch->st.vl = ch->st.v_filter - set.speed_cmd;

	ch->st.thetal = leg_l.theta;
	ch->st.thetar = leg_r.theta;

	ch->st.alphal = leg_l.alpha;
	ch->st.alphar = leg_r.alpha;

	ch->st.angle_err = leg_l.phi0 - leg_r.phi0;

	ch->st.dThetal = leg_l.d_theta;
	ch->st.dThetar = leg_r.d_theta;

	ch->st.d_alphal = leg_l.d_alpha;
	ch->st.d_alphar = leg_r.d_alpha;

	total_yaw = ch->IMU_DATA.toatalyaw;

	if ((ch->IMU_DATA.pitch < (Pi / 6.0f)) && (ch->IMU_DATA.pitch > (-Pi / 6.0f)))
	{ // 根据pitch角度判断倒地自起是否完成
		ch->recover_flag = 0;
	}
}

// 扭矩发送

/************************
 * @brief oria
 *
 * @param ch
 ************************/
void chassis_torque_sent(Chassis_t *ch)
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
void Get_Order(Chassis_t *ch)
{
	ch->robo_status.last_behavior = ch->robo_status.behavior;

	if (ch->rc_data.rc.s[S_R] == UP)
	{
		ch->robo_status.behavior = ROBO_BX_STOP;
	}
	else if (ch->rc_data.rc.s[S_R] == MID) // 停止模式
	{
		ch->robo_status.behavior = ROBO_BX_STOP;
	}
	else if (ch->rc_data.rc.s[S_R] == DOWN) // 使能模式//第一次拨动时起立
	{
		if (ch->rc_data.rc.ch[R_Y] != -660)
		{
			ch->robo_status.behavior = ROBO_BX_NORMAL;

			set.speed_cmd = ch->rc_data.rc.ch[L_Y] * REMOTE_CHANNLE_TO_CHASSIS_SPEED;								 // 目标速度
			set.yaw += -ch->rc_data.rc.ch[L_X] * REMOTE_CHANNLE_TO_CHASSIS_SPEED;									 // 目标 yaw
			set.left_length = set.right_length = LIMIT((ch->rc_data.rc.ch[R_Y] * 0.01f) / 66 + 0.15f, 0.10f, 0.35f); // 目标腿长

			// 行进时不关注位置
			if (set.speed_cmd != 0)
			{
				ch->st.x_filter = 0;
				set.position_set = 0;
			}
			else
			{
				set.position_set = 0;
			}

			set.roll = ch->rc_data.rc.ch[R_X] * PI / 2 / 660 / 4;
		}
		// 右摇杆在最底下
		else // 跳跃
		{
			ch->robo_status.behavior = ROBO_BX_JUMP; // 开启跳跃 ------------------------------------------------------------------------
		}
	}

	/// @todo 改写成行为树
}

// 保险
void Safe_Control(Chassis_t *ch)
{
	if (ch->robo_status.behavior == ROBO_BX_STOP || ch->robo_status.behavior == ROBO_BX_OFF)
	{
		ch->ak_set[0].torset = 0.0f;
		ch->ak_set[1].torset = 0.0f;
		ch->ak_set[2].torset = 0.0f;
		ch->ak_set[3].torset = 0.0f;
		ch->ak_set[4].torset = 0.0f;
		ch->ak_set[5].torset = 0.0f;
	}

	if (ch->robo_status.status == ROBO_STATE_EMERGENCY)
	{
		if (MOTOR_IS_OFFLINE(&motor_status))
		{
			Motor_Init();
		}
	}
}

float test__ = 0.6f;
float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;

#define GROUND_DETECTION_THRESHOLD 15.0f // threshold 阈值
/************************
 * @brief 触地检测器 right
 *
 * @param ch
 * @return uint8_t 离地 1 触地 0
 ************************/
uint8_t Ground_Detection_R(Chassis_t *ch, struct VMC_Leg *leg)
{
	my_debug.ground_det.f0r = leg->F0 * arm_cos_f32(leg->theta);
	my_debug.ground_det.tpr = leg->Tp * arm_sin_f32(leg->theta) / leg->L0;
	t1 = ch->IMU_DATA.az;
	t2 = -leg->dd_L0 * arm_cos_f32(leg->theta);
	t3 = 2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta);
	t4 = leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta);
	t5 = leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta);

	ch->st.FNr = my_debug.ground_det.f0r +
				 my_debug.ground_det.tpr +
				 test__ * (t1 + t2 + t3 + t4 + t5);

	ch->st.FNr = Filter_Average_Update(&ground_detection_filter_r, ch->st.FNr);

	return (ch->st.FNr < GROUND_DETECTION_THRESHOLD) ? 1 : 0;
}

/************************
 * @brief 触地检测器 left
 *
 * @param ch
 * @return uint8_t
 ************************/
uint8_t Ground_Detection_L(Chassis_t *ch, struct VMC_Leg *leg)
{
	my_debug.ground_det.f0l = leg->F0 * arm_cos_f32(leg->theta);
	my_debug.ground_det.tpl = leg->Tp * arm_sin_f32(leg->theta) / leg->L0;

	ch->st.FNl = my_debug.ground_det.f0l +
				 my_debug.ground_det.tpl +
				 test__ * (ch->IMU_DATA.az -
						   leg->dd_L0 * arm_cos_f32(leg->theta) +
						   2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
						   leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
						   leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

	ch->st.FNl = Filter_Average_Update(&ground_detection_filter_l, ch->st.FNl);

	return (ch->st.FNl < GROUND_DETECTION_THRESHOLD) ? 1 : 0;
}

// 轮腿运动状态实际控制函数群
// 起立前过程
void standphase(Chassis_t *ch)
{
}

/// @brief 定义模式宏
// #define ONE_ROD
// #define NO_MOVE
// #define NO_FN_FORWORD

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
/// @date 2025-12-06 16:30 19:11
/// @date 2025-12-06 21:29
/// @date 2025-12-06 22:05
/// @date 2025-12-06 22:42
float lqr_coe[12][4] = {
	{-231.227889580588311, 496.052364399720318, -455.571668472231579, 2.235341858052133},
	{357.996901499281421, -1592.184420514264957, 2253.054824659212954, -0.581959965234939},
	{-12.702365836943169, -14.729889797149809, 28.739422469544760, -0.257575788263792},
	{20.112293197641542, -63.243564959912852, 50.508720757417720, -1.404318419529429},
	{12.687996287527939, -163.914838645566306, 338.098195378229775, -10.094268887983841},
	{269.991032729346102, -1121.021491261677966, 1576.994028918450113, -20.924321587473020},
	{47.345270685277868, -260.367539778938692, 445.895043070423185, -13.983102484486301},
	{319.985169271764676, -1377.507002019784977, 2015.714369992829006, -24.795520003208821},
	{-206.365336666372599, 231.590465146013713, -10.250115776995870, 68.981224479666409},
	{410.085750976084228, -836.514767363972851, 548.789259352081672, 68.400906049807176},
	{-23.732802233380880, 73.931652380853620, -103.281845952447199, 7.059871192024643},
	{-42.895442286669500, 241.462591914458898, -393.569914082211710, 9.207437351524851}};

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void balancephase(Chassis_t *ch)
{
	/* ================================ 补充控制 ================================ */

	if (my_debug.no_yaw_flag)
	{
		turn_t = 0;
	}
	else
	{
		turn_t = chassis_motor_pid[YAW_PID].Kp * (set.yaw - total_yaw) - chassis_motor_pid[YAW_PID].Kd * ch->IMU_DATA.yawspd; // 这样计算更稳一点
	}
	// leg_tp = PID_Update(&chassis_motor_pid[TP_PID], 0.0f, ch->st.angle_err);					// 防劈叉pid计算
	set.roll_set_now = PID_Update(&chassis_motor_pid[ROLL_PID], set.roll, ch->IMU_DATA.roll); // roll 补偿

	/* ================================ LQR 控制 ================================ */

	LQR_K_Calc(lqr_k_l, lqr_coe, leg_l.L0);
	LQR_K_Calc(lqr_k_r, lqr_coe, leg_r.L0);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	xl[0] = (ch->st.thetal);
	xl[1] = (ch->st.dThetal);
	xl[2] = (ch->st.xl - set.position_set);
	xl[3] = (ch->st.vl - set.speed_cmd);
	xl[4] = (ch->st.phi);
	xl[5] = (ch->st.dPhi);

	xr[0] = (ch->st.thetar);
	xr[1] = (ch->st.dThetar);
	xr[2] = (ch->st.xr - set.position_set);
	xr[3] = (ch->st.vr - set.speed_cmd);
	xr[4] = (ch->st.phi);
	xr[5] = (ch->st.dPhi);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/// @brief 核心 LQR 计算，公式 u = - K * x。分左右腿

	if (my_debug.no_above_det_flag != true)
	{
		uint8_t i = 0;
		if (ch->robo_status.flag.above == true)
		{
			for (i = 0; i < 12; i++)
			{
				if (i == 6 || i == 7)
					kx[i] = KK[i % 6];
				else
					kx[i] = 0;
			}
			my_debug.no_yaw_flag = true;
		}
		else
		{
			for (i = 0; i < 12; i++)
			{
				kx[i] = KK[i % 6];
			}
			my_debug.no_yaw_flag = false;
		}
	}

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

	if (my_debug.torque_flag)
	{
		tplqrl = my_debug.tpl;
		tplqrr = my_debug.tpr;
		tlqrl = my_debug.tl;
		tlqrr = my_debug.tr;
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

	/* ================================ 腿 解算 ================================ */

	/// @brief 腿推力 PID
	// 重力前馈
	if (my_debug.no_g_fn_flag == false)
	{
		fn_forward_l = 55.0f * arm_cos_f32(leg_l.theta);
		fn_forward_r = 55.0f * arm_cos_f32(leg_r.theta);

		// 离地
		if (ch->robo_status.flag.above == true)
		{
			// 仅控制腿 theta
			tplqrl = lqr_l_[6] + lqr_l_[7];
			tplqrr = lqr_r_[6] + lqr_r_[7];

			// 提腿前馈
			fn_forward_l = -10.0f;
			fn_forward_r = -10.0f;
		}
	}
	else
	{
		fn_forward_l = 0;
		fn_forward_r = 0;
	}
	if (ch->robo_status.behavior == ROBO_BX_JUMP)
	{
		Jump_Phase(ch);
		leg_l.F0 = fn_forward_l +
				   PID_Update(&chassis_motor_pid[0], set.left_length, leg_l.L0);
		leg_r.F0 = fn_forward_r +
				   PID_Update(&chassis_motor_pid[1], set.right_length, leg_r.L0);
	}
	else if (ch->robo_status.flag.above == true)
	{
		leg_l.F0 = fn_forward_l +
				   PID_Update(&chassis_motor_pid[0], set.left_length, leg_l.L0);
		leg_r.F0 = fn_forward_r +
				   PID_Update(&chassis_motor_pid[1], set.right_length, leg_r.L0);
	}
	else
	{
		leg_l.F0 = fn_forward_l +
				   PID_Update(&chassis_motor_pid[0], set.left_length + set.roll_set_now, leg_l.L0);
		leg_r.F0 = fn_forward_r +
				   PID_Update(&chassis_motor_pid[1], set.right_length - set.roll_set_now, leg_r.L0);
	}

	/// @brief 杆扭矩 PID
	if (my_debug.no_tp_flag)
	{
		tplqrl = PID_Update(&pid_tpl, PI / 2.0f, leg_l.phi0);
		tplqrr = PID_Update(&pid_tpr, PI / 2.0f, leg_r.phi0);
	}

	/// @brief 安全无力
	if (ch->rc_data.rc.s[S_R] == MID)
	{
		tplqrl = 0.0f;
		tplqrr = 0.0f;
		tlqrl = 0.0f;
		tlqrr = 0.0f;
	}

	leg_l.Tp = tplqrl * tp_ratiol;
	leg_r.Tp = tplqrr * tp_ratior;

	/// @brief 正 VMC
	VMC_calc_2(&leg_l);
	VMC_calc_2(&leg_r);

	/// @brief 发送 buf
	set.set_cal_real[0] = leg_r.torque_set[0];
	set.set_cal_real[1] = leg_r.torque_set[1];
	set.set_cal_real[3] = leg_l.torque_set[0];
	set.set_cal_real[2] = leg_l.torque_set[1];

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

uint8_t jump_time = 0;
uint8_t jump_status = 0;
/************************
 * @brief 跳跃过程
 *
 * @param ch
 ************************/
void Jump_Phase(Chassis_t *ch)
{
	if (ch->robo_status.last_behavior != ROBO_BX_JUMP && ch->robo_status.behavior == ROBO_BX_JUMP && jump_status == 0)
	{
		jump_status = 1;
		chassis_motor_pid[0].Kp = chassis_motor_pid[1].Kp = 1500.0f;
		chassis_motor_pid[0].Kd = chassis_motor_pid[1].Kd = 7000.0f;
	}

	/// @brief 收缩段
	if (jump_status == 1)
	{
		set.left_length = set.right_length = 0.1f;

		jump_time++;

		if (jump_time >= 10)
		{
			jump_time = 0;
			jump_status = 2; // 压缩完毕进入上升加速阶段
		}
	}
	/// @brief 起跳段
	else if (jump_status == 2)
	{
		set.left_length = set.right_length = 0.3f;
		fn_forward_l = fn_forward_r = 100.0f;

		jump_time++;

		if (jump_time >= 1)
		{
			jump_time = 0;
			jump_status = 3; // 上升完毕进入缩腿阶段
		}
	}
	/// @brief 缩腿
	else if (jump_status == 3)
	{
		set.left_length = set.right_length = 0.15f;
		fn_forward_l = fn_forward_r = -25.0f;

		jump_time++;

		if (jump_time >= 5)
		{
			jump_time = 0;
			jump_status = 0;

			ch->robo_status.behavior = ROBO_BX_NORMAL;

			chassis_motor_pid[0].Kp = chassis_motor_pid[1].Kp = 500;
			chassis_motor_pid[0].Kd = chassis_motor_pid[1].Kd = 6000;
		}
	}
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
