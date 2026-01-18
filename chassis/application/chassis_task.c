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
#include "leg_conv_1.h"
#include "leg_spd_1.h"
#include "leg_pos_1.h"
#include "LQR_K.h"
#include "user_lib.h"
#include "DM_VMC_test.h"
#include "can.h"
#include "debug.h"
#include "filter.h"
#include "motor.h"

#ifndef LIMIT
#define LIMIT(input, min, max) ((input) > (max)	  ? (max) \
								: (min) < (input) ? (min) \
												  : (input))
#endif

#ifndef ABS
#define ABS(vaule) ((vaule) > 0 ? (vaule) : -(vaule))
#endif

/**********  变量定义 开始 *************/
Chassis_t chassis;
pid_type_def chassis_motor_pid[10];

/**
 * @brief VMC腿摆角扭矩pid，用于板凳模型
 * @date 2025-10-16
 */
pid_type_def pid_tpl = {0}, pid_tpr = {0};

pid_type_def chassis_motor_pid[10];

/**********  变量定义 结束 *************/
float T_AK_set_left[2];
float T_AK_set_right[2];

struct Wheel_Leg_Target set;
Leg_Pos_t legpos[2];
struct VMC_Leg leg_l, leg_r;

/* ======================== 测试代码使用的变量 ======================== */

float lefttp_test;
float leftforce_test;
float righttp_test;

float leftforce;
float rightforce;
const float kk = 1.57079632679489661923f;

float turn_t; // yaw轴补偿
float leg_tp; // 防劈叉补偿
float total_yaw;

float LEGPOS[2], LEGPOS1[2];
float LEGSPD[2], LEGSPD1[2];
float LEGTOR[2], LEGTOR1[2];

float legL_dphi1;
float legL_dphi4;
float legR_dphi1;
float legR_dphi4;

float legL_phi1;
float legL_phi4;
float legR_phi1;
float legR_phi4;

uint8_t left_flag;
uint8_t right_flag;
uint8_t leg_flag;

/* ======================== 测试代码使用的变量 ======================== */

float tplqrl;
float tplqrr;
float tlqrl;
float tlqrr;

float lqr_k_l[2][6], lqr_k_r[2][6], kl[2][6], xl[6], kr[2][6], xr[6];

/// @brief 离地检测滤波
struct Filter_Average ground_detection_filter_l, ground_detection_filter_r;

// typedef uint8_t My_Status_Def;

// typedef enum Robo_Status_Def_e
// {
// 	ROBO_INIT = 0
// } Robo_Status_Def_e;

/********** 函数声明 开始 *************/

static void ChassisInit(void);
static void Fdb_Update(Chassis_t *ch);
void chassis_torque_sent(Chassis_t *ch);
void balancephase(Chassis_t *ch);
void jumpphase(Chassis_t *ch);
bool Ground_Detection_L(Chassis_t *ch, struct VMC_Leg *leg);
bool Ground_Detection_R(Chassis_t *ch, struct VMC_Leg *leg);
void get_order(Chassis_t *ch);
void mySaturate(float *in, float min, float max);
void safe(Chassis_t *ch);
void LQR_K_Calc(float k[2][6], float coe[12][4], float len);
void Motor_Enable(void);

/********** 函数声明 结束 *************/

/* USER CODE END chassis_task */
void chassis_task(void const *argument)
{
	ChassisInit();
	VMC_init(&leg_l);
	VMC_init(&leg_r);

	while (chassis.robo_status.status == ROBO_STATUS_INIT)
	{
		osDelay(1);
	}

	for (;;)
	{
		// Sent_YAW_Data(); // 暂时没用

		// ================================ 状态更新 ================================

		Fdb_Update(&chassis);

		// 控制器//
		get_order(&chassis);

		balancephase(&chassis);

		// ================================ 电机控制指令 ================================
		safe(&chassis);
		chassis_torque_sent(&chassis);

		osDelay(1);
	}
	/* USER CODE END chassis_task */
}

//************************基础控制器*********************************/
// 系统初始化
static void ChassisInit(void)
{
	// 腿部运动PID初始化
	// 腿长
	PID_init(&chassis_motor_pid[0], PID_POSITION, 500, 0, 6000, 80, 0);
	PID_init(&chassis_motor_pid[1], PID_POSITION, 500, 0, 6000, 80, 0);

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

	// ================================================================
	/**
	 * @brief VMC腿摆角扭矩pid，用于板凳模型
	 * @date 2025-10-16
	 */
	PID_init(&pid_tpl, PID_POSITION, 80, 0, 400, 10, 0);
	PID_init(&pid_tpr, PID_POSITION, 80, 0, 400, 10, 0);

	set.left_length = set.right_length = 0.2f;

	Filter_Average_Init(&ground_detection_filter_l, 10);
	Filter_Average_Init(&ground_detection_filter_r, 10);

	Motor_Enable();

	// Reg_Add();
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
		vTaskDelay(2);
	}
	// for (int bb = 4; bb < 6; bb++)
	// {
	// 	controller_init(bb + 1);
	// 	controller_setorigin(bb + 1);
	// 	vTaskDelay(2);
	// }
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
	GetgimbalmotorFdb(&ch->gimbal_motor_fdb);
	GetAKMotor1Fdb(&ch->AK_fdb[0], &ch->ak_fdb_ctrl[0]);
	GetAKMotor2Fdb(&ch->AK_fdb[1], &ch->ak_fdb_ctrl[1]);
	GetAKMotor3Fdb(&ch->AK_fdb[2], &ch->ak_fdb_ctrl[2]);
	GetAKMotor4Fdb(&ch->AK_fdb[3], &ch->ak_fdb_ctrl[3]);
	GetAKMotor5Fdb(&ch->AK_fdb[4], &ch->ak_fdb_ctrl[4]);
	GetAKMotor6Fdb(&ch->AK_fdb[5], &ch->ak_fdb_ctrl[5]);

	// 获取遥控器数据
	ch->time_now.rc = GetRCData(&(ch->rc_data));
	// if (ch->rc_data.rc.s[0] != UP)
	// 	ch->no_force_mode = SHUT;
	// else
	// 	ch->no_force_mode = OPEN;

	ch->robo_status.flag.above_left = Ground_Detection_L(ch, &leg_l);
	ch->robo_status.flag.above_right = Ground_Detection_R(ch, &leg_r);

	ch->robo_status.flag.above = (ch->robo_status.flag.above_left == true) && (ch->robo_status.flag.above_right == true);
}

void chassis_sys_calc(Chassis_t *ch)
{
	//  const float lpfRatio = 0.5f;
	//	float lastLeftDLength = 0, lastRightDLength = 0;
	legR_phi1 = pi / 2.0f - ch->ak_fdb_ctrl[0].motor_ctrlpos;
	legR_phi4 = pi / 2.0f - ch->ak_fdb_ctrl[1].motor_ctrlpos;

	legL_phi1 = pi / 2.0f - ch->ak_fdb_ctrl[2].motor_ctrlpos;
	legL_phi4 = pi / 2.0f - ch->ak_fdb_ctrl[3].motor_ctrlpos;

	leg_l.phi1 = legL_phi1;
	leg_l.phi4 = legL_phi4;
	leg_r.phi1 = legR_phi1;
	leg_r.phi4 = legR_phi4;

	VMC_calc_1(&leg_l, &chassis, 3.0f / 1000.0f);
	VMC_calc_1(&leg_r, &chassis, 3.0f / 1000.0f);
}

void phase_update(Chassis_t *ch)
{
	ch->st.phi = ch->IMU_DATA.pitch;
	ch->st.dPhi = ch->IMU_DATA.pitchspd;

	ch->st.xr = ch->st.x_filter;
	ch->st.vr = ch->st.v_filter;

	ch->st.xl = ch->st.x_filter;
	ch->st.vl = ch->st.v_filter;

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
}

// 扭矩发送

/************************
 * @brief oria
 *
 * @param ch
 ************************/
void chassis_torque_sent(Chassis_t *ch)
{

	if (ch->robo_status.status != ROBO_STATUS_RUN ||
		ch->rc_data.rc.s[S_R] == UP ||
		ch->rc_data.rc.s[S_L] != UP)
	{
		for (int i = 0; i < 6; i++)
			ch->ak_set[i].torset = 0;
	}

	// MIT模式下发送
	pack_cmd(1, 0, 0, 0, 0, ch->ak_set[0].torset);
	pack_cmd(3, 0, 0, 0, 0, ch->ak_set[3].torset);
	delay_ms(1);
	pack_cmd(2, 0, 0, 0, 0, ch->ak_set[1].torset);
	pack_cmd(4, 0, 0, 0, 0, ch->ak_set[2].torset);
	delay_ms(1);

	/// @brief 不用这个轮毂电机
	// pack_cmd(5, 0, 0, 0, 0, ch->ak_set[4].torset);
	// pack_cmd(6, 0, 0, 0, 0, ch->ak_set[5].torset);
	// delay_ms(1);

	/// @brief 用 3508
	DJI_Motor_Transmit(&hcan1, M3508_TX_ID_2,
					   0,
					   HEXROLL_TORQUE_TO_CURRENT(ch->ak_set[4].torset),
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
void get_order(Chassis_t *ch)
{
	ch->robo_status.last_behavior = ch->robo_status.behavior;

	if (ch->rc_data.rc.s[S_R] == DOWN) // 正常行驶
	{
		set.speed_cmd = ch->rc_data.rc.ch[L_Y] * 2.0f / 660.0f;
		set.yaw -= ch->rc_data.rc.ch[L_X] * REMOTE_CHANNLE_TO_CHASSIS_SPEED * 2.0f;
		set.left_length = set.right_length = (ch->rc_data.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;

		if (set.speed_cmd != 0)
			set.position_set = ch->st.x_filter;
	}
	else
	{
		set.speed_cmd = 0;
		set.yaw = total_yaw;
		set.roll = 0;
		set.left_length = set.right_length = 0.2f;
		set.position_set = ch->st.x_filter;
	}
	/// @todo 改写成行为树
}

// 保险
void safe(Chassis_t *ch)
{
	if (ch->robo_status.status == ROBO_STATUS_EMERGENCY)
	{
		if (MOTOR_IS_OFFLINE(&motor_status))
		{
			Motor_Enable();
		}
	}
}

// 限幅
void mySaturate(float *in, float min, float max)
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

#define GROUND_DETECTION_THRESHOLD 15.0f // threshold 阈值

float aaa = 0.6f, bbb, ccc;

/**
 * @brief 触地检测器 right
 *
 * @param ch
 * @param leg
 * @return true
 * @return false
 */
bool Ground_Detection_R(Chassis_t *ch, struct VMC_Leg *leg)
{
	ch->st.FNr = leg->F0 * arm_cos_f32(leg->theta) +
				 leg->Tp * arm_sin_f32(leg->theta) / leg->L0 +
				 aaa * (ch->IMU_DATA.az +
						(-leg->dd_L0 * arm_cos_f32(leg->theta)) +
						2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
						leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
						leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

	ch->st.FNr = Filter_Average_Update(&ground_detection_filter_r, ch->st.FNr);

	return (ch->st.FNr < GROUND_DETECTION_THRESHOLD) ? true : false;
}

/**
 * @brief 触地检测器 left
 *
 * @param ch
 * @param leg
 * @return true
 * @return false
 */
bool Ground_Detection_L(Chassis_t *ch, struct VMC_Leg *leg)
{
	ch->st.FNl = leg->F0 * arm_cos_f32(leg->theta) +
				 leg->Tp * arm_sin_f32(leg->theta) / leg->L0 +
				 aaa * (ch->IMU_DATA.az +
						(-leg->dd_L0 * arm_cos_f32(leg->theta)) +
						2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
						leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
						leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

	ch->st.FNl = Filter_Average_Update(&ground_detection_filter_l, ch->st.FNl);

	return (ch->st.FNl < GROUND_DETECTION_THRESHOLD) ? true : false;
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
float fn_feedforward = 0;

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
/// @date 2026-01-17 19:33
/// @date 2026-01-17 23:46
/// @date 2026-01-17 23:52
/// @date 2026-01-18 00:13
/// @date 2026-01-18 00:21
float lqr_coe[12][4] = {
	{-162.891548088904898, 209.825767990235988, -145.162318570512610, 1.802361609572146},
	{288.541957208171027, -1059.881001656887065, 1248.852764630539014, 4.606398632155034},
	{-10.666991175454511, -29.208083318842050, 32.527802047541392, 0.352173403731176},
	{42.815355450779101, -114.451559133969297, 116.881834572211702, -0.208541551002214},
	{-100.779578104108296, 277.867162661583677, -282.789484357647495, 2.395219726268987},
	{17.789411613746740, -348.394452976747118, 568.637969156218333, 26.160943731961730},
	{-74.137697660529227, 180.411941476293805, -176.086082504311605, 1.428876625094395},
	{27.687935055962470, -284.589050581211325, 435.324523375807075, 18.526293003324142},
	{-180.511452674415693, 152.385711869816902, 69.273816444185741, 64.671534532996944},
	{1025.372279243355024, -2739.993662267117088, 2694.233516230424812, 27.143424304316461},
	{-13.478573980213060, 12.317395600605760, 2.004706291011256, 5.206297262813195},
	{57.237555776681113, -135.537553863169705, 118.030355105522801, -0.287569894854273}};

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void balancephase(Chassis_t *ch)
{
	/* ================================ 补充控制 ================================ */

	turn_t = chassis_motor_pid[YAW_PID].Kp * (set.yaw - total_yaw) - chassis_motor_pid[YAW_PID].Kd * ch->IMU_DATA.yawspd; // 这样计算更稳一点

	// leg_tp = PID_Calc(&chassis_motor_pid[TP_PID], 0.0f, ch->st.angle_err);					// 防劈叉pid计算
	// set.roll_set_now = PID_Calc(&chassis_motor_pid[ROLL_PID], set.roll, ch->IMU_DATA.roll); // roll 补偿

	/* ================================ LQR 控制 ================================ */

	LQR_K_Calc(lqr_k_l, lqr_coe, leg_l.L0);
	LQR_K_Calc(lqr_k_r, lqr_coe, leg_r.L0);

	if (ch->robo_status.flag.above)
	{
		set.speed_cmd = 0;
		set.yaw = total_yaw;
		set.roll = 0;
		set.position_set = ch->st.x_filter;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	xl[0] = (ch->st.thetal);
	xl[1] = (ch->st.dThetal);
	xl[2] = (ch->st.x_filter - set.position_set);
	xl[3] = (ch->st.v_filter - set.speed_cmd);
	xl[4] = (ch->st.phi);
	xl[5] = (ch->st.dPhi);

	xr[0] = (ch->st.thetar);
	xr[1] = (ch->st.dThetar);
	xr[2] = (ch->st.x_filter - set.position_set);
	xr[3] = (ch->st.v_filter - set.speed_cmd);
	xr[4] = (ch->st.phi);
	xr[5] = (ch->st.dPhi);

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

	if (my_debug.torque_flag)
	{
		tplqrl = my_debug.tpl;
		tplqrr = my_debug.tpr;
		tlqrl = my_debug.tl;
		tlqrr = my_debug.tr;
	}

	/// @brief 限幅
#define TPLQR_MAX 10.0f
#define TLQR_MAX 2.0f
	mySaturate(&tplqrl, -TPLQR_MAX, TPLQR_MAX);
	mySaturate(&tplqrr, -TPLQR_MAX, TPLQR_MAX);
	mySaturate(&tlqrl, -TLQR_MAX, TLQR_MAX);
	mySaturate(&tlqrr, -TLQR_MAX, TLQR_MAX);

	/* ================================ 轮 解算 ================================ */

	/// @brief 轮毂力矩计算
	if (my_debug.no_t_flag || ch->robo_status.flag.above)
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
	fn_feedforward = RAMP(fn_feedforward,
						  (ch->robo_status.flag.above ? -20.0f : 55.0f) * arm_cos_f32(leg_r.theta),
						  -100, 100, 0.003f);

	leg_l.F0 = fn_feedforward +
			   PID_Calc(&chassis_motor_pid[0], set.left_length, leg_l.L0);
	leg_r.F0 = fn_feedforward +
			   PID_Calc(&chassis_motor_pid[1], set.right_length, leg_r.L0);

	// /// @brief 杆扭矩 PID
	// if (my_debug.no_tp_flag)
	// {
	// 	tplqrl = PID_Calc(&pid_tpl, PI / 2.0f, leg_l.phi0);
	// 	tplqrr = PID_Calc(&pid_tpr, PI / 2.0f, leg_r.phi0);
	// }

	if (ch->robo_status.flag.above)
	{
		tplqrl = lqr_l_[6] + lqr_l_[7];
		tplqrr = lqr_r_[6] + lqr_r_[7];
	}

	leg_l.Tp = tplqrl;
	leg_r.Tp = tplqrr;

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
#define HIP_TORQUE_MAX 30.0f
#define HUB_TORQUE_MAX 2.5f
	mySaturate(&set.set_cal_real[0], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[1], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[2], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[3], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[4], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	mySaturate(&set.set_cal_real[5], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);

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
Robo_Behavior_Def last_jump_flag = ROBO_BX_FOLLOW;
/************************
 * @brief 跳跃过程
 *
 * @param ch
 ************************/
void jumpphase(Chassis_t *ch)
{
	if (ch->robo_status.last_behavior != ROBO_BX_JUMP && ch->robo_status.behavior == ROBO_BX_JUMP)
	{
		jump_status = 1;
		chassis_motor_pid[0].Kp = chassis_motor_pid[1].Kp = 1500.0f;
		chassis_motor_pid[0].Kd = chassis_motor_pid[1].Kd = 2000.0f;
	}

	/// @brief 收缩段
	if (jump_status == 1)
	{
		set.left_length = set.right_length = 0.1f;

		if (leg_r.L0 < 0.2f && leg_l.L0 < 0.2f)
		{
			jump_time++;
		}
		if (jump_time >= 40)
		{
			jump_time = 0;
			jump_status = 2; // 压缩完毕进入上升加速阶段
		}
	}
	/// @brief 起跳段
	else if (jump_status == 2)
	{
		set.left_length = set.right_length = 0.3f;
		fn_feedforward = 100.0f;

		if (leg_r.L0 > 0.25f && leg_l.L0 > 0.25f)
		{
			jump_time++;
		}
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
		fn_feedforward = -10.0f;

		if (leg_r.L0 < 0.15f && leg_l.L0 < 0.15f)
		{
			jump_time++;
		}
		if (jump_time >= 80)
		{
			jump_time = 0;
			jump_status = 0;

			ch->robo_status.behavior = ROBO_BX_FOLLOW;

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
