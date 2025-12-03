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

float lqr_k_l[2][6], lqr_k_r[2][6], xl[6], xr[6];

float t_ratiol;
float tp_ratiol;
float t_ratior;
float tp_ratior;

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
void chassis_VMC_pid(Chassis_t *ch);
void chassis_torque_sent(Chassis_t *ch);
void standphase(Chassis_t *ch);
void balancephase(Chassis_t *ch);
void balancephase_one_rod(Chassis_t *ch);
void jumpphase(Chassis_t *ch);
void chassis_sent_test(Chassis_t *ch);
uint8_t Ground_Detection_L(Chassis_t *ch, struct VMC_Leg *leg);
uint8_t Ground_Detection_R(Chassis_t *ch, struct VMC_Leg *leg);
void get_order(Chassis_t *ch);
void mySaturate(float *in, float min, float max);
void safe(Chassis_t *ch);
void protection(float TP_ctrl);
void protection_L(float TP_ctrl_L_input);
void protection_R(float TP_ctrl_R_input);
void LQR_K_Calc(float k[2][6], float coe[12][4], float len);
void Motor_Init(void);

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
		// balancephase_one_rod(&chassis);

		// ================================ 测试代码专用部分 ================================
		// chassis_sent_test(&chassis);
		// chassis_VMC_pid(&chassis);
		// ================================ / 测试代码专用部分 ================================

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

	// ================================================================
	/**
	 * @brief VMC腿摆角扭矩pid，用于板凳模型
	 * @date 2025-10-16
	 */
	// left
	pid_tpl.Kp = 80;
	pid_tpl.Ki = 0;
	pid_tpl.Kd = 400;
	pid_tpl.max_out = 10;
	pid_tpl.max_iout = 0;
	// right
	pid_tpr.Kp = 80;
	pid_tpr.Ki = 0;
	pid_tpr.Kd = 400;
	pid_tpr.max_out = 10;
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
	GetgimbalmotorFdb(&ch->gimbal_motor_fdb);
	GetAKMotor1Fdb(&ch->AK_fdb[0], &ch->ak_fdb_ctrl[0]);
	GetAKMotor2Fdb(&ch->AK_fdb[1], &ch->ak_fdb_ctrl[1]);
	GetAKMotor3Fdb(&ch->AK_fdb[2], &ch->ak_fdb_ctrl[2]);
	GetAKMotor4Fdb(&ch->AK_fdb[3], &ch->ak_fdb_ctrl[3]);
	GetAKMotor5Fdb(&ch->AK_fdb[4], &ch->ak_fdb_ctrl[4]);
	GetAKMotor6Fdb(&ch->AK_fdb[5], &ch->ak_fdb_ctrl[5]);

	// 获取遥控器数据
	ch->time_now.rc = GetRCData(&(ch->rc_data));
	if (ch->rc_data.rc.s[0] != UP)
		ch->no_force_mode = SHUT;
	else
		ch->no_force_mode = OPEN;

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

	legL_phi1 = pi / 2.0f - ch->ak_fdb_ctrl[3].motor_ctrlpos;
	legL_phi4 = pi / 2.0f - ch->ak_fdb_ctrl[2].motor_ctrlpos;

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

	if (ch->robo_status.status != ROBO_STATUS_RUN)
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

	if (ch->rc_data.rc.s[S_R] == DOWN) // 使能模式//第一次拨动时起立
	{
		if (ch->rc_data.rc.s[S_L] == UP) // 正常行驶
		{
			if (ch->rc_data.rc.ch[R_Y] >= 0)
			{
				set.speed_cmd = ch->rc_data.rc.ch[L_Y] * REMOTE_CHANNLE_TO_CHASSIS_SPEED;
				set.yaw -= ch->rc_data.rc.ch[L_X] * REMOTE_CHANNLE_TO_CHASSIS_SPEED * 2.0f;
				set.left_length = set.right_length = (ch->rc_data.rc.ch[R_Y] * 0.01f) / 66 + 0.12f;
				slope_following(&set.speed_cmd, &set.speed_integral, 0.001f);

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
			else if (ch->rc_data.rc.ch[L_Y] < 0 && ch->rc_data.rc.ch[L_Y] != -660)
			{
				set.speed_cmd = ch->rc_data.rc.ch[L_Y] * REMOTE_CHANNLE_TO_CHASSIS_SPEED;
				set.yaw = ((ch->rc_data.rc.ch[L_X]) * PI) / 660;
				set.leg_length = 0.13f;
			}
			else if (ch->rc_data.rc.ch[R_Y] == -660) // 跳跃
			{
				set.speed_cmd = ch->rc_data.rc.ch[L_Y] * REMOTE_CHANNLE_TO_CHASSIS_SPEED;
				set.yaw = (ch->rc_data.rc.ch[L_X] / 660) * PI;

				ch->robo_status.behavior = ROBO_BX_JUMP; // 已开启跳跃 ------------------------------------------------------------------------
			}
		}
		else if (ch->rc_data.rc.s[S_L] == MID) // 小陀螺
		{
		}
	}
	else if (ch->rc_data.rc.s[S_R] == DOWN) // 测试模式
	{
	}

	/// @todo 改写成行为树

	// if (ch->rc_data.rc.s[S_R] == UP)
	// {
	// 	ch->robo_status.behavior = ROBO_BX_OFF;
	// }
	// else if (ch->rc_data.rc.s[S_R] == MID)
	// {
	// 	ch->robo_status.behavior = ROBO_BX_STOP;
	// }
	// else if (ch->rc_data.rc.s[S_R] == DOWN)
	// {
	// 	if (ch->rc_data.rc.ch[R_Y] == -660 && ch->rc_data.rc.s[S_L] == UP)
	// 	{
	// 		ch->robo_status.behavior = ROBO_BX_JUMP;
	// 	}
	// 	else if (ch->robo_status.flag.above_ground == FALSE)
	// 	{
	// 		ch->robo_status.behavior = ROBO_BX_FOLLOW;
	// 	}
	// 	else if (ch->robo_status.flag.above_ground == TRUE)
	// 	{
	// 		ch->robo_status.behavior = ROBO_BX_ABOVE_GROUND;
	// 	}
	// }
	// else
	// {
	// 	ch->robo_status.behavior = ROBO_BX_OFF;
	// }
}

// 保险
void safe(Chassis_t *ch)
{
	if (ch->no_force_mode == OPEN)
	{
		chassis_sent_test(ch);
	}
	else if (ch->rc_data.rc.s[S_R] == MID)
	{
		ch->ak_set[4].torset = 0.0f;
		ch->ak_set[5].torset = 0.0f;
	}

	if (ch->robo_status.status == ROBO_STATUS_EMERGENCY)
	{
		if (MOTOR_IS_OFFLINE(&motor_status))
		{
			Motor_Init();
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
/************************
 * @brief 触地检测器 right
 *
 * @param ch
 * @return uint8_t 离地 1 触地 0
 ************************/
float test__ = 0.0f;
uint8_t Ground_Detection_R(Chassis_t *ch, struct VMC_Leg *leg)
{
	my_debug.ground_det.f0r = leg->F0 * arm_cos_f32(leg->theta);
	my_debug.ground_det.tpr = leg->Tp * arm_sin_f32(leg->theta) / leg->L0;

	ch->st.FNr = my_debug.ground_det.f0r + my_debug.ground_det.tpr;

	ch->st.FNr = Filter_Average_Update(&ground_detection_filter_r, ch->st.FNr);

	//  0.6f * (-ch->IMU_DATA.az -
	// 		 leg->dd_L0 * arm_cos_f32(leg->theta) +
	// 		 2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
	// 		 leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
	// 		 leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

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

	ch->st.FNl = my_debug.ground_det.f0l + my_debug.ground_det.tpl;

	ch->st.FNl = Filter_Average_Update(&ground_detection_filter_l, ch->st.FNl);

	//  0.6f * (-ch->IMU_DATA.az -
	// 		 leg->dd_L0 * arm_cos_f32(leg->theta) +
	// 		 2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
	// 		 leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
	// 		 leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

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
float lqr_coe[12][4] = {
	{-58.407728972946963, -191.825800918549191, 391.901636677650117, -3.170048790861382},
	{187.646316791151008, -372.618291998268489, 212.163630184938114, -9.019099201404224},
	{-6.115003586824774, -55.080208526419291, 73.229934619808873, -0.301378246729556},
	{13.930434423810381, -7.595716526088610, -30.849131212435889, -0.860656824079749},
	{22.745711126151591, -276.784350245374412, 564.710464854528595, -10.363160968496230},
	{308.127709126556681, -1217.453463971467045, 1654.456236581708936, -20.581332549476109},
	{24.014690828187089, -244.953283531969504, 464.156854829597307, -8.773328817674795},
	{216.518291903973505, -847.952473901965391, 1153.404322401534046, -15.261643235943289},
	{-108.677400653816605, 347.683592676040291, -513.382221219050052, 36.128865915411197},
	{-46.627144165597500, 460.277127042534119, -799.828652357709871, 43.427898244747333},
	{-15.463523977813979, 74.549308651271417, -124.528531772650197, 5.291975196987041},
	{-35.370046022883130, 166.209761832402904, -242.782495057344391, 7.001915749732061}};

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
		mySaturate(&turn_t, -1.0f, 1.0f);
	}
	// leg_tp = PID_Calc(&chassis_motor_pid[TP_PID], 0.0f, ch->st.angle_err);					// 防劈叉pid计算
	set.roll_set_now = PID_Calc(&chassis_motor_pid[ROLL_PID], set.roll, ch->IMU_DATA.roll); // roll 补偿

	/* ================================ LQR 控制 ================================ */

	LQR_K_Calc(lqr_k_l, lqr_coe, leg_l.L0);
	LQR_K_Calc(lqr_k_r, lqr_coe, leg_r.L0);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	xl[0] = (ch->st.thetal);
	xl[1] = (ch->st.dThetal);
	xl[2] = (ch->st.xl - set.position_set);
	xl[3] = (ch->st.vl - set.speed_cmd - turn_t);
	xl[4] = (ch->st.phi);
	xl[5] = (ch->st.dPhi);

	xr[0] = (ch->st.thetar);
	xr[1] = (ch->st.dThetar);
	xr[2] = (ch->st.xr - set.position_set);
	xr[3] = (ch->st.vr - set.speed_cmd + turn_t);
	xr[4] = (ch->st.phi);
	xr[5] = (ch->st.dPhi);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/// @brief 核心 LQR 计算，公式 u = - K * x。分左右腿

	if (my_debug.no_above_det_flag != TRUE)
	{
		uint8_t i = 0;
		if (ch->robo_status.flag.above == true)
		{
			for (i = 0; i < 12; i++)
			{
				if (i == 6 || i == 7)
					kx[i] = 0;
				else
					kx[i] = KK[i % 6];
			}
			my_debug.no_yaw_flag = TRUE;
		}
		else
		{
			for (i = 0; i < 12; i++)
			{
				kx[i] = KK[i % 6];
			}
			my_debug.no_yaw_flag = FALSE;
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
#define TPLQR_MAX 10.0f
#define TLQR_MAX 2.5f
	mySaturate(&tplqrl, -TPLQR_MAX, TPLQR_MAX);
	mySaturate(&tplqrr, -TPLQR_MAX, TPLQR_MAX);
	mySaturate(&tlqrl, -TLQR_MAX, TLQR_MAX);
	mySaturate(&tlqrr, -TLQR_MAX, TLQR_MAX);

	/* ================================ 轮 解算 ================================ */

	/// @brief 轮毂力矩计算
	if (my_debug.no_t_flag)
	{
		set.set_cal_real[5] = 0;
		set.set_cal_real[4] = 0;
	}
	else
	{
		set.set_cal_real[5] = tlqrl;
		set.set_cal_real[4] = tlqrr;
	}

	/* ================================ 腿 解算 ================================ */

	/// @brief 腿推力 PID
	// 重力前馈
	if (!my_debug.no_g_fn_flag)
	{
		fn_forward_l = 55.0f * arm_cos_f32(leg_l.theta);
		fn_forward_r = 55.0f * arm_cos_f32(leg_r.theta);

		if (ch->robo_status.flag.above == true && ch->rc_data.rc.ch[R_Y] != 0)
		{
			tplqrl = lqr_l_[6] + lqr_l_[7];
			tplqrr = lqr_r_[6] + lqr_r_[7];

			// if (leg_l.L0 >= (set.left_length + 10.0f))
			// 	fn_forward_l *= -1.5f;
			// else if (leg_l.L0 < (set.left_length + 10.0f))
			// 	fn_forward_l = 0;

			// if (leg_r.L0 >= (set.right_length + 10.0f))
			// 	fn_forward_r *= -1.5f;
			// else if (leg_r.L0 < (set.right_length + 10.0f))
			// 	fn_forward_r = 0;
		}
		else
		{
		}
	}
	else
	{
		fn_forward_l = 0;
		fn_forward_r = 0;
	}
	if (ch->robo_status.behavior == ROBO_BX_JUMP)
	{
		jumpphase(ch);
		leg_l.F0 = fn_forward_l +
				   PID_Calc(&chassis_motor_pid[0], set.left_length, leg_l.L0);
		leg_r.F0 = fn_forward_r +
				   PID_Calc(&chassis_motor_pid[1], set.right_length, leg_r.L0);
	}
	else
	{
		leg_l.F0 = fn_forward_l +
				   PID_Calc(&chassis_motor_pid[0], set.left_length + set.roll_set_now, leg_l.L0);
		leg_r.F0 = fn_forward_r +
				   PID_Calc(&chassis_motor_pid[1], set.right_length - set.roll_set_now, leg_r.L0);
	}

	/// @brief 杆扭矩 PID
	if (my_debug.no_tp_flag)
	{
		tplqrl = PID_Calc(&pid_tpl, PI / 2.0f, leg_l.phi0);
		tplqrr = PID_Calc(&pid_tpr, PI / 2.0f, leg_r.phi0);
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
#define HIP_TORQUE_MAX 12.5f
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

float xl_[4];
float xr_[4];
float target_xl_[4] = {0, 0, 0, 0};
float target_xr_[4] = {0, 0, 0, 0};
float kResl_[4] = {-20.8598, -1.8225, -1.0000, -2.4085};
/***********************************************
 * @brief 板凳模型，定死腿平衡车
 *
 * @param ch
 *************************************************/
void balancephase_one_rod(Chassis_t *ch)
{
	// 	set.yaw = total_yaw;
	// 	turn_t = chassis_motor_pid[YAW_PID].Kp * (set.yaw - total_yaw) - chassis_motor_pid[YAW_PID].Kd * ch->IMU_DATA.yawspd; // 这样计算更稳一点
	// 	mySaturate(&turn_t, -0.5f, 0.5f);
	// 	leg_tp = PID_Calc(&chassis_motor_pid[TP_PID], 0.0f, ch->st.angle_err);					// 防劈叉pid计算
	// 	set.roll_set_now = PID_Calc(&chassis_motor_pid[ROLL_PID], set.roll, ch->IMU_DATA.roll); // 前馈pd

	/* ================================ LQR 和 T ================================ */

	// #define NO_MOVE
#ifndef NO_MOVE

	/// @brief 获取状态变量
	xl_[0] = (ch->st.xl - target_xl_[0]);
	xl_[1] = (ch->st.vl - target_xl_[0]);
	xl_[2] = -(ch->st.phi - target_xl_[0]);
	xl_[3] = -(ch->st.dPhi - target_xl_[0]);

	xr_[0] = (ch->st.xr - target_xl_[0]);
	xr_[1] = (ch->st.vr - target_xl_[0]);
	xr_[2] = -(ch->st.phi - target_xl_[0]);
	xr_[3] = -(ch->st.dPhi - target_xl_[0]);

	/// @brief 测试段：完全不考虑位移与速度
	// xl_[0] = 0.0f; // x
	// xl_[1] = 0.0f; // dx
	// xl_[2] = 0.0f; // phi
	// xl_[3] = 0.0f; // dphi
	// xr_[0] = 0.0f; // x
	// xr_[1] = 0.0f; // dx
	// xr_[2] = 0.0f; // phi
	// xr_[3] = 0.0f; // dphi
	// 测试段：结束

	/// @brief LQR 核心
	tlqrl = -(kResl_[2] * xl_[0] + kResl_[3] * xl_[1] + kResl_[0] * xl_[2] + kResl_[1] * xl_[3]);
	tlqrr = -(kResl_[2] * xr_[0] + kResl_[3] * xr_[1] + kResl_[0] * xr_[2] + kResl_[1] * xr_[3]);

	/// @brief 限幅
	mySaturate(&tlqrl, -0.8f, 0.8f);
	mySaturate(&tlqrr, -0.8f, 0.8f);

	/// @brief 左右轮电机扭矩计算，为LQR计算输出与YAW轴叠加
	set.set_cal_real[5] = tlqrl;
	set.set_cal_real[4] = tlqrr;

#else

	set.set_cal_real[4] = 0;
	set.set_cal_real[5] = 0;

#endif

	/* ================================ 定腿 ================================ */

	/// @brief 杆扭矩 PID
	leg_l.Tp = PID_Calc(&pid_tpl, 0, ch->st.thetal);
	leg_r.Tp = -PID_Calc(&pid_tpr, 0, ch->st.thetar);

	/// @brief 杆推力 PID
	// #define NO_FN_FORWORD
#ifndef NO_FN_FORWORD
	leg_l.F0 = PID_Calc(&chassis_motor_pid[0], set.left_length, leg_l.L0) +	 // 前馈
			   +55.0f / arm_cos_f32(leg_l.theta);							 // 立直重力前馈
	leg_r.F0 = PID_Calc(&chassis_motor_pid[1], set.right_length, leg_r.L0) + // 前馈
			   +55.0f / arm_cos_f32(leg_r.theta);							 // 立直重力前馈
#else
	leg_l.F0 = PID_Calc(&chassis_motor_pid[0], set.left_length, leg_l.L0);
	leg_r.F0 = PID_Calc(&chassis_motor_pid[1], set.right_length, leg_r.L0);
#endif

	/// @brief 正 VMC
	VMC_calc_2(&leg_l);
	VMC_calc_2(&leg_r);

	/// @brief 发送 buf 赋值
	set.set_cal_real[0] = leg_r.torque_set[0];
	set.set_cal_real[1] = leg_r.torque_set[1];
	set.set_cal_real[3] = leg_l.torque_set[0];
	set.set_cal_real[2] = leg_l.torque_set[1];

	/* ================================ 全部发送 ================================ */

	mySaturate(&set.set_cal_real[0], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[1], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[2], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[3], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	mySaturate(&set.set_cal_real[4], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	mySaturate(&set.set_cal_real[5], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);

	ch->ak_set[0].torset = set.set_cal_real[0];
	ch->ak_set[1].torset = set.set_cal_real[1];
	ch->ak_set[2].torset = -set.set_cal_real[2];
	ch->ak_set[3].torset = -set.set_cal_real[3];
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
		fn_forward_l = fn_forward_r = 100.0f;

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
		fn_forward_l = fn_forward_r = -10.0f;

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

// 超限保护
void protection(float TP_ctrl)
{
	protection_R(TP_ctrl);
	protection_L(TP_ctrl);
}

void protection_L(float TP_ctrl_L_input)
{
	if (leg_l.phi0 <= 1.22f && (TP_ctrl_L_input >= 0))
	{
		set.left_leg_angle = PID_Calc(&chassis_motor_pid[2], 1.22f, leg_l.phi0) - leg_tp;
	}
	else if (leg_l.phi0 <= 1.50f && (legpos[LEFT].dAngle <= -0.1f))
	{
		set.left_leg_angle = -PID_Calc(&chassis_motor_pid[2], 1.22f, leg_l.phi0) - leg_tp;
	}
	else if (leg_l.phi0 >= 1.92f && (TP_ctrl_L_input <= 0))
	{
		set.left_leg_angle = PID_Calc(&chassis_motor_pid[2], 1.92f, leg_l.phi0) - leg_tp;
	}
	else if (leg_l.phi0 >= 1.64f && (legpos[LEFT].dAngle >= 0.1f))
	{
		set.left_leg_angle = -PID_Calc(&chassis_motor_pid[2], 1.92f, leg_l.phi0) - leg_tp;
	}
	else
	{
		set.left_leg_angle = TP_ctrl_L_input - leg_tp;
	}
}

void protection_R(float TP_ctrl_R_input)
{
	if (leg_r.phi0 <= 1.22f && (TP_ctrl_R_input >= 0))
	{
		set.right_leg_angle = PID_Calc(&chassis_motor_pid[3], 1.22f, leg_r.phi0) + leg_tp;
	}
	else if (leg_r.phi0 <= 1.50f && (legpos[RIGHT].dAngle <= -0.1f))
	{
		set.right_leg_angle = -PID_Calc(&chassis_motor_pid[3], 1.22f, leg_r.phi0) + leg_tp;
	}
	else if (leg_r.phi0 >= 1.92f && (TP_ctrl_R_input <= 0))
	{
		set.right_leg_angle = PID_Calc(&chassis_motor_pid[3], 1.92f, leg_r.phi0) + leg_tp;
	}
	else if (leg_r.phi0 >= 1.64f && (legpos[RIGHT].dAngle >= 0.1f))
	{
		set.right_leg_angle = -PID_Calc(&chassis_motor_pid[3], 1.92f, leg_r.phi0) + leg_tp;
	}
	else
	{
		set.right_leg_angle = TP_ctrl_R_input + leg_tp;
	}
}

// 测试代码群
// 测试代码：电机状态获取
void chassis_sent_test(Chassis_t *ch)
{
	ch->ak_set[0].torset = 0.0f;
	ch->ak_set[1].torset = 0.0f;
	ch->ak_set[2].torset = 0.0f;
	ch->ak_set[3].torset = 0.0f;
	ch->ak_set[4].torset = 0.0f;
	ch->ak_set[5].torset = 0.0f;
}

// 测试代码：VMC
void chassis_VMC_pid(Chassis_t *ch)
{

	leftforce_test = (ch->rc_data.rc.ch[L_Y] * 0.01f) / 66 + 0.13f;
	lefttp_test = kk + (ch->rc_data.rc.ch[L_X] * kk) / 660;
	righttp_test = kk + (ch->rc_data.rc.ch[L_X] * kk) / 660;
	leg_tp = PID_Calc(&chassis_motor_pid[TP_PID], 0.0f, legpos[LEFT].angle - legpos[RIGHT].angle);

	set.left_length = PID_Calc(&chassis_motor_pid[0], leftforce_test, legpos[LEFT].length);
	set.right_length = PID_Calc(&chassis_motor_pid[1], leftforce_test, legpos[RIGHT].length);
	set.left_leg_angle = PID_Calc(&chassis_motor_pid[2], lefttp_test, legpos[LEFT].angle) - leg_tp;
	set.right_leg_angle = PID_Calc(&chassis_motor_pid[3], righttp_test, legpos[RIGHT].angle) + leg_tp;

	protection(ch->rc_data.rc.ch[L_X]);

	// MIT模式下扭矩计算
	leg_conv_1(set.left_length, set.left_leg_angle, legL_phi1, legL_phi4, LEGTOR);
	T_AK_set_left[0] = LEGTOR[0];
	T_AK_set_left[1] = LEGTOR[1];

	leg_conv_1(set.right_length, set.right_leg_angle, legR_phi1, legR_phi4, LEGTOR1);
	T_AK_set_right[0] = LEGTOR1[0];
	T_AK_set_right[1] = LEGTOR1[1];

	mySaturate(&T_AK_set_left[0], -3.5f, 3.5f);
	mySaturate(&T_AK_set_left[1], -3.5f, 3.5f);
	mySaturate(&T_AK_set_right[0], -3.5f, 3.5f);
	mySaturate(&T_AK_set_right[1], -3.5f, 3.5f);

	// 伺服模式下扭矩计算
	//	leg_conv_1(set.left_length,set.leg_angle,ch->AK_fdb[0].motor_pos,ch->AK_fdb[1].mo tor_pos,LEGTOR2);
	//  T_AK_set_left[0]=LEGTOR2[0];
	//  T_AK_set_left[1]=LEGTOR2[1];
	//
	//	leg_conv_1(set.right_length,set.leg_angle,ch->AK_fdb[3].motor_pos,ch->AK_fdb[2].motor_pos,LEGTOR3);
	//  T_AK_set_right[0]=LEGTOR3[0];
	//  T_AK_set_right[1]=LEGTOR3[1];`

	if (ch->rc_data.rc.s[S_L] == UP)
	{
		ch->ak_set[0].torset = -T_AK_set_right[0];
		ch->ak_set[1].torset = -T_AK_set_right[1];
		ch->ak_set[3].torset = T_AK_set_left[0];
		ch->ak_set[2].torset = T_AK_set_left[1];

		//		  if(ch->rc_data.rc.ch[R_Y]==660)
		//		  {
		//
		//				ch->ak_set[4].torset=-0.5f;
		//	      ch->ak_set[5].torset=0.5f;
		//		  }
	}

	//	ch->current_set[0]=ch->ak_set[0].torset*KTAK10;
	//  ch->current_set[1]=ch->ak_set[1].torset*KTAK10;
	//	ch->current_set[2]=ch->ak_set[2].torset*KTAK10;
	//	ch->current_set[3]=ch->ak_set[3].torset*KTAK10;
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
