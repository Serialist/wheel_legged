#ifndef __CHASSISMOTOR_H__
#define __CHASSISMOTOR_H__

#include "stdint.h"
#include "remote_control.h"
#include "referee_system.h"
#include "main.h"
#include "struct_typedef.h"
// #include "bsp_usart.h"

// DJI motors
#define CHASSIS_MOTOR1_ID 0x201
#define CHASSIS_MOTOR2_ID 0x202
// cubemars motors

#define AK_ID_1 0x2963 // L
#define AK_ID_2 0x2964 // R

#define AK_ID_3 0x2965
#define AK_ID_4 0x2966
#define AK_ID_5 0x2967
#define AK_ID_6 0x2968

#define AK_ID_11 0x63 // L
#define AK_ID_21 0x64 // R

#define AK_ID_31 0x65
#define AK_ID_41 0x66
#define AK_ID_51 0x67
#define AK_ID_61 0x68

#define M1 0
#define M2 1
// 两腿
#define AK1 0
#define AK2 1
// 左侧
#define AK3 2
#define AK4 3
// 右侧
#define AK5 4
#define AK6 5

// 串级PID调参

#define M1I 7
#define M2I 8

#define REDUCTION_RATIO_M3508 19.20320855614973f
#define REDUCTION_RATIO_M2006 36.20320855614973f
// 扭矩常数
#define KTAK10 1 / 0.198f
#define KTAK60 1 / 0.135f

// #define REMOTE_CHANNLE_TO_CHASSIS_SPEED 0.0053f

#define LEFT 0
#define RIGHT 1

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define V_MIN_2 -45.0f
#define V_MAX_2 45.0f
#define T_MIN -65.0f
#define T_MAX 65.0f
#define T_MIN_2 -15.0f
#define T_MAX_2 15.0f
#define Kp_MIN 0
#define Kp_MAX 500.0f
#define Kd_MIN 0
#define Kd_MAX 5.0f

#define wheelRadius 0.077f
#define Mg 1.482f * 9.8f

//**********************************************************//
// motors//
// cubemars//

typedef struct
{
	float prev_theta; // 上次角度
	float v_prev;	  // 前次速度
	float v_prev2;	  // 前前次速度
	float b0, a1, a2; // 滤波器系数
} SpeedEstimator;

typedef struct
{
	float motor_pos;   // 电机位置
	float motor_spd;   // 电机速度
	float motor_cur;   // 电机电流
	float motor_temp;  // 电机温度
	float motor_error; // 电机故障码
} AK_motor_fdb_t;

typedef struct
{
	float motor_ctrlpos;   // 电机位置
	float motor_ctrlspd;   // 电机速度
	float motor_ctrltor;   // 电机扭矩
	float motor_ctrltemp;  // 电机温度
	float motor_ctrlerror; // 电机故障码
} AK_motor_ctrl_fdb_t;

typedef enum
{
	CAN_PACKET_SET_DUTY = 0,	  // 占空比模式
	CAN_PACKET_SET_CURRENT,		  // 电流环模式
	CAN_PACKET_SET_CURRENT_BRAKE, // 电流刹车模式
	CAN_PACKET_SET_RPM,			  // 转速模式
	CAN_PACKET_SET_POS,			  // 位置模式
	CAN_PACKET_SET_ORIGIN_HERE,	  // 设置原点模式
	CAN_PACKET_SET_POS_SPD,		  // 位置速度环模式
} CAN_PACKET_ID;

typedef struct
{

	float spdset;
	float posset;
	float torset;

} CM_TRANSMIT_DATA;

// DJI//
typedef struct
{
	int16_t angle_first;
	int16_t angle;
	int16_t angle_last;
	int16_t angle_set;

} MOTOR_ANGLE;

typedef struct
{
	int16_t ecd_fdb;
	int16_t speed_rpm_fdb;
	int16_t current_fdb;
	int16_t temperate_fdb;
	int16_t motor_old_angle;
	int16_t cycle_num;
	int16_t angle_err;

	fp32 speed;

} MOTOR_RECEIVE_DATA;

typedef struct
{

	int16_t motor_speed_set;
	int16_t motor_current_set;
	int16_t motor_speed_set_calc;

} MOTOR_TRANSMIT_DATA;

//**********************************************************//
// config//
typedef struct
{
	uint32_t omni_motor[2]; // 电机反馈时间
	// uint32_t momentum_motor[2];
	uint32_t speed_set[2]; // 底盘速度设定时间
	uint32_t ree_power;	   // 裁判系统反馈功率时间
	uint32_t ree_statue;   // 裁判系统反馈机器人状态时间
	uint32_t model;
	uint32_t imu;
	uint32_t rc;
} Time_t;

typedef struct
{
	float omni_motor_cur[2];
	// float momentum_motor_cur[2];
	float cur_sum;
	float cur_filter; // 滤波后的结果

	float adc_scale;
} Chassis_Current_t;

#ifdef OPEN_SUPER_POWER
typedef struct // 超级电容数据
{
	float input_vot;	// 输入电压
	float cap_vot;		// 电容电压
	float input_cur;	// 输入电流
	float target_power; // 设定功率
						//	Flag control_flag; //超级电容开关标志，1开，0关
} Super_Power_t;
#endif

typedef struct
{
	// 机体加速度
	float ax;
	float az;
	// 机体角速度
	float yawspd;
	float pitchspd;
	float rollspd;
	// 欧拉角
	float yaw;
	float toatalyaw;
	float roll;
	float pitch;
} IMU_FDB;

typedef struct Robo_Attitude_Def_t
{
	float yaw, v_yaw, a_yaw;
	float pitch, v_pitch, a_pitch;
	float roll, v_roll, a_roll;

	float x, v_x, a_x;
	float y, v_y, a_y;
	float z, v_z, a_z;

	float f_x, f_y, f_z;
	float m_x, m_y, m_z;
} Robo_Attitude_Def_t;

typedef struct Leg_Def_t
{
	float theta, v_theta;
	float alpha, v_lpha;

} Leg_Def_t;

#define LEFT 0
#define RIGHT 1

typedef struct Chassis_Def_t
{
	Robo_Attitude_Def_t attitude;
	Leg_Def_t leg[2];

} Chassis_Def_t;

typedef struct
{
	float thetal, dThetal, thetar, dThetar;
	float v_filter; // 滤波后的车体速度，单位是m/s
	float x_filter; // 滤波后的车体位置，单位是m
	float vl, vr;	// m/s 实际控制的目标前进速度（加入积分项消除静差）
	float xl, xr;
	float phi, dPhi;
	float angle_err;
	float alphal, d_alphal, alphar, d_alphar;
	float FNl, FNr;
	float F0l, F0r;
	float TPl, TPr;
} State_Var_s;

////////////////////////////////////////////////////////////////////////////////////////////////

#include "robo_behavior.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	MOTOR_RECEIVE_DATA gimbal_motor_fdb;
	//	Motor_Feedback_t momentum_motor_fdb[2];
	//	Motor_Set_t momentum_motor_set[2];

	float chassis_speed_set[2];
	float speed_calc_sets[3];
	AK_motor_fdb_t AK_fdb[6];
	AK_motor_ctrl_fdb_t ak_fdb_ctrl[6];
	CM_TRANSMIT_DATA ak_set[6];
	float current_set[6];

	Power_Heat_Data_t power_data;

	Game_Robot_Status_t status;

	Chassis_Current_t current;

	Time_t time_now;
	Time_t time_last;
	Time_t time_error;

#ifdef OPEN_SUPER_POWER
	Super_Power_t super_power;
#endif

	IMU_FDB IMU_DATA;

	Flag no_force_mode;

	RC_ctrl_t rc_data;

	float middle_data;

	uint8_t recover_flag;

	State_Var_s st;

	struct Robo_Status robo_status;

} Chassis_t;

typedef struct
{
	float kRatio[2][6];
	float LQR_Tp_ratio;
	float LQR_T_ratio;
	float length_ratio;
} Ratio_t;

struct Wheel_Leg_Target
{
	float position_set; // m  期望达到的位置
	float v_set;
	float speed_cmd;	   // m/s 期望达到的目标前进速度
	float speed_integral;  // m/s 速度积分项
	float rotation_torque; // N*m 旋转力矩
	float yaw;			   // rad 期望达到的目标航向角
	float pitch;		   // rad 期望达到的目标俯仰角
	float roll;			   // rad 期望达到的目标横滚角
	float roll_set_now;
	float leg_length;					   // m  期望达到的目标腿长
	float left_length, right_length;	   // m  期望达到的目标腿长
	float left_leg_angle, right_leg_angle; // rad 期望达到的目标腿角
	float set_cal_real[6];

	float vyaw;

	float mode_state;
	float rotate_state;
	float shoot_state;
};

/**
 * 古代代码，不建议使用
 */
typedef struct
{
	float angle, length;   // rad, m
	float dAngle, dLength; // rad/s, m/s
	float ddLength;		   // m/s^2

} Leg_Pos_t;

extern void chassis_task(void const *argument);

// DJI motor//
void TransmitChassisMotorCurrent(int16_t o1, int16_t o2);
// 伺服模式//
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);
// 运控模式//
void controller_init(uint8_t id);
void controller_close(uint8_t id);
void controller_setorigin(uint8_t id);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
void pack_cmd(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff);

/* ================================ new motor program ================================ */

struct Motor_AK_Rx_Data
{
	uint8_t id;
	float position; // 位置
	float speed;	// 速度
	float torque;	// 扭矩
	float temp;		// 温度
	float errCode;	// 故障码
};

float Uint_To_Float(int x_int, float x_min, float x_max, int bits);
void Motor_AK_MIT_Decode(struct Motor_AK_Rx_Data *rxData, uint8_t data[8],
						 float pMax, float vMax, float tMax);

#endif
