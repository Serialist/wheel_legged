#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H
#include "struct_typedef.h"
#include "bsp_can.h"
#include "referee.h"
#include "remote_control.h"
#define APP_FR1 0
#define APP_FR2 1
#define ST  2
#define FRICTION_MOTOR_PID_KP 30
#define FRICTION_MOTOR_PID_KI 0
#define FRICTION_MOTOR_PID_KD 2
#define FRICTION_MOTOR_PID_MAX_IOUT 5
#define FRICTION_MOTOR_PID_MAX_OUT 10000

#define SHOOT_MOTOR_PID_KP 25
#define SHOOT_MOTOR_PID_KI 0
#define SHOOT_MOTOR_PID_KD 0
#define SHOOT_MOTOR_PID_MAX_IOUT 0
#define SHOOT_MOTOR_PID_MAX_OUT 10000

#define FRICTION_MOTOR_SPEED_SET 8000

#define SHOOT_MOTOR_SPEED_SET 4500 					//������Ƶ
#define SHOOT_MOTOR_SINGLE_SPEED_SET 5000 	//������Ƶ
//������������

typedef enum
{
	SHOOT_STOP,
	SHOOT_READY,
	SHOOT_NORMAL,
	SHOOT_REVERSAL
}Shoot_Mode;

typedef enum
{
	SHOOT_DARTLE,
	SHOOT_SINGLE
}Shoot_Fire_Mode;



typedef struct
{
	Motor_Feedback_t FR_motor_fdb[2];
	Motor_Feedback_t ST_motor_fdb;
	Motor_Feedback_t Test_Yaw_fdb;
	Motor_Set_t      FR_motor_set[2];
	Motor_Set_t			 ST_motor_set;
	
	int16_t 				 FR_speed;
	int16_t 				 ST_speed;
	
	RC_ctrl_t 			 rc_data;
	
	uint8_t         Shoot_Is_Ok;
	Shoot_Mode       shoot_mode;
	Shoot_Fire_Mode  shoot_fire_mode;
	uint8_t					 Change_Shoot_Mode_Flag;
	uint32_t 				 sht_time;
	Referee 				 referee;
	uint32_t reversal_detect_time;	//����תʱ��
	uint32_t reversal_time;
	uint32_t no_reversal_flag;	//��תʱ��
	uint8_t FR_state;
	uint8_t new_rc_FR_state;
	uint8_t new_rc_RE_FR_state;
    uint8_t new_rc_ST_state;
}Shoot_t;
extern Shoot_t shoot;
typedef struct
{
float mean_sum;
float mean;
float var_sum;
float s_var;
float var;
float data;
float dwt_time;
uint32_t num;
}Var_Mean;
typedef struct
{
 float a;
 float m;
 float d;
 float Shoot_Time;
 float shoot_time;
 float shoot_fre;
}shoot_heat_limit;

extern void shoot_task(void const *pvParameters);

#endif
