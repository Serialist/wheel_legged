#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

typedef struct
{
	uint32_t Stdid;
	uint8_t  Data[3][8];
}CAN_Data_t;

#define ECD_MIN 0
#define ECD_MID 4096
#define ECD_MAX 8191


//云底联动ID
#define CHASSIS_X_Y_W_ID 0x301
#define YAW_MOTOR_ID     0x305
#define CHASSIS_TO_GIMBAL 0x303
#define RC_DATA 0x306


#define ROBOT_Infantry 0xAA
#define ROBOT_Sentry 0xBB
#define ROBOT_ID ROBOT_Infantry
//2025.10.1,大底盘的叫ROBOT_Sentry，小底盘的有雷达的步改哨叫ROBOT_Infantry
//其实电机ID可以自己改，但是现在机械装了不好改了，主要还是PID参数有差别

 #define M_PI  (float)3.1415926535
	 
	#if (ROBOT_ID==ROBOT_Infantry) 
	#define PITCH_ZERO_ECD 7000
  #define YAW_ZERO_ECD   938
	#define Y_YAW_ZERO_ECD 1975
	#define ANGLE_MIN -180
	#define ANGLE_ZERO 0
	#define ANGLE_MAX 180
  //电机ID
#define PITCH_MOTOR_ID 0x208
#define FR1_MOTOR_ID 0x201	//friction motor 1 ID
#define FR2_MOTOR_ID 0x202	//friction motor 2 ID
#define SHOOT_MOTOR_ID 0x203	//shoot motor I
#elif (ROBOT_ID==ROBOT_Sentry) 
  #define PITCH_ZERO_ECD 7000
  #define YAW_ZERO_ECD   5521
	#define Y_YAW_ZERO_ECD 6545
	#define ANGLE_MIN -180
	#define ANGLE_ZERO 0
	#define ANGLE_MAX 180
  //电机ID
#define PITCH_MOTOR_ID 0x209
#define FR1_MOTOR_ID 0x201	//friction motor 1 ID
#define FR2_MOTOR_ID 0x202	//friction motor 2 ID
#define SHOOT_MOTOR_ID 0x203	//shoot motor I
#endif
typedef struct 
{     
	uint8_t quat_euler:1;     
	uint8_t gyro_rangle:3;     
	uint8_t accel_rangle:2;     
	uint8_t imu_sensor_rotation:5;     
	uint8_t ahrs_rotation_sequence:3;     
	int16_t quat[4];     
	float quat_fp32[4];     
	int16_t euler_angle[3];     
	float euler_angle_fp32[3];     
	int16_t gyro_int16[3];     
	int16_t accel_int16[3];     
	int16_t mag_int16[3];     
	float gyro_fp32[3];     
	float accel_fp32[3];     
	uint16_t sensor_time;     
	uint16_t sensor_temperature;     
	int16_t sensor_control_temperature;     
	float gyro_sen;     
	float accel_sen; 
}rm_imu_data_t;


typedef struct
{
	int16_t ecd_fdb;
    int16_t speed_rpm_fdb;
	int16_t current_fdb;
	int8_t temperate_fdb;
	int16_t torque_fdb;
}Motor_Feedback_t;

typedef struct
{
	int16_t speed_rpm_set;
	int16_t current_set;
	float current_set_f;
	float speed_rpm_set_f;
	float angle;
	float ecd_set;
}Motor_Set_t;


extern fp32 Y_yaw_motor_relative_angle_fdb;
extern void can_filter_init(void);
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_shoot(int16_t fri1, int16_t fri2, int16_t sht, int16_t rev);
extern void CAN_cmd_chassis_speed(float chassis_x, float chassis_y, float chassis_z, int super_flag);
void CAN_cmd_chassis_data_FOUR(uint8_t rotate, uint8_t cover, uint8_t vision, uint8_t fri,uint8_t shoot_mode,uint8_t reset,uint8_t direction,uint8_t superpower_state);
void CAN_chassis_data_2ed(uint8_t no_ref_Flag, uint8_t a, uint8_t b, uint8_t c,uint8_t d,uint8_t e,uint8_t f,uint8_t g);
void Sent_Shoot(int16_t fr1, int16_t fr2, int16_t sht, int16_t rev);
uint32_t GetPitchMotorFdb(Motor_Feedback_t* motor);
uint32_t GetYawMotorFdb(Motor_Feedback_t* motor);
uint32_t GetMotorPitchRelativeAngle(fp32* angle);
uint32_t GetMotorYawRelativeAngle(fp32* angle);
uint32_t GetFR1Motor(Motor_Feedback_t* motor);
uint32_t GetFR2Motor(Motor_Feedback_t* motor);
uint32_t GetSTMotor(Motor_Feedback_t* motor);
void Get_one_head_angle(fp32* One_Head_Angle);
void Get_robot_ID(uint8_t* robot_ID);
void sentAGVspeed(int16_t speed_x, int16_t speed_y, int16_t speed_w);
 void sentGimbal(int16_t pitch);//xData data
#endif
