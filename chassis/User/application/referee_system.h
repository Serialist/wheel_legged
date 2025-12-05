#ifndef __REFEREE_SYSTEM_H
#define __REFEREE_SYSTEM_H

#include "struct_typedef.h"

typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
}Game_Robot_Status_t;

typedef __packed struct
{
	uint16_t chassis_volt_fdb; 
	uint16_t chassis_current_fdb; 
	float chassis_power_fdb; 
	uint16_t chassis_power_buffer_fdb; 
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
}Power_Heat_Data_t;

typedef __packed struct
{
	uint8_t SOF;
	uint16_t data_length;
	uint8_t seq;
	uint8_t CRC8;
	uint16_t cmd_id;
}Frame_Data_t;

typedef union{
	Power_Heat_Data_t real_data;
	char temp_data[16];
}Power_u;

typedef union{
	Game_Robot_Status_t real_data;
	char temp_data[27];
}Status_u;

typedef union{
	Frame_Data_t real_data;
	char temp_data[7];
}Frame_u;
#endif
