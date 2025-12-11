/************************
 * @file bsp_can.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-30
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "bsp_can.h"
#include "can.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "data_transform.h"
#include "motor.h"

extern struct Wheel_Leg_Target set;
extern struct Chassis_State chassis;

struct DJI_RxData yaw_data;
MOTOR_TRANSMIT_DATA gimbal_motor_set;

AK_motor_fdb_t AK_motor[6];
AK_motor_ctrl_fdb_t ak_motion[6];

struct Motor_AK_Rx_Data ak10[4];
struct DJI_RxData m3508[2];

static fp32 chassis_speed_set[2];

uint8_t canRxBuf[8];

/// @brief CAN 回调函数注册结构体
/// @date 2025-10-30
Reg_Def_t can_callback_reg;

void CAN_Bsp_Init(void)
{
	CAN_FilterTypeDef filter;

	filter.FilterActivation = ENABLE;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;

	HAL_CAN_ConfigFilter(&hcan1, &filter);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	filter.SlaveStartFilterBank = 14;
	filter.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &filter);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

	/// @date 2025-10-30
	// Reg_Init(&can_callback_reg);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_message;
	if (hcan == &hcan1)
	{
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_message, canRxBuf) == HAL_OK) // 获得接收到的数据头和数据
		{
			if (rx_message.StdId == YAW_ID)
			{
				yaw_data.angle = (canRxBuf[0] << 8) | canRxBuf[1];
				yaw_data.speed = (canRxBuf[2] << 8) | canRxBuf[3];
				yaw_data.current = (canRxBuf[4] << 8) | canRxBuf[5];
				yaw_data.temp = canRxBuf[6];
			}
			/**
			 * @brief M3508 轮毂 接收
			 * @date 2025-10-30
			 */
			/******** M3508 ********/
			else if (rx_message.StdId == M3508_WHELL_ID_1)
			{
				DJI_Motor_Receive(&m3508[0], canRxBuf);
				motor_status.receive_flag[4] = true;
			}
			else if (rx_message.StdId == M3508_WHELL_ID_2)
			{
				DJI_Motor_Receive(&m3508[1], canRxBuf);
				motor_status.receive_flag[5] = true;
			}
			/******** / M3508 ********/
			else if (rx_message.ExtId == 0x2967)
			{
				int16_t pos_int = (canRxBuf[0] << 8) | (canRxBuf[1]);
				int16_t spd_int = (canRxBuf[2] << 8) | (canRxBuf[3]);
				int16_t cur_int = (canRxBuf[4] << 8) | (canRxBuf[5]);
				AK_motor[2].motor_pos = (float)(pos_int * 0.1f);  // 电机位置
				AK_motor[2].motor_spd = (float)(spd_int * 10.0f); // 电机速度
				AK_motor[2].motor_cur = (float)(cur_int * 0.01f); // 电机电流
				AK_motor[2].motor_temp = canRxBuf[6];			  // 电机温度
				AK_motor[2].motor_error = canRxBuf[7];			  // 电机故障码
			}
			else if (rx_message.ExtId == 0x2968)
			{
				int16_t pos_int = (canRxBuf[0] << 8) | (canRxBuf[1]);
				int16_t spd_int = (canRxBuf[2] << 8) | (canRxBuf[3]);
				int16_t cur_int = (canRxBuf[4] << 8) | (canRxBuf[5]);
				AK_motor[3].motor_pos = (float)(pos_int * 0.1f);  // 电机位置
				AK_motor[3].motor_spd = (float)(spd_int * 10.0f); // 电机速度
				AK_motor[3].motor_cur = (float)(cur_int * 0.01f); // 电机电流
				AK_motor[3].motor_temp = canRxBuf[6];			  // 电机温度
				AK_motor[3].motor_error = canRxBuf[7];			  // 电机故障码
			}
			else if (rx_message.ExtId == 0x2966)
			{
				int16_t pos_int = (canRxBuf[0] << 8) | (canRxBuf[1]);
				int16_t spd_int = (canRxBuf[2] << 8) | (canRxBuf[3]);
				int16_t cur_int = (canRxBuf[4] << 8) | (canRxBuf[5]);
				AK_motor[1].motor_pos = (float)(pos_int * 0.1f);  // 电机位置
				AK_motor[1].motor_spd = (float)(spd_int * 10.0f); // 电机速度
				AK_motor[1].motor_cur = (float)(cur_int * 0.01f); // 电机电流
				AK_motor[1].motor_temp = canRxBuf[6];			  // 电机温度
				AK_motor[1].motor_error = canRxBuf[7];			  // 电机故障码
			}
			else if (rx_message.ExtId == 0x2965)
			{
				int16_t pos_int = (canRxBuf[0] << 8) | (canRxBuf[1]);
				int16_t spd_int = (canRxBuf[2] << 8) | (canRxBuf[3]);
				int16_t cur_int = (canRxBuf[4] << 8) | (canRxBuf[5]);
				AK_motor[0].motor_pos = (float)(pos_int * 0.1f);  // 电机位置
				AK_motor[0].motor_spd = (float)(spd_int * 10.0f); // 电机速度
				AK_motor[0].motor_cur = (float)(cur_int * 0.01f); // 电机电流
				AK_motor[0].motor_temp = canRxBuf[6];			  // 电机温度
				AK_motor[0].motor_error = canRxBuf[7];			  // 电机故障
			}
			else if (rx_message.ExtId == 0x2964)
			{
				int16_t pos_int = (canRxBuf[0] << 8) | (canRxBuf[1]);
				int16_t spd_int = (canRxBuf[2] << 8) | (canRxBuf[3]);
				int16_t cur_int = (canRxBuf[4] << 8) | (canRxBuf[5]);
				AK_motor[4].motor_pos = (float)(pos_int * 0.1f);  // 电机位置
				AK_motor[4].motor_spd = (float)(spd_int * 10.0f); // 电机速度
				AK_motor[4].motor_cur = (float)(cur_int * 0.01f); // 电机电流
				AK_motor[4].motor_temp = canRxBuf[6];			  // 电机温度
				AK_motor[4].motor_error = canRxBuf[7];			  // 电机故障码
			}
			else if (rx_message.ExtId == 0x2963)
			{
				int16_t pos_int = (canRxBuf[0] << 8) | (canRxBuf[1]);
				int16_t spd_int = (canRxBuf[2] << 8) | (canRxBuf[3]);
				int16_t cur_int = (canRxBuf[4] << 8) | (canRxBuf[5]);
				AK_motor[5].motor_pos = (float)(pos_int * 0.1f);  // 电机位置
				AK_motor[5].motor_spd = (float)(spd_int * 10.0f); // 电机速度
				AK_motor[5].motor_cur = (float)(cur_int * 0.01f); // 电机电流
				AK_motor[5].motor_temp = canRxBuf[6];			  // 电机温度
				AK_motor[5].motor_error = canRxBuf[7];			  // 电机故障码
			}
			else if (rx_message.StdId == 0x05)
			{

				int id = canRxBuf[0]; // 驱动 ID 号
				int p_int = (canRxBuf[1] << 8) | (canRxBuf[2]);
				int v_int = (canRxBuf[3] << 4) | (canRxBuf[4] >> 4);
				int i_int = ((canRxBuf[4] & 0xF) << 8) | (canRxBuf[5]);
				int T_int = canRxBuf[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, -V_MAX_2, V_MAX_2, 12);
				float i = uint_to_float(i_int, -T_MAX_2, T_MAX_2, 12);
				float Temp = T_int;
				if (id == 5)
				{
					ak_motion[id - 1].motor_ctrlpos = -p;
					ak_motion[id - 1].motor_ctrlspd = -v;
					ak_motion[id - 1].motor_ctrltor = i;
					ak_motion[id - 1].motor_ctrltemp = Temp - 40;
				}
			}
			else if (rx_message.StdId == 0x06)
			{

				int id = canRxBuf[0]; // 驱动 ID 号
				int p_int = (canRxBuf[1] << 8) | (canRxBuf[2]);
				int v_int = (canRxBuf[3] << 4) | (canRxBuf[4] >> 4);
				int i_int = ((canRxBuf[4] & 0xF) << 8) | (canRxBuf[5]);
				int T_int = canRxBuf[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, -V_MAX_2, V_MAX_2, 12);
				float i = uint_to_float(i_int, -T_MAX_2, T_MAX_2, 12);
				float Temp = T_int;
				if (id == 6)
				{
					ak_motion[id - 1].motor_ctrlpos = p;
					ak_motion[id - 1].motor_ctrlspd = v;
					ak_motion[id - 1].motor_ctrltor = i;
					ak_motion[id - 1].motor_ctrltemp = Temp - 40;
				}
			}

			else if (rx_message.StdId == 0x01)
			{
				/////////////////////////////////////////////////////////////////
				Motor_AK_MIT_Decode(&ak10[0], canRxBuf, P_MIN, V_MAX, T_MAX);
				/////////////////////////////////////////////////////////////////

				int id = canRxBuf[0]; // 驱动 ID 号
				int p_int = (canRxBuf[1] << 8) | (canRxBuf[2]);
				int v_int = (canRxBuf[3] << 4) | (canRxBuf[4] >> 4);
				int i_int = ((canRxBuf[4] & 0xF) << 8) | (canRxBuf[5]);
				int T_int = canRxBuf[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 1)
				{
					ak_motion[id - 1].motor_ctrlpos = p;
					ak_motion[id - 1].motor_ctrlspd = v;
					ak_motion[id - 1].motor_ctrltor = i;
					ak_motion[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}
			else if (rx_message.StdId == 0x02)
			{
				/////////////////////////////////////////////////////////////////
				Motor_AK_MIT_Decode(&ak10[1], canRxBuf, P_MIN, V_MAX, T_MAX);
				/////////////////////////////////////////////////////////////////

				int id = canRxBuf[0]; // 驱动 ID 号
				int p_int = (canRxBuf[1] << 8) | (canRxBuf[2]);
				int v_int = (canRxBuf[3] << 4) | (canRxBuf[4] >> 4);
				int i_int = ((canRxBuf[4] & 0xF) << 8) | (canRxBuf[5]);
				int T_int = canRxBuf[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 2)
				{
					ak_motion[id - 1].motor_ctrlpos = p;
					ak_motion[id - 1].motor_ctrlspd = v;
					ak_motion[id - 1].motor_ctrltor = i;
					ak_motion[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}
			else if (rx_message.StdId == 0x03)
			{
				/////////////////////////////////////////////////////////////////
				Motor_AK_MIT_Decode(&ak10[2], canRxBuf, P_MIN, V_MAX, T_MAX);
				ak10[2].position = -ak10[2].position;
				ak10[2].speed = -ak10[2].speed;
				/////////////////////////////////////////////////////////////////

				int id = canRxBuf[0]; // 驱动 ID 号
				int p_int = (canRxBuf[1] << 8) | (canRxBuf[2]);
				int v_int = (canRxBuf[3] << 4) | (canRxBuf[4] >> 4);
				int i_int = ((canRxBuf[4] & 0xF) << 8) | (canRxBuf[5]);
				int T_int = canRxBuf[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 3)
				{
					ak_motion[id - 1].motor_ctrlpos = -p;
					ak_motion[id - 1].motor_ctrlspd = -v;
					ak_motion[id - 1].motor_ctrltor = i;
					ak_motion[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}
			else if (rx_message.StdId == 0x04)
			{
				/////////////////////////////////////////////////////////////////
				Motor_AK_MIT_Decode(&ak10[3], canRxBuf, P_MIN, V_MAX, T_MAX);
				ak10[3].position = -ak10[3].position;
				ak10[3].speed = -ak10[3].speed;
				/////////////////////////////////////////////////////////////////

				int id = canRxBuf[0]; // 驱动 ID 号
				int p_int = (canRxBuf[1] << 8) | (canRxBuf[2]);
				int v_int = (canRxBuf[3] << 4) | (canRxBuf[4] >> 4);
				int i_int = ((canRxBuf[4] & 0xF) << 8) | (canRxBuf[5]);
				int T_int = canRxBuf[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 4)
				{
					ak_motion[id - 1].motor_ctrlpos = -p;
					ak_motion[id - 1].motor_ctrlspd = -v;
					ak_motion[id - 1].motor_ctrltor = i;
					ak_motion[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}

			// Reg_Run(&can_callback_reg, rx_message.StdId, canRxBuf);
		}
	}
	else if (hcan == &hcan2)
	{

		Float_Transform_u float_temp;
		if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_message, canRxBuf) == HAL_OK) // 获得接收到的数据头和数据
		{

			if (rx_message.StdId == CHASSIS_Y_ID)
			{
				// chassis_speed_set_time[CHASSIS_Y] = xTaskGetTickCount();
				float_temp.data[0] = canRxBuf[4];
				float_temp.data[1] = canRxBuf[5];
				float_temp.data[2] = canRxBuf[6];
				float_temp.data[3] = canRxBuf[7];
				chassis_speed_set[CHASSIS_Y] = float_temp.data_raw;
			}
			else if (rx_message.StdId == CHASSIS_Z_ID)
			{
				// chassis_speed_set_time[CHASSIS_Z] = xTaskGetTickCount();
				float_temp.data[0] = canRxBuf[0];
				float_temp.data[1] = canRxBuf[1];
				float_temp.data[2] = canRxBuf[2];
				float_temp.data[3] = canRxBuf[3];
				chassis_speed_set[CHASSIS_Z] = float_temp.data_raw;
			}
			else if (rx_message.StdId == GIMBAL_TO_CHASSIS)
			{
				gimbal_motor_set.motor_current_set = (canRxBuf[0] << 8) | canRxBuf[1];
				set.mode_state = canRxBuf[2];
				set.rotate_state = canRxBuf[3];
				set.shoot_state = canRxBuf[4];
			}
		}
	}
}

void GetChassisYSpeedSet(fp32 *set)
{
	*set = chassis_speed_set[CHASSIS_Y];
}

void GetChassisZSpeedSet(fp32 *set)
{
	*set = chassis_speed_set[CHASSIS_Z];
}

void GetAKMotor1Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2)
{
	*motor1 = AK_motor[0];
	*motor2 = ak_motion[0];
}
void GetAKMotor2Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2)
{
	*motor1 = AK_motor[1];
	*motor2 = ak_motion[1];
}
void GetAKMotor3Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2)
{
	*motor1 = AK_motor[2];
	*motor2 = ak_motion[2];
}
void GetAKMotor4Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2)
{
	*motor1 = AK_motor[3];
	*motor2 = ak_motion[3];
}
void GetAKMotor5Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2)
{
	*motor1 = AK_motor[4];
	*motor2 = ak_motion[4];
}
void GetAKMotor6Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2)
{
	*motor1 = AK_motor[5];
	*motor2 = ak_motion[5];
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/***********************************************
 **                                           **
 **  ooooooooo.   oooooooooooo   .oooooo.     **
 **  `888   `Y88. `888'     `8  d8P'  `Y8b    **
 **   888   .d88'  888         888            **
 **   888ooo88P'   888oooo8    888            **
 **   888`88b.     888    "    888     ooooo  **
 **   888  `88b.   888       o `88.    .88'   **
 **  o888o  o888o o888ooooood8  `Y8bood8P'    **
 **                                           **
 ***********************************************/

/************************
 * @brief 初始化 注册
 *
 * @param reg
 ************************/
void Reg_Init(Reg_Def_t *reg)
{
	reg->len = 0;
}

/************************
 * @brief 注册函数
 *
 * @param reg
 * @param Func
 * @return uint8_t
 ************************/
uint8_t Reg_Add(Reg_Def_t *reg, void (*Func)(uint8_t flag, uint8_t *data))
{
	if (reg->len >= REGISTER_FUNC_RUN_MAX)
	{
		return 1;
	}
	else
	{
		reg->Func[reg->len] = Func;
		reg->len++;
		return 0;
	}
}

/************************
 * @brief 清空注册列表
 *
 * @param reg
 ************************/
void Reg_Clear(Reg_Def_t *reg)
{
	reg->len = 0;
}

/************************
 * @brief 注册函数运行
 *
 * @param reg
 * @param flag
 * @param data
 ************************/
void Reg_Run(Reg_Def_t *reg, uint8_t flag, uint8_t *data)
{
	for (uint8_t i = 0; i < reg->len; i++)
	{
		reg->Func[i](flag, data);
	}
}
