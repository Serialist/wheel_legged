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

extern Wheel_Leg_Target_t set;
extern Chassis_t chassis;

AK_motor_fdb_t AK_motor[6];
AK_motor_ctrl_fdb_t motorAK10[6];

struct Motor_AK_Rx_Data ak10[4];
DJI_RxData_Def_t m3508[2];

uint8_t can_rx_data[8];
void can_filter_init(void)
{

	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_message;
	if (hcan == &hcan1)
	{
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_message, can_rx_data) == HAL_OK) // 获得接收到的数据头和数据
		{
			/**
			 * @brief M3508 轮毂 接收
			 * @date 2025-10-30
			 */
			/******** M3508 ********/
			if (rx_message.StdId == M3508_WHELL_ID_1)
			{
				DJI_Motor_Receive(&m3508[0], can_rx_data);
				motor_status.receive_flag[4] = true;
			}
			else if (rx_message.StdId == M3508_WHELL_ID_2)
			{
				DJI_Motor_Receive(&m3508[1], can_rx_data);
				motor_status.receive_flag[5] = true;
			}
			else if (rx_message.StdId == 0x01)
			{
				Motor_AK_MIT_Decode(&ak10[0], can_rx_data, P_MIN, V_MAX, T_MAX);
				/////////////////////////////////////////////////////////////////

				int id = can_rx_data[0]; // 驱动 ID 号
				int p_int = (can_rx_data[1] << 8) | (can_rx_data[2]);
				int v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);
				int i_int = ((can_rx_data[4] & 0xF) << 8) | (can_rx_data[5]);
				int T_int = can_rx_data[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 1)
				{
					motorAK10[id - 1].angle = p;
					motorAK10[id - 1].motor_ctrlspd = v;
					motorAK10[id - 1].motor_ctrltor = i;
					motorAK10[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}
			else if (rx_message.StdId == 0x02)
			{
				Motor_AK_MIT_Decode(&ak10[1], can_rx_data, P_MIN, V_MAX, T_MAX);
				/////////////////////////////////////////////////////////////////

				int id = can_rx_data[0]; // 驱动 ID 号
				int p_int = (can_rx_data[1] << 8) | (can_rx_data[2]);
				int v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);
				int i_int = ((can_rx_data[4] & 0xF) << 8) | (can_rx_data[5]);
				int T_int = can_rx_data[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 2)
				{
					motorAK10[id - 1].angle = p;
					motorAK10[id - 1].motor_ctrlspd = v;
					motorAK10[id - 1].motor_ctrltor = i;
					motorAK10[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}
			else if (rx_message.StdId == 0x03)
			{
				Motor_AK_MIT_Decode(&ak10[2], can_rx_data, P_MIN, V_MAX, T_MAX);
				ak10[2].position = -ak10[2].position;
				ak10[2].speed = -ak10[2].speed;
				/////////////////////////////////////////////////////////////////

				int id = can_rx_data[0]; // 驱动 ID 号
				int p_int = (can_rx_data[1] << 8) | (can_rx_data[2]);
				int v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);
				int i_int = ((can_rx_data[4] & 0xF) << 8) | (can_rx_data[5]);
				int T_int = can_rx_data[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 3)
				{
					motorAK10[id - 1].angle = -p;
					motorAK10[id - 1].motor_ctrlspd = -v;
					motorAK10[id - 1].motor_ctrltor = i;
					motorAK10[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}
			else if (rx_message.StdId == 0x04)
			{
				Motor_AK_MIT_Decode(&ak10[3], can_rx_data, P_MIN, V_MAX, T_MAX);
				ak10[3].position = -ak10[3].position;
				ak10[3].speed = -ak10[3].speed;
				/////////////////////////////////////////////////////////////////

				int id = can_rx_data[0]; // 驱动 ID 号
				int p_int = (can_rx_data[1] << 8) | (can_rx_data[2]);
				int v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);
				int i_int = ((can_rx_data[4] & 0xF) << 8) | (can_rx_data[5]);
				int T_int = can_rx_data[6];
				float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
				float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
				float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
				float Temp = T_int;
				if (id == 4)
				{
					motorAK10[id - 1].angle = -p;
					motorAK10[id - 1].motor_ctrlspd = -v;
					motorAK10[id - 1].motor_ctrltor = i;
					motorAK10[id - 1].motor_ctrltemp = Temp - 40;
					motor_status.receive_flag[id - 1] = true;
				}
			}
		}
	}
	else if (hcan == &hcan2)
	{
		if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_message, can_rx_data) == HAL_OK) // 获得接收到的数据头和数据
		{
		}
	}
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
