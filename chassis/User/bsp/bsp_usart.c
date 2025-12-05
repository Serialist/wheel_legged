#include "bsp_usart.h"
#include "usart.h"

#include "cmsis_os.h"

//机器人功率数据反馈
static Power_Heat_Data_t power_data_fdb;
static uint32_t power_data_fdb_time;
//机器人状态数据反馈
static Game_Robot_Status_t status_fdb;
static uint32_t status_fdb_time;


static uint8_t statusDataTranslate(uint8_t res, Game_Robot_Status_t* status);
static uint8_t powerDataTranslate(uint8_t res, Power_Heat_Data_t* power);

int usart6_d = 0;
int cnt_d = 0;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart6)
//	{
//		usart6_d++;
//		
//		if(huart6.Instance->SR & UART_FLAG_RXNE)
//		{
//			static Frame_u frame_u;
//			static uint8_t frame_index = 0;
//			uint8_t Res = huart6.Instance->DR;
//			
//			switch(frame_index)
//			{
//				case 0:
//					if(Res == 0xA5)
//						frame_u.temp_data[frame_index++] = Res;
//					break;
//				case 7:
//					if(frame_u.real_data.cmd_id == 0x201){
//						if(statusDataTranslate(Res, &status_fdb) == 1)
//						{
//							frame_index = 0;
//							status_fdb_time = xTaskGetTickCount();
//						}
//					}
//					else if(frame_u.real_data.cmd_id == 0x202){
//						if(powerDataTranslate(Res, &power_data_fdb) == 1)
//						{
//							frame_index = 0;
//							power_data_fdb_time = xTaskGetTickCount();
//						}
//							
//					}
//					else
//						frame_index = 0;
//					break;
//				default:
//					frame_u.temp_data[frame_index++] = Res;
//					break;
//			}
//			cnt_d = frame_index;
//		} 
//		__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
//	}
//}

void USART6_IRQHandler(void)
{
  usart6_d++;
		
		if(huart6.Instance->SR & UART_FLAG_RXNE)
		{
			static Frame_u frame_u;
			static uint8_t frame_index = 0;
			uint8_t Res = huart6.Instance->DR;
			
			switch(frame_index)
			{
				case 0:
					if(Res == 0xA5)
						frame_u.temp_data[frame_index++] = Res;
					break;
				case 7:
					if(frame_u.real_data.cmd_id == 0x201){
						if(statusDataTranslate(Res, &status_fdb) == 1)
						{
							frame_index = 0;
							status_fdb_time = xTaskGetTickCount();
						}
					}
					else if(frame_u.real_data.cmd_id == 0x202){
						if(powerDataTranslate(Res, &power_data_fdb) == 1)
						{
							frame_index = 0;
							power_data_fdb_time = xTaskGetTickCount();
						}
							
					}
					else
						frame_index = 0;
					break;
				default:
					frame_u.temp_data[frame_index++] = Res;
					break;
			}
			cnt_d = frame_index;
		} 
		__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
}

static uint8_t powerDataTranslate(uint8_t res, Power_Heat_Data_t* power)
{
	static int power_cnt = 0;
	static Power_u power_u;
	
	power_u.temp_data[power_cnt++] = res;
	if(power_cnt == 16)
	{
		power_cnt = 0;
		
		power->chassis_volt_fdb 		= power_u.real_data.chassis_volt_fdb; 
		power->chassis_current_fdb 		= power_u.real_data.chassis_current_fdb;
		power->chassis_power_fdb 		= power_u.real_data.chassis_power_fdb; 
		power->chassis_power_buffer_fdb = power_u.real_data.chassis_power_buffer_fdb; 
		power->shooter_id1_17mm_cooling_heat = power_u.real_data.shooter_id1_17mm_cooling_heat; 
		power->shooter_id2_17mm_cooling_heat = power_u.real_data.shooter_id2_17mm_cooling_heat; 
		power->shooter_id1_42mm_cooling_heat = power_u.real_data.shooter_id1_42mm_cooling_heat;
		
		return 1;	//一组数据接收完成
	}
	return 0;	//数据接收未完成
}

static uint8_t statusDataTranslate(uint8_t res, Game_Robot_Status_t* status)
{
	static int status_cnt = 0;
	static Status_u status_u;
	
	status_u.temp_data[status_cnt++] = res;
	
	if(status_cnt == 27)
	{
		status_cnt = 0;
		
//		status->robot_id						= status_u.real_data.robot_id;
//		status->robot_level						= status_u.real_data.robot_level;
//		status->remain_HP						= status_u.real_data.remain_HP;
//		status->max_HP							= status_u.real_data.max_HP;
//		status->shooter_id1_17mm_cooling_rate	= status_u.real_data.shooter_id1_17mm_cooling_rate;
//		status->shooter_id1_17mm_cooling_limit	= status_u.real_data.shooter_id1_17mm_cooling_limit;
//		status->shooter_id1_17mm_speed_limit	= status_u.real_data.shooter_id1_17mm_speed_limit;
//		status->shooter_id2_17mm_cooling_rate	= status_u.real_data.shooter_id2_17mm_cooling_rate;
//		status->shooter_id2_17mm_cooling_limit	= status_u.real_data.shooter_id2_17mm_cooling_limit;
//		status->shooter_id2_17mm_speed_limit	= status_u.real_data.shooter_id2_17mm_speed_limit;
//		status->shooter_id1_42mm_cooling_rate	= status_u.real_data.shooter_id1_42mm_cooling_rate;
//		status->shooter_id1_42mm_cooling_limit	= status_u.real_data.shooter_id1_42mm_cooling_limit;
//		status->shooter_id1_42mm_speed_limit	= status_u.real_data.shooter_id1_42mm_speed_limit;
//		status->chassis_power_limit				= status_u.real_data.chassis_power_limit;
//		status->mains_power_gimbal_output		= status_u.real_data.mains_power_gimbal_output;
//		status->mains_power_chassis_output		= status_u.real_data.mains_power_chassis_output;
//		status->mains_power_shooter_output		= status_u.real_data.mains_power_shooter_output;
		
		status->robot_id						= status_u.temp_data[0];
		status->robot_level						= status_u.temp_data[1];
		status->remain_HP						= (status_u.temp_data[3] << 8) | status_u.temp_data[2]; 
		status->max_HP							= (status_u.temp_data[5] << 8) | status_u.temp_data[4]; 
		status->shooter_id1_17mm_cooling_rate	= (status_u.temp_data[7] << 8) | status_u.temp_data[6]; 
		status->shooter_id1_17mm_cooling_limit	= (status_u.temp_data[9] << 8) | status_u.temp_data[8]; 
		status->shooter_id1_17mm_speed_limit	= (status_u.temp_data[10] << 8) | status_u.temp_data[11]; 
		status->shooter_id2_17mm_cooling_rate	= (status_u.temp_data[12] << 8) | status_u.temp_data[13]; 
		status->shooter_id2_17mm_cooling_limit	= (status_u.temp_data[14] << 8) | status_u.temp_data[15]; 
		status->shooter_id2_17mm_speed_limit	= (status_u.temp_data[16] << 8) | status_u.temp_data[17]; 
		status->shooter_id1_42mm_cooling_rate	= (status_u.temp_data[18] << 8) | status_u.temp_data[19]; 
		status->shooter_id1_42mm_cooling_limit	= (status_u.temp_data[20] << 8) | status_u.temp_data[21]; 
		status->shooter_id1_42mm_speed_limit	= (status_u.temp_data[22] << 8) | status_u.temp_data[23]; 
		status->chassis_power_limit				= (status_u.temp_data[25] << 8) | status_u.temp_data[24]; 
//		status->mains_power_gimbal_output		= status_u.temp_data[26];
//		status->mains_power_chassis_output		= status_u.temp_data[24];
//		status->mains_power_shooter_output		= status_u.temp_data[24];
		
		return 1;	//一组数据接收完成
	}
	return 0;	//数据接收未完成
}

uint32_t GetRefereeSystemPowerData(Power_Heat_Data_t* data)
{
	*data = power_data_fdb;
	return power_data_fdb_time;
}

uint32_t GetRefereeSystemStatusData(Game_Robot_Status_t* data)
{
	*data = status_fdb;
	return status_fdb_time;
}

