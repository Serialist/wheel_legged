#include "chassis_control.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;


HAL_StatusTypeDef usart6_t_d;
void TransmitHalfWordToChassis(int16_t data)
{
	Half_Word_Transform_u temp;
	
	temp.data_raw = data;
	
	//关闭中断，防止接收冲突
	//HAL_NVIC_DisableIRQ(USART1_IRQn);
	
	
	HAL_UART_Transmit(&huart1, &temp.data[0], 1, 100);
	HAL_UART_Transmit(&huart1, &temp.data[1], 1, 100);
	
	usart6_t_d = HAL_UART_Transmit(&huart6, &temp.data[0], 1, 100);
	HAL_UART_Transmit(&huart6, &temp.data[1], 1, 100);
	
	
	//开启接收中断
	//HAL_NVIC_EnableIRQ(USART1_IRQn);

}

float MAX_X_Y_Speed_Set(float speed,float max)
{
	if(speed>max)
	{
		speed=max;
		return speed;
	}
	else 
		return speed; 
}
float MIN_X_Y_Speed_Set(float speed,float min)
{
	if(speed<min)
	{
		speed=min;
		return speed;
	}
	else 
		return speed; 
}
