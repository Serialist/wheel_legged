#include "transmit_task.h"
#include "can.h"
/*  ==============================================================================
利用队列发送数据，2025赛季没用
  ============================================================================== */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void CAN1Transmit(CAN1_Data_t data);
static void CAN2Transmit(CAN2_Data_t data);
int transmit_b=0,transmit_a=0,transmit_c=0,erro=0,ttt=0;
BaseType_t sendStatus1;
BaseType_t sendStatus2;
BaseType_t sendStatus3;
void transmit_Task( void const*pvParameters)
{
	
	CAN1_Data_t sent1;
	CAN2_Data_t sent2;
	CAN2_Data_t sent3;
	
	
	for( ;; )
	{
		
		sendStatus1 = xQueueReceive(CAN1_data_Queue, &sent1, portMAX_DELAY );
		sendStatus2 = xQueueReceive(CAN2_data_Queue1, &sent2,  0 );
		sendStatus3 = xQueueReceive(CAN2_data_Queue2, &sent3,  0 );
		ttt++;
		
		//taskENTER_CRITICAL();
		if( sendStatus1 == pdPASS )
		{
			transmit_a++;
			CAN1Transmit(sent1);		
		}
//		if( sendStatus == pdPASS )
//		{
//			transmit_a++;
//			CAN2_Send(0x301,Sent.Data[0]);
//		}
//		
//		sendStatus = xQueueReceive(CAN2_data_Queue2, &Sent.Data[1],  portMAX_DELAY );
//		if( sendStatus == pdPASS )
//		{
//			transmit_b++;
//			CAN2_Send(0x302,Sent.Data[1]);
//		}
//		
//		sendStatus = xQueueReceive(CAN2_data_Queue3, &Sent.Data[2],  portMAX_DELAY );
//		if( sendStatus == pdPASS )
//		{
//			transmit_c++;
//			CAN2_Send(0x303,Sent.Data[2]);
//		}
		if( sendStatus2 == pdPASS )
		{
			transmit_b++;
			CAN2Transmit(sent2);
		}
		if( sendStatus3 == pdPASS )
		{
			CAN2Transmit(sent3);
		}	
			erro++;
		osDelay(1);
	} 
}


static void CAN1Transmit(CAN1_Data_t data)
{
	CAN_TxHeaderTypeDef  tx_message;
	uint8_t              can_send_data[8];
	
    uint32_t send_mail_box;
    tx_message.StdId = data.stdid;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = data.data[0];
    can_send_data[1] = data.data[1];
    can_send_data[2] = data.data[2];
    can_send_data[3] = data.data[3];
    can_send_data[4] = data.data[4];
    can_send_data[5] = data.data[5];
    can_send_data[6] = data.data[6];
    can_send_data[7] = data.data[7];
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}

static void CAN2Transmit(CAN2_Data_t data)
{
	CAN_TxHeaderTypeDef  tx_message;
	uint8_t              can_send_data[8];
	
    uint32_t send_mail_box;
    tx_message.StdId = data.stdid;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = data.data[0];
    can_send_data[1] = data.data[1];
    can_send_data[2] = data.data[2];
    can_send_data[3] = data.data[3];
    can_send_data[4] = data.data[4];
    can_send_data[5] = data.data[5];
    can_send_data[6] = data.data[6];
    can_send_data[7] = data.data[7];
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
}



