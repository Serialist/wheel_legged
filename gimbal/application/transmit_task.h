#ifndef __TRANSMIT_TASK_H
#define __TRANSMIT_TASK_H
#include "FreeRTOS.h"
#include "cmsis_os.h"
extern xQueueHandle CAN1_data_Queue;
extern xQueueHandle CAN2_data_Queue1;
extern xQueueHandle CAN2_data_Queue2;
//extern xQueueHandle CAN2_data_Queue3;
typedef struct
{
	uint32_t stdid;
	uint8_t data[8];
}CAN1_Data_t;

typedef struct
{
	uint32_t stdid;
	uint8_t data[8];
}CAN2_Data_t;



void transmit_Task( void const*pvParameters );
void transmit2_Task( void const*pvParameters );




#endif
