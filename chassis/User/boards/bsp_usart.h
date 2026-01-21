#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "referee_system.h"


uint32_t GetRefereeSystemPowerData(Power_Heat_Data_t* data);
uint32_t GetRefereeSystemStatusData(Game_Robot_Status_t* data);

#endif

