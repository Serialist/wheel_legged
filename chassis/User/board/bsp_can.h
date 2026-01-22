#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "struct_typedef.h"
#include "chassismotor.h"
#include "dji_motor.h"

#define M3508_WHELL_ID_1 0x206
#define M3508_WHELL_ID_2 0x207

extern DJI_RxData_Def_t m3508[2];

void can_filter_init(void);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void TransmitChassisMotorCurrent(int16_t o1, int16_t o2);

void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);

#endif
