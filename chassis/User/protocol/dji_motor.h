/***********************************************
 * @file dji_motor.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-26
 *
 * @copyright Copyright (c) VGD 2025
 *
 *************************************************/

#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

/* ================================================================ include ================================================================ */

#include "can.h"
#include "struct_typedef.h"

/* ================================================================ define ================================================================ */

#define M3508_TX_ID_1 0x200
#define M3508_TX_ID_2 0x1FF

#ifndef PI
#define PI 3.141592653589793f
#endif

#define DJI_MOTOR_ANGLE(rxData) ((float)(rxData)->angle / 8191.0f * 2 * PI)
#define DJI_MOTOR_SPEED(rxData) ((float)(rxData)->speed / 60.0f * 2 * PI / (268.0f / 17.0f))
#define DJI_MOTOR_CURRENT(rxData) ((rxData)->current)
#define DJI_MOTOR_TEMP(rxData) ((rxData)->temp)

#define HEXROLL_TORQUE_TO_CURRENT(torque) (int16_t)(torque * 3554.3214161749397f)
#define HEXROLL_CURRENT_TO_TORQUE(curren) ((float)curren * 0.00030059168198529415f)

/* ================================================================ struct ================================================================ */

typedef struct M3508_RxData_Def_t
{
    int32_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temp;
} DJI_RxData_Def_t;

// typedef struct DJI_Motor_Def_t
// {
//     CAN_HandleTypeDef *hcan;

// } DJI_Motor_Def_t;

/* ================================================================ value ================================================================ */

/* ================================================================ proto ================================================================ */

void DJI_Motor_Transmit(CAN_HandleTypeDef *hcan, uint32_t tx_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

void DJI_Motor_Receive(DJI_RxData_Def_t *rxData, uint8_t *data);

#endif
