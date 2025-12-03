/***********************************************
 * @file motor.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-26
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 *************************************************/

/* ================================================================ include ================================================================ */

#include "dji_motor.h"

/* ================================================================ define ================================================================ */

/* ================================================================ struct ================================================================ */

/* ================================================================ value ================================================================ */

/* ================================================================ proto ================================================================ */

/* ================================================================ function ================================================================ */

void DJI_Motor_Transmit(CAN_HandleTypeDef *hcan, uint32_t tx_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    uint8_t buf[8] = {0};

    uint32_t txMailbox;

    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = tx_id;
    txHeader.DLC = 0x08;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;

    buf[0] = current1 >> 8;
    buf[1] = current1;
    buf[2] = current2 >> 8;
    buf[3] = current2;
    buf[4] = current3 >> 8;
    buf[5] = current3;
    buf[6] = current4 >> 8;
    buf[7] = current4;

    HAL_CAN_AddTxMessage(hcan, &txHeader, buf, &txMailbox);
}

void DJI_Motor_Receive(struct DJI_RxData *rxData, uint8_t *data)
{
    rxData->angle = (int16_t)((data[0] << 8) | data[1]);
    rxData->speed = (int16_t)((data[2] << 8) | data[3]);
    rxData->current = (int16_t)((data[4] << 8) | data[5]);
    rxData->temp = data[6];
}
