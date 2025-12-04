/************************
 * @file c2g.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief chassis to gimbal 云底通信
 * @version 0.1.0
 * @date 2025-12-04
 *
 * @copyright Copyright (c) Serialist 2025
 *
 * @note
 * 发送301，接收302
 ************************/

/* ================================================================ include ================================================================*/

#include "c2g.h"
#include "can.h"

/* ================================================================ macro ================================================================*/

/* ================================================================ typedef ================================================================*/

/* ================================================================ variable ================================================================*/

/* ================================================================ prototype ================================================================*/

/* ================================================================ function ================================================================*/

/************************
 * @brief decode
 *
 ************************/
void C2G_Decode(struct C2G_Data *data, uint8_t *rxbuf)
{
    data->vx = rxbuf[0] << 8 | rxbuf[1];
    data->vyaw = rxbuf[2] << 8 | rxbuf[3];
}

/************************
 * @brief transmit
 *
 ************************/
void C2G_Transmit(struct DJI_RxData *motor)
{
    uint8_t buf[8];
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;

    header.StdId = 0x301;
    header.DLC = 0x08;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;

    buf[0] = motor->angle >> 8;
    buf[1] = motor->angle & 0xFF;
    buf[2] = motor->speed >> 8;
    buf[3] = motor->speed & 0xFF;
    buf[4] = motor->current >> 8;
    buf[5] = motor->current & 0xFF;
    buf[6] = motor->temp;
    buf[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &header, buf, &mailbox);
}
