/***********************************************
 * @file bsp_rc.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief 遥控器数据 接收
 * @version 0.1.0
 * @date 2025-10-30
 *
 * @copyright Copyright (c) VGD 2025
 *
 *************************************************/

#ifndef __BSP_RC_H
#define __BSP_RC_H

#include "struct_typedef.h"

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);

#endif
