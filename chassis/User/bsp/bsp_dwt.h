/************************
 * @file bsp_dwt.c
 * @author Wang Hongxi
 * @brief
 * @version 1.1.0
 * @date 2022-03-08
 *
 * @copyright Copyright (c) Serialist 2025
 *
 * @note
 * file has been refactored by Serialist (ba3pt@chd.edu.cn)
 *
 * DWT是stm32内部的一个"隐藏资源",他的用途是：
 * - 给下载器提供准确的定时
 * - 从而为调试信息加上时间戳
 * - 在固定的时间间隔将调试数据发送到你的xxlink上
 ************************/

#ifndef __BSP_DWT_H
#define __BSP_DWT_H

/* ================================================================ include ================================================================ */

#include "main.h"
#include "stdint.h"

/* ================================================================ macro ================================================================ */

/* ================================================================ typedef ================================================================ */

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/* ================================================================ variable ================================================================ */

extern DWT_Time_t SysTime;

/* ================================================================ prototype ================================================================ */

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

/* ================================================================ function ================================================================ */

#endif
