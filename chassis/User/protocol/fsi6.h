/************************************************
 * @file fsi6.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 1.0.0
 * @date 2025-10-04
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 ********************************/

#ifndef __FSI6_H
#define __FSI6_H

/**************************************************************** include ****************************************************************/

#include "user_lib.h"
#include "bsp_dwt.h"

/**************************************************************** define ****************************************************************/

/* ================================ config ================================ */

#define RC_TIME_GET_MS(pLastCNT) (DWT_GetDeltaT(pLastCNT))
#define RC_OFFLINE 200

/* ================================ 常量 ================================ */

/// @brief 控制量定义
#define RC_MAX 1000  // 最大值
#define RC_ZERO 0    // 中值
#define RC_MIN -1000 // 最小值

#define RC_UP 2   // 开关上
#define RC_MID 1  // 开关中
#define RC_DOWN 0 // 开关下

/// @brief 索引
#define RC_LX 0
#define RC_LY 1
#define RC_RX 2
#define RC_RY 3

#define RC_A 0
#define RC_B 1
#define RC_C 2
#define RC_D 3


/* ================================ 宏定义函数 ================================ */

/// @brief 摇杆
#define FSI6_JOY_LX(fsi6) (fsi6->joy[RC_LX]) // 左摇杆 横轴
#define FSI6_JOY_LY(fsi6) (fsi6->joy[RC_LY]) // 左摇杆 纵轴
#define FSI6_JOY_RX(fsi6) (fsi6->joy[RC_RX]) // 右摇杆 横轴
#define FSI6_JOY_RY(fsi6) (fsi6->joy[RC_RY]) // 右摇杆 纵轴
/// @brief 波轮
#define FSI6_VR_A(fsi6) (fsi6->vr[RC_A])
#define FSI6_VR_B(fsi6) (fsi6->vr[RC_B])
/// @brief 开关
#define FSI6_SW_A(fsi6) (fsi6->sw[RC_A])
#define FSI6_SW_B(fsi6) (fsi6->sw[RC_B])
#define FSI6_SW_C(fsi6) (fsi6->sw[RC_C])
#define FSI6_SW_D(fsi6) (fsi6->sw[RC_D])
/// @brief 状态
#define FSI6_STATUS(fsi6) (fsi6->status)
#define FSI6_OFFLINE(fsi6) (fsi6->status == RC_OFFLINE ? 1 : 0) // 离线检测

/**************************************************************** struct ****************************************************************/

 enum RC_Status
{
    RC_STATUS_NONE = 0,
    RC_STATUS_OFFLINE = 0,
    RC_STATUS_ONLINE = 1,
    RC_STATUS_ERROR = 2,
} ;

typedef struct FSI6_Def_t
{
    enum RC_Status status;
    int16_t channel[18];
    int16_t joy[4];
    int16_t vr[2];
    uint8_t sw[4];
    uint32_t offline_cnt;
} FSI6_Def_t;

/**************************************************************** extren ****************************************************************/

/**************************************************************** proto ****************************************************************/

uint8_t FSI6_Init(FSI6_Def_t *hfsi6);
uint8_t FSI6_Parse(FSI6_Def_t *hfsi6, uint8_t *data, uint8_t len);

#endif
