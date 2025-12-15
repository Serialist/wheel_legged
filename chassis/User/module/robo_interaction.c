/************************
 * @file robo_interaction.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-12-14
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#include "robo_interaction.h"

/**
 * 交互接口大概分为三类：开关 switch、通道 channel 和数据流 data stream
 * switch 是一种离散量，用于表示状态、模式等信息
 * channel 是一种连续量，用于表示各种量
 * data stream 是预留给各种数据流的接口，如图像、点云、音频等
 *
 * 交互是一个过程，包括单工和双工
 *
 * 交互对象是 2 个及以上的模块，有主从和平等之分。主从时，交互对象分为主模块和从模块，主模块也称控制端或 user，从模块也称被控端或 device
 */

struct Remote_Control
{
    int16_t ch[16];
    uint16_t sw[16];
};
