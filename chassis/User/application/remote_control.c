/************************
 * @file remote_control.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief 遥控器处理
 * @version 0.1.0
 * @date 2025-10-30
 *
 * @copyright Copyright (c) VGD 2025
 *
 * @note
 * 类 SBUS 协议
 * 使用 DMA，节约 CPU 资源
 * 空闲中断处理数据
 * 提供掉线重启 DMA、串口，保证热插拔稳定性。
 * 通过串口中断启动，不是 freeRTOS 任务
 ************************/

/* ================================================================ include ================================================================ */

#include "remote_control.h"
#include "main.h"
#include "string.h"

#include "cmsis_os.h"

/* ================================================================ define ================================================================ */

#define RC_CHANNAL_ERROR_VALUE 700 // 遥控器出错数据上限

// 取正函数
#define RC_ABS(value) ((value) > 0 ? (value) : -(value))

/* ================================================================ struct ================================================================ */

/* ================================================================ const ================================================================ */

/* ================================================================ var ================================================================ */

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

DT7_Data rc_ctrl; // 遥控器变量

uint32_t rc_time = 0;

/* ================================================================ proto ================================================================ */

static void Sbus_Parse(DT7_Data *rc_ctrl, volatile const uint8_t *sbus_buf);
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM]; // 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界

/* ================================================================ func ================================================================ */

/************************
 * @brief 遥控器初始化
 *
 ************************/
void remote_control_init(void)
{
    rc_ctrl.offline_flag = 1;
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/************************
 * @brief 获取遥控器数据指针
 *
 * @return const DT7_Data* 遥控器数据指针
 ************************/
const DT7_Data *get_remote_control_point(void)
{
    return &rc_ctrl;
}

/************************
 * @brief 判断遥控器数据是否出错
 *
 * @return uint8_t (True)?0:1
 ************************/
uint8_t RC_Data_Check(void)
{
    // 使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_ABS(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_ABS(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_ABS(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_ABS(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    //    rc_ctrl.rc.s[0] = RC_SW_UP;
    //    rc_ctrl.rc.s[1] = RC_SW_MID;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void Slove_RC_Lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void Slove_Data_Error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

int usart3_d = 0;
// 串口中断
void USART3_IRQHandler(void)
{
    usart3_d++;
    if (huart3.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                Sbus_Parse(&rc_ctrl, sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // 处理遥控器数据
                Sbus_Parse(&rc_ctrl, sbus_rx_buf[1]);
            }
        }
    }
}

// int rem_d = 0;
// int rem_d_d = 0;
/************************
 * @brief 遥控器协议解析
 *
 * @param sbus_buf 原数据
 * @param rc_ctrl 遥控器
 ************************/
static void Sbus_Parse(DT7_Data *rc_ctrl, volatile const uint8_t *buf)
{
    if (buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    rc_time = xTaskGetTickCount();

    // rem_d_d++;
    rc_ctrl->rc.ch[0] = (buf[0] | (buf[1] << 8)) & 0x07ff;                         // Channel 0
    rc_ctrl->rc.ch[1] = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff;                  // Channel 1
    rc_ctrl->rc.ch[2] = ((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07ff; // Channel 2
    rc_ctrl->rc.ch[3] = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff;                  // Channel 3
    rc_ctrl->rc.s[0] = ((buf[5] >> 4) & 0x0003);                                   // Switch left
    rc_ctrl->rc.s[1] = ((buf[5] >> 4) & 0x000C) >> 2;                              // Switch right
    rc_ctrl->mouse.x = buf[6] | (buf[7] << 8);                                     // Mouse X axis
    rc_ctrl->mouse.y = buf[8] | (buf[9] << 8);                                     // Mouse Y axis
    rc_ctrl->mouse.z = buf[10] | (buf[11] << 8);                                   // Mouse Z axis
    rc_ctrl->mouse.press_l = buf[12];                                              // Mouse Left Is Press
    rc_ctrl->mouse.press_r = buf[13];                                              // Mouse Right Is Press
    rc_ctrl->key.v = buf[14] | (buf[15] << 8);                                     // KeyBoard value
    rc_ctrl->rc.ch[4] = buf[16] | (buf[17] << 8);                                  // Wheel

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    // 记录出现错误的次数
    // if (RC_data_is_error() == 1)
    //     rem_d++;

    rc_ctrl->receive_flag = 1;
}

/************************
 * @brief 获取遥控器数据
 *
 * @param rc_data
 * @return uint32_t
 ************************/
uint32_t GetRCData(DT7_Data *rc_data)
{
    *rc_data = rc_ctrl;
    return rc_time;
}

#define RC_TIMEOUT 200 // 超时时间 单位 ms
/************************
 * @brief 离线检测
 *
 * @param rc_ctrl
 * @param dt
 ************************/
void RC_Offline_Detection(DT7_Data *rc_ctrl, uint32_t dt)
{
    if (rc_ctrl->receive_flag)
    {
        rc_ctrl->time_count = 0;
        rc_ctrl->offline_flag = 0;
        rc_ctrl->receive_flag = 0;
    }
    else
    {
        if (rc_ctrl->time_count >= RC_TIMEOUT)
        {
            rc_ctrl->offline_flag = 1;
        }
        else
        {

            rc_ctrl->time_count += dt;
        }
    }
}
