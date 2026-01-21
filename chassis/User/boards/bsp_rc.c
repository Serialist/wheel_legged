/***********************************************
 * @file bsp_rc.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief DT7遥控器接收数据函数
 * @version 0.1.0
 * @date 2025-10-30
 *
 * @copyright Copyright (c) VGD 2025
 *
 * @note
 * 正式比赛时底盘不接遥控器数据，云台接遥控器数据，云台通过CAN2将各种控制信息传给底盘
 * 平时调试和检录时可用DT7遥控器调试，比赛时统一用图传链路，控制信息都从云台传下底盘
 *************************************************/

/* ================================================================ include ================================================================ */

#include "usart.h"
#include "bsp_rc.h"

/* ================================================================ define ================================================================ */

/* ================================================================ struct ================================================================ */

/* ================================================================ const ================================================================ */

/* ================================================================ var ================================================================ */

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* ================================================================ proto ================================================================ */

void UART_BSP_DMA_2Buf_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, uint8_t *buf1, uint8_t *buf2, uint16_t dma_buf_len);
void UART_BSP_DMA_2Buf_Unable(UART_HandleTypeDef *huart);
void UART_BSP_DMA_2Buf_Restart(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, uint16_t dma_buf_len);

/* ================================================================ func ================================================================ */

/* ================================================ adapter ================================================*/

/********************************
 * @brief 初始化遥控器接收数据
 *
 * @param rx1_buf 两个接收数据的缓冲区
 * @param rx2_buf
 * @param dma_buf_num
 *
 * @note
 * 使用 DMA 接收数据，开启空闲中断：
 *
 * 1. 设置 寄存器、缓冲区 地址。开启 DMA 后，自动接收数据
 * 2. 开 DMA 双缓冲，用 CT 位切换缓冲区
 * 3. 开空闲中断
 * 4. 空闲中断，在接收完成后触发
 *    要在 ISR（中断函数）中，重启 DMA 接收
 ********************************/
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    UART_BSP_DMA_2Buf_Init(&huart3, &hdma_usart3_rx, rx1_buf, rx2_buf, dma_buf_num);
}

/************************
 * @brief 关闭遥控器接收数据
 *
 ************************/
void RC_unable(void)
{
    UART_BSP_DMA_2Buf_Unable(&huart3);
}

/************************
 * @brief 重启遥控器接收数据
 *
 * @param dma_buf_num
 ************************/
void RC_restart(uint16_t dma_buf_num)
{
    UART_BSP_DMA_2Buf_Restart(&huart3, &hdma_usart3_rx, dma_buf_num);
}

/* ================================================ UART DMA 2 buf ================================================*/

/***********************************************
 * @brief DMA 双缓冲区 初始化
 *
 * @param rx1_buf 两个接收数据的缓冲区
 * @param rx2_buf
 * @param dma_buf_num
 *
 * @note
 *************************************************/
void UART_BSP_DMA_2Buf_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, uint8_t *buf1, uint8_t *buf2, uint16_t dma_buf_len)
{
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); // enable DMA rx
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);     // enalbe idle interrupt

    __HAL_DMA_DISABLE(hdma_usart_rx); // disable DMA 失效 DMA

    while (hdma_usart_rx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma_usart_rx);
    }

    hdma_usart_rx->Instance->PAR = (uint32_t)&(huart->Instance->DR);

    hdma_usart_rx->Instance->M0AR = (uint32_t)(buf1); // buf 1
    hdma_usart_rx->Instance->M1AR = (uint32_t)(buf2); // buf 2

    hdma_usart_rx->Instance->NDTR = dma_buf_len; // data len

    SET_BIT(hdma_usart_rx->Instance->CR, DMA_SxCR_DBM); // enable 2 buf
    __HAL_DMA_ENABLE(hdma_usart_rx);                    // enable DMA
}

/***********************************************
 * @brief DMA 双缓冲区 关闭
 *
 *************************************************/
void UART_BSP_DMA_2Buf_Unable(UART_HandleTypeDef *huart)
{
    __HAL_UART_DISABLE(huart);
}

/***********************************************
 * @brief DMA 双缓冲区 重启
 *
 * @param dma_buf_num
 *************************************************/
void UART_BSP_DMA_2Buf_Restart(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, uint16_t dma_buf_len)
{
    __HAL_UART_DISABLE(huart);
    __HAL_DMA_DISABLE(hdma_usart_rx);

    hdma_usart_rx->Instance->NDTR = dma_buf_len;

    __HAL_DMA_ENABLE(hdma_usart_rx);
    __HAL_UART_ENABLE(huart);
}

/************************
 * @brief DMA 双缓冲区 处理函数
 *
 ************************/
void UART_BSP_DMA_2Buf_Callback(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, uint16_t buf_len, uint16_t frame_len, uint8_t *buf, void (*Parse_Func)(uint8_t *data))
{

    if (huart->Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {

        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(huart);

        if ((hdma_usart_rx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(hdma_usart_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = buf_len - hdma_usart_rx->Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart_rx->Instance->NDTR = buf_len;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart_rx->Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(hdma_usart_rx);

            if (this_time_rx_len == frame_len)
            {
                Parse_Func(&buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(hdma_usart_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = buf_len - hdma_usart_rx->Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart_rx->Instance->NDTR = buf_len;

            // set memory buffer 0
            // 设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(hdma_usart_rx);

            if (this_time_rx_len == frame_len)
            {
                // 处理遥控器数据
                Parse_Func(&buf[1]);
            }
        }
    }
}
