/*
  文件内容：DT7遥控器接收数据函数
  文件注释：正式比赛时底盘不接遥控器数据，云台接遥控器数据，云台通过CAN2将各种控制信息传给底盘
	         平时调试和检录时可用DT7遥控器调试，比赛时统一用图传链路，控制信息都从云台传下底盘
*/
#include "usart.h"
#include "bsp_rc.h"
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//函数作用：初始化遥控器接收数据
//参数：两个接收数据的缓冲区
//返回值：无
//函数注释：使用DMA接收数据，开启空闲中断
/*DMA接收数据时需要设置寄存器地址和缓冲区地址，开启DMA后会自动接收数据
使用DMA接收数据时需要开启DMA的双缓冲模式，使用CT位来切换缓冲区
使用DMA接收数据时需要开启空闲中断，空闲中断会在接收完成后触发
空闲中断会在接收完成后触发，需要在中断服务函数中重新开启DMA接收*/
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    //enable DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}
/*函数作用：关闭遥控器接收数据*/
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
/*函数作用：重启遥控器接收数据*/
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}
