#include "referee.h"
#include "crc_t.h"
#include "usart.h"
Referee_info_t REF;

void Referee_Read_Data(uint8_t *ReadFromUsart )
{
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析	
		
	memcpy(&REF.FrameHeader,ReadFromUsart,LEN_HEADER);   //储存帧头数据
	
	if(ReadFromUsart[SOF_t] == JUDGE_FRAME_HEADER)                   //判断帧头是否为0xa5
	{
		if(Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)  //帧头CRC校验
		{
			judge_length = ReadFromUsart[DATA_LENGTH_t] + LEN_HEADER + LEN_CMDID + LEN_TAIL;	//统计一帧数据长度,用于CR16校验
			
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)//帧尾CRC16校验
			{
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				if(CmdID == ID_keyboard_information)
				{
					 memcpy(&REF.RemoteControl, (ReadFromUsart+DATA), LEN_keyboard_information);
				}
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + REF.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Referee_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + REF.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

int usart1=0;
uint8_t Usart1_Buffer_Rx[ USART1_BUFFER_RX_LEN ] = {0};
void USART1_IRQHandler(void)
{
	usart1++; 
  uint8_t res;
	
	if(huart1.Instance->SR & UART_FLAG_IDLE)
	{
		res=res;
		res = USART1->SR ;
		res = USART1->DR ;
		
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
		
		HAL_UART_Receive_DMA(&huart1,Usart1_Buffer_Rx,100);
		
		res=USART1_BUFFER_RX_LEN-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		
		Referee_Read_Data(Usart1_Buffer_Rx);
		
		memset(Usart1_Buffer_Rx, 0, USART1_BUFFER_RX_LEN);	// 读完之后内容清零
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF1_5);
		__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,USART1_BUFFER_RX_LEN);
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
	}
}
