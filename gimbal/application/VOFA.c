/*
----------------------------------
使用VOFA把数据可视化。
说明：利用串口6或其他串口向VOFA发送数据。

有两种发送形式，一种是直接使用printf函数发送数据
另一种是要使用到共用体将float类型转换成整型加上帧头帧尾发送
具体可查看VOFA提供的文档
----------------------------------
*/
#include "VOFA.h"
#include "usart.h"

extern UART_HandleTypeDef huart6;
fp32 buffer[3];

/*
函数作用：串口重定向函数，将串口六定向到printf函数（不知道能不能怎么说，反正是这个意思）
参数：ch：要发送的字符；f：文件指针
返回值：ch：发送的字符
说明：该函数重定向了printf函数的输出到串口六，这样就可以将数据发送到VOFA上。
*/
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xffff);//HAL库串口发送函数
  return ch;
}

/*
函数作用：用printf函数发送数据到VOFA，对printf函数做再封装
参数：无
返回值：无
*/
void transmit_to_VOFA(void)//发送上位机函数firewater
{
  //printf("%2f,%2f\n",pitch_curren[normal],ecd_out);
}

HAL_StatusTypeDef TEST_UART[3]; 

void vofa_send()
{

	uint8_t byte[4]={0};		//float转化为4个字节数据
	uint8_t tail[4]={0x00, 0x00, 0x80, 0x7f};	//帧尾
	//Float_to_Byte(pitch_curren,byte);
    TEST_UART[0]=HAL_UART_Transmit(&huart6, byte, sizeof(byte), 10);
	//Float_to_Byte(ecd_out,byte);
	TEST_UART[1]=HAL_UART_Transmit(&huart6, byte, sizeof(byte), 10);
	
	TEST_UART[2]=HAL_UART_Transmit(&huart6, tail, sizeof(tail), 10);
    HAL_Delay(5);
}
