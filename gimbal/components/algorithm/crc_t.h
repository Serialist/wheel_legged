#ifndef __CRC_T_H
#define __CRC_T_H
#include "stdint.h"
#include "stdlib.h"
/***********************************************************************/
/*  文件作用：存放用来实现CRC校验的函数和宏定义
    文件说明：CRC校验是用在和裁判系统的通信，云台和裁判系统交互的通道是图传链路
	底盘和裁判系统交互的通道是电源管理模块                               */
/***********************************************************************/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif
