#include "HWT906.h"
/*  
文件说明：2025赛季尝试使用维特智能的HWT906九轴传感器通过串口获取姿态角，其为成品陀螺仪可以直接读取姿态角。
他的姿态角比较精准，但是它的分度值太大了，每轴的角度只用一个16位的数据表示，没办法用于电机控制，故放弃
*/
float HWT_angle[3];
void HWT906_Read_Data(unsigned char ucData)
{
 static unsigned char ucRxBuffer[25];
    static unsigned char ucRxCnt = 0;

    ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
    if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
    {
        ucRxCnt=0;
        return;
    }
    if (ucRxCnt<11)
    {
        return;   //数据不满11个，则返回
    }
    else
    {
        switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
        {
				case 0x53:
        HWT_angle[0]=(ucRxBuffer[3]<<8|ucRxBuffer[2])/32768*180;
				HWT_angle[1]=(ucRxBuffer[5]<<8|ucRxBuffer[4])/32768*180;
				HWT_angle[2]=(ucRxBuffer[7]<<8|ucRxBuffer[6])/32768*180;
        break;				
				}
				ucRxCnt=0;

   }
 }
