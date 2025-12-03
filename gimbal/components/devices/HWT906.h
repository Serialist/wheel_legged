#ifndef HWT906_H
#define HWT906_H

/*  
文件说明：2025赛季尝试使用维特智能的HWT906九轴传感器通过串口获取姿态角，其为成品陀螺仪可以直接读取姿态角。
他的姿态角比较精准，但是它的分度值太大了，每轴的角度只用一个16位的数据表示，没办法用于电机控制，故放弃
*/
void HWT906_Read_Data(unsigned char ucData);


#endif
