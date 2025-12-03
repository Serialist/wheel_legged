#ifndef _BSP_REFEREE_H
#define _BSP_REFEREE_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "referee.h"
//还另外有一个referee.h文件用来存放裁判系统的结构体
void Referee_Read_Data(uint8_t *ReadFromUsart);
void new_rc_read_data(uint8_t *ReadFromUsart);
void Get_New_Rc_data(remote_data_t* rc_data);
void Get_Referee_Data(Referee_info_t* referee_data);
#endif
