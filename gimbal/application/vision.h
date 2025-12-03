#ifndef __VISION_H
#define __VISION_H
#include "struct_typedef.h"
#define USART6_BUFFER_RX_LEN 30
#define vision_red  0x00
#define vision_blue 0x01
#define shoot_fire 0x01
#define shoot_stop 0x00

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

typedef union
{
	float data_raw;
	uint8_t data[4];
}Float_Transform_u;
typedef union{
	int32_t data_int;
uint8_t data[4];
}Int_Transform_u;
typedef struct
{	
	float angle_fdb[2];
	float angle_set_err[3];
	uint8_t  vision_flag;
	uint8_t  fire_flag;
  int x_go;
	int y_go;
	uint8_t navi_finish;
}Vision_t;




void Float_to_Byte(float f,unsigned char byte[]);
void 	user_USB_Receive(uint8_t* Buf);
void transmit_vision_data(void);
void vision_task(void const *pvParameters);
void Get_IMU_Angle_Data(void);
void Get_vision_Data(Vision_t* vision_data);
#endif
