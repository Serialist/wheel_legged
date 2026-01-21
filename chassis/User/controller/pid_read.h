#include "struct_typedef.h"

typedef enum
{
	SHUT_ = 0,
	OPEN_,
	READY_
}Read_Flag;

typedef struct
{
	float pid_data[30];
	Read_Flag read_flag;
	uint16_t count;
}Read_PID_Data_t;


