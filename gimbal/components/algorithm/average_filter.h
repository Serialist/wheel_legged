#ifndef AVERAGE_FILTER__H__
#define AVERAGE_FILTER__H__

#include "struct_typedef.h"

#define MAF_MaxSize 100

typedef struct moving_average_filter_struct
{
	float num[MAF_MaxSize];
	uint8_t lenth;
	uint8_t pot;//当前位置
	float total;
	float aver_num;
}moving_average_filter_t;	//最大设置MAF_MaxSize个


//*************均值滤波结构体
typedef struct
{
	int16_t Aver_num;
	int16_t Aver_tot_num;
	float *Aver_array;
	float Aver_sum_out;
	float Aver_array_mem;
   
}Average_type_def_t;


//*************均值中位数滤波结构体
typedef struct
{
int n,i,j;
float  M_Ave_temp;
float M_Ave_Buf[4];
float  M_Ave_sum_out;
int16_t M_Ave_Flag[3];
int16_t M_Ave_num;
float per_M_Ave_sum_out;
}
median_Ave_type_def_t;


//*************低通滤波结构体
typedef struct
{
	float last_data;
	float now_data;
	float output;
	float input;
	float LPF_par;
}LPF_t;


//滑动滤波器对应的操作函数
extern void average_add(moving_average_filter_t *Aver, float add_data);
extern float average_get(moving_average_filter_t *Aver, uint16_t pre);//获取前n次的数据
extern void average_init(moving_average_filter_t *Aver, uint8_t lenth);
extern void average_clear(moving_average_filter_t *Aver);
extern void average_fill(moving_average_filter_t *Aver, float temp);//往滑动滤波填充某个值
float Low_pass_filter(LPF_t *lpf_t,float input,float LPF_par);
int16_t moveAverageFilter(int16_t input);
float dith_Filter(float input);
float filter5(float input);
float moveAverageFilter_T(Average_type_def_t *Ave,float input);
float M_Ave_filter(median_Ave_type_def_t *M_Ave,float input);
#endif

