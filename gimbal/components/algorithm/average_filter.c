#include "average_filter.h"
int16_t test_M[3];

/*滑动平均滤波器进入队列，先进先出 */
void average_add(moving_average_filter_t *Aver, float add_data)
{
	//先入先出处理
	Aver->total -=  Aver->num[Aver->pot];
	Aver->total += add_data;
	//先入先出处理
	Aver->num[Aver->pot] = add_data;
	
	Aver->aver_num = (Aver->total)/(Aver->lenth);
	Aver->pot++;
	
	if(Aver->pot == Aver->lenth)
	{
		Aver->pot = 0;
	}
}

/*获取第前pre次的数据，如果超出数组长度则取记录的最早的数据*/
float average_get(moving_average_filter_t *Aver, uint16_t pre)
{
	float member;
	uint8_t temp;
	
	if(Aver->pot != 0)
	{
		temp = Aver->pot-1;
	}
	else 
	{
		temp = Aver->lenth-1;
	}
	
	if(pre>Aver->lenth)
		pre = pre % Aver->lenth;
	
	if(pre>temp)
	{
		pre = Aver->lenth+temp-pre;
	}
	
	else
	{
		pre = temp-pre;
	}
	
	member = Aver->num[pre];
	
	return member;
}

/*滑动滤波器初始化，设置长度*/
void average_init(moving_average_filter_t *Aver, uint8_t lenth)
{
	uint16_t i;
	
	for(i = 0; i < MAF_MaxSize; i++)
		Aver->num[i] = 0;
	
	if(lenth > MAF_MaxSize)
	{
		lenth = MAF_MaxSize;
	}
	
	Aver->lenth = lenth;
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
}

/*滑动滤波器清空*/
void average_clear(moving_average_filter_t *Aver)
{
	uint16_t i;
	
	for(i = 0; i<MAF_MaxSize; i++)
		Aver->num[i] = 0;
	
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
}

/*滑动滤波器填充值*/
void average_fill(moving_average_filter_t *Aver, float temp)
{
	uint16_t i;
	
	for(i = 0; i<(Aver->lenth); i++)
		Aver->num[i] = temp;
	
	Aver->pot = 0;
	Aver->aver_num = temp;
	Aver->total = temp*(Aver->lenth);
}

float dith_Filter(float input)

{
	
float per_input;
	if(per_input-input>4||per_input-input<-4)
	{
	return per_input;
	}
	else
	{
	return input;
	}
per_input=input;
	
}

//用结构体写好的均值滤波//
float moveAverageFilter_T(Average_type_def_t *Ave,float input)
{
	
	
    if(Ave->Aver_num < Ave->Aver_tot_num)
    {
        Ave->Aver_array[Ave->Aver_num] = input;
        Ave->Aver_sum_out += Ave->Aver_array[Ave->Aver_num];
			  Ave->Aver_num++;
        return Ave->Aver_sum_out/Ave->Aver_num;
    }
    else
    {
        Ave->Aver_sum_out -=   Ave->Aver_sum_out/Ave->Aver_tot_num;
        Ave->Aver_sum_out +=  input;
        return Ave->Aver_sum_out/Ave->Aver_num;
    }
}

//用结构体写好的均值滤波//



//中位值平均滤波//
float M_Ave_filter(median_Ave_type_def_t *M_Ave,float input) 

{
	
   M_Ave->M_Ave_Buf[M_Ave->n]=input;
	 M_Ave->M_Ave_sum_out=0;
  (M_Ave->n)++;
	
   if(M_Ave->n>=4)
	 {
		 
		  M_Ave->n = 0;
		 
	 }
		 //冒泡排序求最大值//
	 for(M_Ave->j=0;M_Ave->j < 3;M_Ave->j++)
	 {
			 for(M_Ave->i = 0;M_Ave->i < 3 - M_Ave->j;M_Ave->i++) 
		{
      if(M_Ave->M_Ave_Buf[M_Ave->i] > M_Ave->M_Ave_Buf[M_Ave->i+1]) 
			{
        M_Ave->M_Ave_temp = M_Ave->M_Ave_Buf[M_Ave->i];
        M_Ave->M_Ave_Buf[M_Ave->i] =  M_Ave->M_Ave_Buf[M_Ave->i+1];
        M_Ave->M_Ave_Buf[M_Ave->i+1]=M_Ave->M_Ave_temp;
      }
    } 	 
	 }
	  //冒泡排序求最大值//
	 
	 for(M_Ave->i=1;M_Ave->i<3;M_Ave->i++)
	 {
	  M_Ave->M_Ave_sum_out+=M_Ave->M_Ave_Buf[M_Ave->i];
	 } 
	  
	 
	return M_Ave->M_Ave_sum_out /2;
 
}
//中位值平均滤波//

//低通滤波器
float Low_pass_filter(LPF_t *lpf_t,float input,float LPF_par)
{
 lpf_t->now_data=input;
 if(lpf_t->last_data!=0)
 {
 lpf_t->output=lpf_t->now_data*(1-LPF_par)+LPF_par*lpf_t->last_data;
 }
 lpf_t->now_data=lpf_t->output;
 lpf_t->last_data=lpf_t->now_data;
return lpf_t->output;
}
//低通滤波器
