
#include "constrain_calc.h"
float fdb_set_err;
int8_t random_spin_flag;

void ConstrainLoop(float* output, float input, float min, float max)
{
	if (input >max)
	{
		do
		{
			input = input - (max - min);
		}while(input > max);
	}
	else if(input < min)
	{
		do
		{
			input = input + (max - min);
		}while(input < min);
	}
	
	*output = input;
}

void SectionTransform(float* output, float input, float out_min, float out_zero, float out_max, float in_min, float in_zero, float in_max)
{
	float temp_in = -(input - in_zero);
	float temp_out = 0;
	
	if(temp_in > (in_max - in_min) / 2)
	{
		temp_in = temp_in - (in_max - in_min);
	}
	else if(temp_in < -(in_max - in_min) / 2)
	{
		temp_in = temp_in + (in_max - in_min);
	}
	
	temp_out = temp_in * (out_max - out_min) / (in_max - in_min) + out_zero;
	ConstrainLoop(output, temp_out, out_min, out_max);
}

void MinMaxConstrain(float* input, float min, float max)
{
	if(*input > max)
	{
		*input = max;
	}
	else if(*input < min)
	{
		*input = min;
	}
}

float Abs(float num)
{
	return (num >= 0) ? num : -num;
}


/**
  * @brief 防止PITCH、YAW轴绕大圈达到设定值，计算出一个临时的设定值
  * @param 角度设定值
  * @param 角度反馈值
  * 
  * @retval 临时角度设定值
  */
float PreventFullCircleConstrain(float set, float fdb)
{
	if(set - fdb >180.0f)
	{
	set=set-360.0f;
	return set; 
	}
		
	else if(set - fdb < -180.0f)
	{
		set=set+360.0f;
		return set;
	}
	
	else
	{
		return set;
	}
		
	
}
float PreventFullCircleConstrain_ecd(float set, float fdb)
{
	if(set - fdb >7000.f)
	{
	set=set-8191.f;
	return set; 
	}
	else if(set - fdb < -7000.f)
	{
		set=set+8191.f;
		return set;
	}
	
	else
	{
		return set;
	}
}

float random_spin_PreventFullCircleConstrain(float set, float fdb)
{
	
if(fdb_set_err>=10||fdb_set_err<=-10)
	{
	random_spin_flag=1;
	}	
	
	if(set - fdb >180.0f)
	{
	set=set-360.0f;
	return set; 
	fdb_set_err=set-fdb;
	}
		
	else if(set - fdb < -180.0f)
	{
		set=set+360.0f;
		return set;
		fdb_set_err=set-fdb;
	}
	
	else
	{
		fdb_set_err=set-fdb;
		return set;
	}
	
		
}

int flag_over=0;
int flag_less=0;
int temp=0;
float yaw_PreventFullCircleConstrain(float set, float fdb)
{

	if(set - fdb >180.0f)
	{flag_over=1;temp++; return set - 360.0f; }
	else if(set - fdb < -180.0f)
	{flag_less=1;	return set + 360.0f; }
	else
		return set;
	
}
