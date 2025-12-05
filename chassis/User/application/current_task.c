#include "current_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "cmsis_os.h"
#include "kalman.h"
#include "chassis_task.h"

extern Chassis_t chassis;

/****************变量定义****************/
float coe[AVERAGE_FILTER_NUM] = {1, 2, 3};    // 加权系数表
kalman cur_km;
/****************************************/


/****************函数声明****************/
void GetChassisCurrent(Chassis_t* motor_current);
void currentFilter(Chassis_t* cur_filter);
static void currentTaskInit(void);
static void currentKmFilter(Chassis_t* cur_filter, kalman *p);
/****************************************/


void current_task(void const * argument)
{
	currentTaskInit();
	for(;;)
	{
		GetChassisCurrent(&chassis);
		//currentFilter(&chassis);
		currentKmFilter(&chassis, &cur_km);
		osDelay(1);
	}
}

static void currentTaskInit(void)
{
	chassis.current.adc_scale = ADC_CURRENT_SCALE;
	
	kalmanCreate(&cur_km, 5, 1000);
}

void GetChassisCurrent(Chassis_t* motor_current)
{
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
//	motor_current->current.omni_motor_cur[O1] = (float)HAL_ADC_GetValue(&hadc1) * motor_current->current.adc_scale;
	
	HAL_ADC_PollForConversion(&hadc1,10);
//	motor_current->current.omni_motor_cur[O2] = (float)HAL_ADC_GetValue(&hadc1) * motor_current->current.adc_scale;
	
//	HAL_ADC_PollForConversion(&hadc1,10);
//	motor_current->current.motor_cur[M3] = (float)HAL_ADC_GetValue(&hadc1) * ADC_CURRENT_SCALE;
//	
//	HAL_ADC_PollForConversion(&hadc1,10);
//	motor_current->current.motor_cur[M4] = (float)HAL_ADC_GetValue(&hadc1) * ADC_CURRENT_SCALE;
                              
//	motor_current->current.cur_sum = motor_current->current.omni_motor_cur[O1] + motor_current->current.omni_motor_cur[O2];
	
//	motor_current->current.cur_sum = (motor_current->current.motor_cur[M1] + motor_current->current.motor_cur[M2] \
//			+ motor_current->current.motor_cur[M3] + motor_current->current.motor_cur[M4]);
}

#if CURRENT_READ == 1
int cur_index = 0;
int cur_fdb[10000];
int cur_ftl[10000];
int cur_real[10000];
int cur_d;
#endif
void currentFilter(Chassis_t* cur_filter)
{
	static float current_f[AVERAGE_FILTER_NUM] = {0};
	static float current_f2[2] = {0};
	
	//加权平均递推滤波
	int i;
	float temp = 0;
	for(i = 0; i < AVERAGE_FILTER_NUM; i++)
	{
		if(i == AVERAGE_FILTER_NUM - 1){
			current_f[i] = cur_filter->current.cur_sum;
		}
		else{
			current_f[i] = current_f[i + 1];
		}			
		
		
		temp += current_f[i] * coe[i];
	}
	current_f[AVERAGE_FILTER_NUM - 1] = temp / 6;	//6 1到3的和
	//cur_filter->current.cur_filter = current_f[AVERAGE_FILTER_NUM - 1];
	
	//一阶滞后滤波
	current_f2[0] = current_f2[1];
	current_f2[1] = FILTER_A * current_f[AVERAGE_FILTER_NUM - 1] + \
										(1 - FILTER_A) * current_f2[0];
	cur_filter->current.cur_filter = current_f2[1];

#if CURRENT_READ == 1 
cur_fdb[cur_index] = cur_filter->current.cur_sum * 100;
cur_ftl[cur_index] = cur_filter->current.cur_filter * 100;
cur_real[cur_index] = cur_filter->power_data.chassis_power_fdb / 22.0f * 100;
cur_d = cur_real[cur_index];
cur_index = cur_index >= 9999 ? 9999 : cur_index + 1;
#endif
}

static void currentKmFilter(Chassis_t* cur_filter, kalman *p)
{
	cur_filter->current.cur_filter = KalmanFilter(p, cur_filter->current.cur_sum);
}
