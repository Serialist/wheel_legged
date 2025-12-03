#include "fuzzyPID.h"
#include "stdio.h"
#include <stdlib.h>
#include <math.h>

static void setMf_sub(Fuzzy_PID_t* pid, MF_Type_e type,float *paras,int n);
static float Trimf(float x,float a,float b,float c);
static float Gaussmf(float x,float ave,float sigma);
static float Trapmf(float x,float a,float b,float c,float d);

#define LimitMax(input, max)   \
    {                          \
        if (input > (max))       \
        {                      \
            input = (max);       \
        }                      \
        else if (input < -(max)) \
        {                      \
            input = -(max);      \
        }                      \
    }

#define LimitMaxMin(input, sign, max, min) 		\
	{                          					\
		if(sign >= 0)							\
		{										\
			if(input > max)						\
			{									\
				input = max;					\
			}									\
			else if(input < min)				\
			{									\
				input = min;					\
			}									\
		}										\
		else									\
		{										\
			if(input < -max)					\
			{									\
				input = -max;					\
			}									\
												\
			else if(input > -min)				\
			{									\
				input = -min;					\
			}									\
		}										\
	}	

void FuzzyPIDInit(Fuzzy_PID_t* pid, float e_max,float de_max,float kp_max,float ki_max,float kd_max,float Kp0,float Ki0,float Kd0)
{
	pid->target = 0;
	pid->actual = 0;
	pid->emax = e_max;
	pid->demax = de_max;
	pid->delta_Kp_max = kp_max;
	pid->delta_Ki_max = ki_max;
	pid->delta_Kd_max = kd_max;
	pid->e_mf_paras = NULL;
	pid->de_mf_paras = NULL;
	pid->Kp_mf_paras = NULL;
	pid->Ki_mf_paras = NULL;
	pid->Kd_mf_paras = NULL;
	
	pid->e = pid->target - pid->actual;
	pid->e_pre_1 = 0;
	pid->e_pre_2 = 0;
	pid->de = pid->e - pid->e_pre_1;
	pid->Ke = (FUZZY_N / 2) / pid->emax;
	pid->Kde = (FUZZY_N / 2) / pid->demax;
	pid->Ku_p = pid->delta_Kp_max / (FUZZY_N / 2);
	pid->Ku_i = pid->delta_Ki_max / (FUZZY_N / 2);
	pid->Ku_d = pid->delta_Kd_max / (FUZZY_N / 2);

	pid->Kp0 = Kp0;
	pid->Ki0 = Ki0;
	pid->Kd0 = Kd0;
}

void SetMf(Fuzzy_PID_t* pid, 
			MF_Type_e mf_type_e,float *e_mf,
			MF_Type_e mf_type_de,float *de_mf,
			MF_Type_e mf_type_Kp,float *Kp_mf,
		    MF_Type_e mf_type_Ki,float *Ki_mf,
			MF_Type_e mf_type_Kd,float *Kd_mf)
{
	setMf_sub(pid, mf_type_e,e_mf,0);
	setMf_sub(pid, mf_type_de,de_mf,1);
	setMf_sub(pid, mf_type_Kp,Kp_mf,2);
	setMf_sub(pid, mf_type_Ki,Ki_mf,3);
	setMf_sub(pid, mf_type_Kd,Kd_mf,4);
}

//设置模糊规则Matrix
void SetRuleMatrix(Fuzzy_PID_t* pid, int kp_m[FUZZY_N][FUZZY_N], int ki_m[FUZZY_N][FUZZY_N], int kd_m[FUZZY_N][FUZZY_N])
{
	for (int i = 0; i < FUZZY_N; i++)
		for (int j = 0; j < FUZZY_N; j++)
		{
			pid->Kp_rule_matrix[i][j] = kp_m[i][j];
			pid->Ki_rule_matrix[i][j] = ki_m[i][j];
			pid->Kd_rule_matrix[i][j] = kd_m[i][j];
		}
}

float FuzzyPIDCalc(Fuzzy_PID_t* pid, float set, float fdb, float distance)
{
	
	//实现模糊控制
	float u_e[FUZZY_N], u_de[FUZZY_N];
	int u_e_index[3], u_de_index[3];//假设一个输入最多激活3个模糊子集
	pid->target = set;
	pid->actual = fdb;
	pid->e = pid->target - pid->actual;
	LimitMax(pid->e, pid->emax-0.01f);
	pid->de = pid->e - pid->e_pre_1;
	LimitMax(pid->de, pid->demax-0.01f);
	pid->e = pid->Ke * pid->e;
	pid->de = pid->Kde * pid->de;
	/* 将误差e模糊化*/
	int j = 0;
	for (int i = 0; i < FUZZY_N; i++)
	{
		if (pid->mf_t_e == TRIMF)
			u_e[i] = Trimf(pid->e, pid->e_mf_paras[i * 3], pid->e_mf_paras[i * 3 + 1], pid->e_mf_paras[i * 3 + 2]);//e模糊化，计算它的隶属度
		else if (pid->mf_t_e == GAUSSMF)
			u_e[i] = Gaussmf(pid->e, pid->e_mf_paras[i * 2], pid->e_mf_paras[i * 2 + 1]);//e模糊化，计算它的隶属度
		else if (pid->mf_t_e == TRAPMF)
			u_e[i] = Trapmf(pid->e, pid->e_mf_paras[i * 4], pid->e_mf_paras[i * 4 + 1], pid->e_mf_paras[i * 4 + 2], pid->e_mf_paras[i * 4 + 3]);//e模糊化，计算它的隶属度

		if (u_e[i] != 0)
		{
			u_e_index[j++] = i;                //存储被激活的模糊子集的下标，可以减小计算量
		}
	}
	for (; j < 3; j++)u_e_index[j] = 0;             //富余的空间填0

	/*将误差变化率de模糊化*/
	j = 0;
	for (int i = 0; i < FUZZY_N; i++)
	{
		if (pid->mf_t_de == TRIMF)
			u_de[i] = Trimf(pid->de, pid->de_mf_paras[i * 3], pid->de_mf_paras[i * 3 + 1], pid->de_mf_paras[i * 3 + 2]);//de模糊化，计算它的隶属度
		else if (pid->mf_t_de == GAUSSMF)
			u_de[i] = Gaussmf(pid->de, pid->de_mf_paras[i * 2], pid->de_mf_paras[i * 2 + 1]);//de模糊化，计算它的隶属度
		else if (pid->mf_t_de == TRAPMF)
			u_de[i] = Trapmf(pid->de, pid->de_mf_paras[i * 4], pid->de_mf_paras[i * 4 + 1], pid->de_mf_paras[i * 4 + 2], pid->de_mf_paras[i * 4 + 3]);//de模糊化，计算它的隶属度

		if (u_de[i] != 0)
		{
			u_de_index[j++] = i;                //存储被激活的模糊子集的下标，可以减小计算量
		}
	}
	for (; j < 3; j++)u_de_index[j] = 0;          //富余的空间填0

	float den = 0, num = 0;
	/*计算delta_Kp和Kp*/
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * pid->Kp_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}
	
	pid->delta_Kp = num / den;
	
	pid->delta_Kp = pid->Ku_p * pid->delta_Kp;
	LimitMax(pid->delta_Kp, pid->delta_Kp_max);

	pid->Kp = pid->Kp0 + pid->delta_Kp;
	LimitMaxMin(pid->Kp, pid->Kp0, pid->Kp_max, pid->Kp_min);
	
	/*计算delta_Ki和Ki*/
	den = 0; num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * pid->Ki_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}

	pid->delta_Ki = num / den;
	
	pid->delta_Ki = pid->Ku_i * pid->delta_Ki;
	LimitMax(pid->delta_Ki, pid->delta_Ki_max);

	pid->Ki = pid->Ki0 + pid->delta_Ki;
	LimitMaxMin(pid->Ki, pid->Ki0, pid->Ki_max, pid->Ki_min);
		
	/*计算delta_Kd和Kd*/
	den = 0; num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * pid->Kd_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}
	pid->delta_Kd = num / den;
	
	pid->delta_Kd = pid->Ku_d * pid->delta_Kd;
	LimitMax(pid->delta_Kd, pid->delta_Kd_max);

	pid->Kd = pid->Kd0 + pid->delta_Kd;
	LimitMaxMin(pid->Kd, pid->Kd0, pid->Kd_max, pid->Kd_min);

	pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - fdb;
	pid->Pout = pid->Kp * pid->error[0];

	pid->Iout += pid->Ki * pid->error[0];
	
//	float i_limit = -0.1f * distance + 5.2857f;
//	i_limit = (i_limit < 0) ? 0 : i_limit;
//	i_limit = (i_limit > 4.5f) ? 4.5f : i_limit;
	if(fabs(pid->error[0]) < 1.0f)
		pid->Iout = 0;
	
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
    	
	pid->e_pre_2 = pid->e_pre_1;
	pid->e_pre_1 = pid->e;

	return pid->out;
}

//设置模糊隶属度函数的子函数
static void setMf_sub(Fuzzy_PID_t* pid, MF_Type_e type, float* paras, int n)
{
	int N_mf_e, N_mf_de, N_mf_Kp, N_mf_Ki, N_mf_Kd;
	switch (n)
	{
	case 0:
		pid->mf_t_e = type;
		if (pid->mf_t_e == TRIMF)
			N_mf_e = 3;
		else if (pid->mf_t_e == GAUSSMF)
			N_mf_e = 2;
		else if (pid->mf_t_e == TRAPMF)
			N_mf_e = 4;
		else
			N_mf_e = 3;

		pid->e_mf_paras = (float*)malloc(sizeof(float) * FUZZY_N * N_mf_e);
		for (int i = 0; i < FUZZY_N * N_mf_e; i++)
			pid->e_mf_paras[i] = paras[i];
		break;

	case 1:
		pid->mf_t_de = type;

		if (pid->mf_t_de == TRIMF)
			N_mf_de = 3;
		else if (pid->mf_t_de == GAUSSMF)
			N_mf_de = 2;
		else if (pid->mf_t_de == TRAPMF)
			N_mf_de = 4;
		else
			N_mf_de = 3;
		pid->de_mf_paras = (float*)malloc(sizeof(float) * FUZZY_N * N_mf_de);
		for (int i = 0; i < FUZZY_N * N_mf_de; i++)
			pid->de_mf_paras[i] = paras[i];
		break;

	case 2:

		pid->mf_t_Kp = type;

		if (pid->mf_t_Kp == TRIMF)
			N_mf_Kp = 3;
		else if (pid->mf_t_Kp == GAUSSMF)
			N_mf_Kp = 2;
		else if (pid->mf_t_Kp == TRAPMF)
			N_mf_Kp = 4;
		else
			N_mf_Kp = 3;

		pid->Kp_mf_paras = (float*)malloc(sizeof(float) * FUZZY_N * N_mf_Kp);
		for (int i = 0; i < FUZZY_N * N_mf_Kp; i++)
			pid->Kp_mf_paras[i] = paras[i];
		break;

	case 3:

		pid->mf_t_Ki = type;

		if (pid->mf_t_Ki == TRIMF)
			N_mf_Ki = 3;
		else if (pid->mf_t_Ki == GAUSSMF)
			N_mf_Ki = 2;
		else if (pid->mf_t_Ki == TRAPMF)
			N_mf_Ki = 4;
		else
			N_mf_Ki = 3;
		pid->Ki_mf_paras = (float*)malloc(sizeof(float) * FUZZY_N * N_mf_Ki);

		for (int i = 0; i < FUZZY_N * N_mf_Ki; i++)
			pid->Ki_mf_paras[i] = paras[i];
		break;

	case 4:
		pid->mf_t_Kd = type;
		if (pid->mf_t_Kd == TRIMF)
			N_mf_Kd = 3;
		else if (pid->mf_t_Kd == GAUSSMF)
			N_mf_Kd = 2;
		else if (pid->mf_t_Kd == TRAPMF)
			N_mf_Kd = 4;
		else
			N_mf_Kd = 3;

		pid->Kd_mf_paras = (float*)malloc(sizeof(float) * FUZZY_N * N_mf_Kd);
		for (int i = 0; i < FUZZY_N * N_mf_Kd; i++)
			pid->Kd_mf_paras[i] = paras[i];
		break;

	default: break;
	}
}

//三角隶属度函数
static float Trimf(float x,float a,float b,float c)
{
   float u;
   if(x>=a&&x<=b)
	   u=(x-a)/(b-a);
   else if(x>b&&x<=c)
	   u=(c-x)/(c-b);
   else
	   u=0.0;
   return u;
}
//正态隶属度函数
static float Gaussmf(float x,float ave,float sigma) 
{
	float u;
	u=exp(-pow(((x-ave)/sigma),2));
	return u;
}
//梯形隶属度函数
static float Trapmf(float x,float a,float b,float c,float d)
{
    float u;
	if(x>=a&&x<b)
		u=(x-a)/(b-a);
	else if(x>=b&&x<c)
        u=1;
	else if(x>=c&&x<=d)
		u=(d-x)/(d-c);
	else
		u=0;
	return u;
}
