#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H
#include "struct_typedef.h"
#define FUZZY_N 7

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

//隶属度函数类型
typedef enum{
	TRIMF,
	GAUSSMF,
	TRAPMF
}MF_Type_e;

typedef struct
{
	float target;  //系统的控制目标
	float actual;  //采样获得的实际值
	float e;       //误差
	float e_pre_1; //上一次的误差
	float e_pre_2; //上上次的误差
	float de;      //误差的变化率
	float emax;    //误差基本论域上限
	float demax;   //误差辩化率基本论域的上限
	float delta_Kp_max;   //delta_kp输出的上限
	float delta_Ki_max;   //delta_ki输出上限
	float delta_Kd_max;   //delta_kd输出上限
	float Ke;      //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	float Kde;     //Kde=n/demax,量化论域为[-3,-2,-1,0,1,2,3]
	float Ku_p;    //Ku_p=Kpmax/n,量化论域为[-3,-2,-1,0,1,2,3]
	float Ku_i;    //Ku_i=Kimax/n,量化论域为[-3,-2,-1,0,1,2,3]
	float Ku_d;    //Ku_d=Kdmax/n,量化论域为[-3,-2,-1,0,1,2,3]
	int Kp_rule_matrix[FUZZY_N][FUZZY_N];//Kp模糊规则矩阵
	int Ki_rule_matrix[FUZZY_N][FUZZY_N];//Ki模糊规则矩阵
	int Kd_rule_matrix[FUZZY_N][FUZZY_N];//Kd模糊规则矩阵
	MF_Type_e mf_t_e;       //e的隶属度函数类型
	MF_Type_e mf_t_de;      //de的隶属度函数类型
	MF_Type_e mf_t_Kp;      //kp的隶属度函数类型
	MF_Type_e mf_t_Ki;      //ki的隶属度函数类型
	MF_Type_e mf_t_Kd;      //kd的隶属度函数类型
	float *e_mf_paras; //误差的隶属度函数的参数
	float *de_mf_paras;//误差的偏差隶属度函数的参数
	float *Kp_mf_paras; //kp的隶属度函数的参数
	float *Ki_mf_paras; //ki的隶属度函数的参数
	float *Kd_mf_paras; //kd的隶属度函数的参数
	
	float Kp0;
	float Ki0;
	float Kd0;
	
	float Kp;
	float Ki;
	float Kd;
	float delta_Kp;
	float delta_Ki;
	float delta_Kd;
	float Kp_max;
	float Kp_min;
	float Ki_max;
	float Ki_min;
	float Kd_max;
	float Kd_min;
	
	float max_out;  //最大输出
    float max_iout; //最大积分输出
	
	float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
	float error[3]; //误差项 0最新 1上一次 2上上次
}Fuzzy_PID_t;


void FuzzyPIDInit(Fuzzy_PID_t* pid, float e_max,float de_max,float kp_max,float ki_max,float kd_max,float Kp0,float Ki0,float Kd0);
void SetMf(Fuzzy_PID_t* pid, 
			MF_Type_e mf_type_e,float *e_mf,
			MF_Type_e mf_type_de,float *de_mf,
			MF_Type_e mf_type_Kp,float *Kp_mf,
		    MF_Type_e mf_type_Ki,float *Ki_mf,
			MF_Type_e mf_type_Kd,float *Kd_mf);
//设置模糊规则Matrix
void SetRuleMatrix(Fuzzy_PID_t* pid, int kp_m[FUZZY_N][FUZZY_N], int ki_m[FUZZY_N][FUZZY_N], int kd_m[FUZZY_N][FUZZY_N]);
//实现模糊控制
float FuzzyPIDCalc(Fuzzy_PID_t* pid, float set, float fdb, float distance);

#endif
