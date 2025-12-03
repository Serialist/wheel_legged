/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef enum 
{
    PIECE_1 = 0,
    PIECE_2,
    PIECE_3
}PID_Piece_e;

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
	  fp32 kf;
	

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;
	
    fp32 f_f_out;
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

    fp32 now_target;
		
		fp32 per_target;
		
} pid_type_def;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err[3];
    fp32 Duf[3];
	
	
	
    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;

typedef struct
{
    PID_Piece_e piece_now;  //PID所处的阶段
    PID_Piece_e piece_last;

    //PID 三参数
    fp32 Kp[2];
    fp32 Ki[2];
    fp32 Kd[2];

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
    fp32 err_boundary;   //误差分段点
} piece2_pid_type_def;

typedef struct
{
    PID_Piece_e piece_now;  //PID所处的阶段
    PID_Piece_e piece_last;

    //PID 三参数
    fp32 Kp[3];
    fp32 Ki[3];
    fp32 Kd[3];

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
    fp32 err_boundary[2];   //误差分段点
} piece3_pid_type_def;

typedef struct PID
{
	float kp;//kp
	float ki;//ki
	float kd;//kd
	double pout;//k输出
	float iout;//i输出
	float dout;//d输出
	float now_error;			//当前误差
	float Last_error;			//上一次误差
	float Last_Last_error;	//上上次误差
	float sum_of_error;     //历史总误差
	float set;		//设置
	float now;		//当前
	float	last_fdb;
	float out;		//输出
 
  float MaxOutput;//PID输出限幅	
	float IntegralLimit;//I输出限幅	
	float plus;		//本次增量值
  float plus_out;	//增量式输出值plus_out = last_plus_out + plus
  float last_plus_out;//上次增量式输出值
	
	float Max_Error_Data;
	
	float Small_Error_Limit;
	
	int Set_Out_Mode;
	float Set_A;
	float Set_B;
	float Set_ratio;
	float Set_alpha;
	float Last_Ud;
	
	uint8_t Derivative_mode;
	float c1;
	float c2;
	float c3;
	float gama;

}MATH_PID;	


//前馈
typedef struct
{
	fp32 error[3];
	fp32 set_now;
	fp32 set_last;
	fp32 d_out;
	fp32 KF;
	fp32 Dbuf[3];
}Ff_type_def;	
//前馈







/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
//extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern fp32 PID_calc(pid_type_def *pid, fp32 set, fp32 ref);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);
float Gimbal_Math_pid_calc(MATH_PID	*pid, float now, float set);
fp32 PID_dead_zone_calc(pid_type_def *pid, fp32 set, fp32 ref,fp32 dead_par);
extern  fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
fp32 FeedForward_calc(Ff_type_def *ff,fp32 set_now,fp32 set_last);


#endif
