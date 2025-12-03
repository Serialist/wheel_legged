/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


#include "pid.h"
#include "data_transform.h"
#include "stdio.h"
#include <stdlib.h>
#include <math.h>
#include "user_lib.h"
double fabs(double);

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

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
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

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
//fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
 fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{ 
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->err[0] = set - get;
	
	  pid->err[0] = ang_format(pid->err[0]);
    pid->Pout = pid->kp * pid->err[0];
    pid->Iout += pid->ki * pid->err[0];
		pid->Duf[2]=pid->Duf[1];
		pid->Duf[1]=pid->Duf[0];
		pid->Duf[0]=pid->err[0]-pid->err[1];
    pid->Dout = pid->kd * pid->Duf[0];
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out; 
}

//大二上寒假前解决pitch轴运动到位置后微微向上，死区pid
fp32 PID_dead_zone_calc(pid_type_def *pid, fp32 set, fp32 ref,fp32 dead_par)
{
 if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
	
    pid->now_target=pid->set = set;
		//pid->per_target=0;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		if(fabs(pid->error[0])<dead_par)
		{
		return 0;
		}
		else
		{
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
			if(ABS(pid->error[0])>=3)
			{
			pid->f_f_out=pid->kf*(pid->now_target-pid->per_target)*(pid->error[0]/__fabs(pid->error[0]));
			}
			else
			{pid->f_f_out=0;}
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout+pid->f_f_out;
        LimitMax(pid->out, pid->max_out);
			
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
		pid->per_target=0;
    return pid->out;
	}
}

fp32 PID_calc(pid_type_def *pid, fp32 set, fp32 ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
		pid->now_target=set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
			  pid->f_f_out=(pid->now_target-pid->per_target);
        pid->out = pid->Pout + pid->Iout + pid->Dout+pid->kf*pid->f_f_out;
        LimitMax(pid->out, pid->max_out);
			  pid->per_target=pid->now_target;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }		
		//pid->now_target=0;`
		
    return pid->out;
}
//1.11前馈控制//
fp32 F_PID_calc(pid_type_def *pid, fp32 set, fp32 ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
		pid->now_target=set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		if(__fabs(pid->error[0])/pid->Kp)
    if (pid->mode == PID_POSITION)
    {   
			  
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
			  pid->f_f_out=(pid->now_target-pid->per_target);
        pid->out = pid->Pout + pid->Iout + pid->Dout+0;//pid->kf*pid->f_f_out;
        LimitMax(pid->out, pid->max_out);
			  pid->per_target=pid->now_target;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }		
		//pid->now_target=0;`
		
    return pid->out;
}



//1.11前馈控制//

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

float Gimbal_Math_pid_calc(MATH_PID	*pid, float now, float set)
{
    pid->now = now;
    pid->set = set;

		pid->now_error = pid->set - pid->now;	//set - measure
		pid->now_error = ang_format(pid->now_error);
		if((pid->now_error < pid->Small_Error_Limit) && (pid->now_error > -pid->Small_Error_Limit))
			return 0;
    
		pid->pout = pid->kp * pid->now_error;
		if(((pid->Set_Out_Mode == 1) && (pid->now_error > 0)) || ((pid->Set_Out_Mode == -1) && (pid->now_error < 0)));
		else
		{
			if(fabs(pid->now_error) <= pid->Set_B)
				pid->sum_of_error += pid->now_error;
			else if(fabs(pid->now_error) >= (pid->Set_B + pid->Set_A))
				pid->sum_of_error = 0;
			else
			{
				pid->Set_ratio = (pid->Set_A + pid->Set_B - fabs(pid->now_error)) / pid->Set_A;
				pid->sum_of_error += pid->Set_ratio * pid->now_error;
			}
		}
		
		//变速积分
		pid->iout = pid->ki * pid->sum_of_error;		
		
//		if(pid->Derivative_mode==FIRST)
//		{
//			//微分先行
//			pid->c1=(pid->Set_alpha*pid->kd)/(pid->Set_alpha*pid->kd+pid->kp);
//			pid->c2=(pid->kd+pid->kp)/(pid->Set_alpha*pid->kd+pid->kp);
//			pid->c3=(pid->kd)/(pid->Set_alpha*pid->kd+pid->kp);
//		
//			pid->dout=pid->c1*pid->Last_Ud+pid->c2*pid->now-pid->c3*pid->last_fdb;
//		}
//		else if(pid->Derivative_mode==PART)			//不完全微分	
			pid->dout = pid->kd * (pid->now_error - pid->Last_error) * (1 - pid->Set_alpha) + pid->Set_alpha * pid->Last_Ud;

		pid->out = pid->pout + pid->iout + pid->dout;		

		if(pid->out > pid->MaxOutput)
		{
			pid->out = pid->MaxOutput;
			pid->Set_Out_Mode = 1;
		}
		else
			pid->Set_Out_Mode = 0;
		
		if(pid->out < -pid->MaxOutput)
		{
			pid->out = -pid->MaxOutput;
			pid->Set_Out_Mode = -1;
		}
		else
			pid->Set_Out_Mode = 0;
		
		
		pid->last_fdb=pid->now;
		pid->Last_Ud = pid->dout;
		pid->Last_Last_error= pid->Last_error;
		pid->Last_error = pid->now_error;


		return pid->out;
}
// float Feed_Forward_pid(pid_type_def *pid)
//{
//float f_f_error=pid->error[0];
//    
//}

//fp32 PID_calc_Kp(pid_type_def *pid, fp32 set, fp32 ref)



//前馈控制2025.1.10摘自VGD电控框架模版//


fp32 FeedForward_calc(Ff_type_def *ff,fp32 set_now,fp32 set_last)
{
    if (ff == NULL)
    {
        return 0.0f;
    }

    ff->error[2] = ff->error[1];
    ff->error[1] = ff->error[0];
    ff->set_now= set_now;
    ff->set_last = set_last;
    ff->error[0] = set_now - set_last;
   
     
        ff->Dbuf[2] = ff->Dbuf[1];
        ff->Dbuf[1] = ff->Dbuf[0];
        ff->Dbuf[0] = (ff->error[0] - ff->error[1]);
        ff->d_out = ff->KF * ff->Dbuf[0];

      

       LimitMax(ff->d_out, 50);
   
    return ff->d_out;
}
//前馈控制2025.1.10摘自VGD电控框架模版//
