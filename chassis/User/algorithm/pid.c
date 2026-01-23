/************************
 * @file pid.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-09
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "pid.h"
#include "user_lib.h"
#include "math.h"

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
 * @brief pid struct data init
 *
 * @param pid PID结构数据指针
 * @param mode PID_POSITION:普通PID
 *             PID_DELTA: 差分PID
 * @param kp
 * @param ki
 * @param kd
 * @param max_out 最大输出
 * @param max_iout 最大积分输出
 */
void PID_init(PID_Typedef *pid, uint8_t mode, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

void PID_Set(PID_Typedef *pid, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
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
// fp32 PID_Calc(PID_Typedef *pid, fp32 ref, fp32 set)

fp32 PID_Calc(PID_Typedef *pid, fp32 set, fp32 ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
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

        //		if(fabs(pid->error[0]) <= pid->err_up)
        //		{
        //			pid->cnt_ki++;
        //		}
        //		else{
        //			pid->cnt_ki = 0;
        //		}
        //		if(pid->cnt_ki > pid->max_cnt_ki)
        //		{
        //			pid->Iout = 0;
        //			pid->cnt_ki = 0;
        //		}

        pid->out = pid->Pout + pid->Iout + pid->Dout;
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
    return pid->out;
}
float err_limit = 23.0f;
float kp_max = 35.0f;
float a_k = 1;
float b_k = 1;
int n_d = 3;
float kp;
fp32 ExpKp_PID_Calc(PID_Typedef *pid, fp32 set, fp32 ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    float E = 0.0f;
    if (pid->error[0] < 0)
    {
        if (pid->error[0] - pid->error[1] < 0)
        {
            E = a_k * fabs(pid->error[0]) - b_k * (pid->error[0] - pid->error[1]);
        }
        else
        {
            E = a_k * fabs(pid->error[0]) + b_k * (pid->error[0] - pid->error[1]);
        }
    }
    else
    {
        if (pid->error[0] - pid->error[1] < 0)
        {
            E = a_k * fabs(pid->error[0]) + b_k * (pid->error[0] - pid->error[1]);
        }
        else
        {
            E = a_k * fabs(pid->error[0]) - b_k * (pid->error[0] - pid->error[1]);
        }
    }

    // float a = (kp_max - pid->Kp) / pow(err_limit, n_d);
    // float kp = a * pow(fabs(pid->error[0]) / err_limit, n_d) + pid->Kp;

    float a = (kp_max - pid->Kp) / pow(err_limit, n_d);
    kp = a * pow(E, n_d) + pid->Kp;

    if (kp > kp_max)
        kp = kp_max;
    else if (kp < 0)
        kp = 0;

    pid->Pout = kp * pid->error[0];

    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    return pid->out;
}

fp32 Piece2_PID_Calc(piece2_pid_type_def *pid2, fp32 set, fp32 ref)
{
    if (pid2 == NULL)
    {
        return 0.0f;
    }

    pid2->error[2] = pid2->error[1];
    pid2->error[1] = pid2->error[0];
    pid2->set = set;
    pid2->fdb = ref;
    pid2->error[0] = set - ref;

    pid2->piece_last = pid2->piece_now;
    if (fabs(pid2->error[0]) > pid2->err_boundary)
        pid2->piece_now = PIECE_2;
    else
        pid2->piece_now = PIECE_1;

    if (pid2->piece_now != pid2->piece_last)
    {
        pid2->Iout = 0.0f;
    }

    pid2->Pout = pid2->Kp[pid2->piece_now] * pid2->error[0];
    pid2->Iout += pid2->Ki[pid2->piece_now] * pid2->error[0];
    pid2->Dbuf[2] = pid2->Dbuf[1];
    pid2->Dbuf[1] = pid2->Dbuf[0];
    pid2->Dbuf[0] = (pid2->error[0] - pid2->error[1]);
    pid2->Dout = pid2->Kd[pid2->piece_now] * pid2->Dbuf[0];

    LimitMax(pid2->Iout, pid2->max_iout);
    pid2->out = pid2->Pout + pid2->Iout + pid2->Dout;
    LimitMax(pid2->out, pid2->max_out);

    return pid2->out;
}

fp32 Piece3_PID_Calc(piece3_pid_type_def *pid3, fp32 set, fp32 ref)
{
    if (pid3 == NULL)
    {
        return 0.0f;
    }

    pid3->error[2] = pid3->error[1];
    pid3->error[1] = pid3->error[0];
    pid3->set = set;
    pid3->fdb = ref;
    pid3->error[0] = set - ref;

    pid3->piece_last = pid3->piece_now;
    fp32 err_abs = fabs(pid3->error[0]);
    if (err_abs > pid3->err_boundary[1])
        pid3->piece_now = PIECE_3;
    else if (err_abs > pid3->err_boundary[0] && err_abs <= pid3->err_boundary[1])
        pid3->piece_now = PIECE_2;
    else
        pid3->piece_now = PIECE_1;

    if (pid3->piece_now != pid3->piece_last)
    {
        pid3->Iout = 0.0f;
    }

    pid3->Pout = pid3->Kp[pid3->piece_now] * pid3->error[0];
    pid3->Iout += pid3->Ki[pid3->piece_now] * pid3->error[0];
    pid3->Dbuf[2] = pid3->Dbuf[1];
    pid3->Dbuf[1] = pid3->Dbuf[0];
    pid3->Dbuf[0] = (pid3->error[0] - pid3->error[1]);
    pid3->Dout = pid3->Kd[pid3->piece_now] * pid3->Dbuf[0];

    LimitMax(pid3->Iout, pid3->max_iout);
    pid3->out = pid3->Pout + pid3->Iout + pid3->Dout;
    LimitMax(pid3->out, pid3->max_out);

    return pid3->out;
}

fp32 anglePidCalc(PID_Typedef *pid, fp32 set, fp32 get, fp32 gyro)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = get;
    pid->error[0] = set - get;

    pid->Pout = pid->Kp * pid->error[0];

    pid->Iout += pid->Ki * pid->error[0];

    pid->Dout = pid->Kd * gyro;

    LimitMax(pid->Iout, pid->max_iout);

    pid->out = pid->Pout + pid->Iout + pid->Dout;

    LimitMax(pid->out, pid->max_out);

    return pid->out;
}

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
void PID_clear(PID_Typedef *pid)
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
