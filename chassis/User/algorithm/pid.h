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
} PID_Piece_e;

typedef struct
{
  uint8_t mode;
  // PID 三参数
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;  // 最大输出
  fp32 max_iout; // 最大积分输出

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];  // 微分项 0最新 1上一次 2上上次
  fp32 error[3]; // 误差项 0最新 1上一次 2上上次

  float err_up;
  int cnt_ki;
  int max_cnt_ki;
} PID_Typedef;

typedef struct
{
  PID_Piece_e piece_now; // PID所处的阶段
  PID_Piece_e piece_last;

  // PID 三参数
  fp32 Kp[2];
  fp32 Ki[2];
  fp32 Kd[2];

  fp32 max_out;  // 最大输出
  fp32 max_iout; // 最大积分输出

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];      // 微分项 0最新 1上一次 2上上次
  fp32 error[3];     // 误差项 0最新 1上一次 2上上次
  fp32 err_boundary; // 误差分段点
} piece2_pid_type_def;

typedef struct
{
  PID_Piece_e piece_now; // PID所处的阶段
  PID_Piece_e piece_last;

  // PID 三参数
  fp32 Kp[3];
  fp32 Ki[3];
  fp32 Kd[3];

  fp32 max_out;  // 最大输出
  fp32 max_iout; // 最大积分输出

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];         // 微分项 0最新 1上一次 2上上次
  fp32 error[3];        // 误差项 0最新 1上一次 2上上次
  fp32 err_boundary[2]; // 误差分段点
} piece3_pid_type_def;

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
void PID_init(PID_Typedef *pid, uint8_t mode, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout);

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
// extern fp32 PID_Calc(PID_Typedef *pid, fp32 ref, fp32 set);
fp32 PID_Calc(PID_Typedef *pid, fp32 set, fp32 ref);
void PID_Set(PID_Typedef *pid, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout);
fp32 anglePidCalc(PID_Typedef *pid, fp32 set, fp32 get, fp32 gyro);
fp32 Piece2_PID_Calc(piece2_pid_type_def *pid, fp32 set, fp32 ref);
fp32 Piece3_PID_Calc(piece3_pid_type_def *pid, fp32 set, fp32 ref);
fp32 ExpKp_PID_Calc(PID_Typedef *pid, fp32 set, fp32 ref);
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
extern void PID_clear(PID_Typedef *pid);

#endif
