/**
 * @file observer.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-01-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

/* ================================================================ include ================================================================ */

#include "observer.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "dt7.h"
#include "kalman_filter.h"
#include "vmc-dm.h"
#include "bsp_can.h"
#include "chassismotor.h"
#include "bsp_buzzer.h"
#include "motor.h"

/* ================================================================ macro ================================================================ */

#define TASK_PERIOD_MS 3 // 任务周期

/* ================================================================ typedef ================================================================ */

/* ================================================================ variable ================================================================ */

KalmanFilter_t vaEstimateKF; // 这是一个卡尔曼滤波器对象

float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f}; // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f,
                           0.0f, 0.5f}; // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f, 100.0f};

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f}; // 设置矩阵H为常量

extern INS_t INS;
extern Chassis_t chassis;
extern VMC_t leg_l, leg_r;
extern Wheel_Leg_Target_t set;

float vel_acc[2];

float motor_vel_r;
float motor_vel_l;

float wr, wl = 0.0f;          // wheel 轮毂速度
float vrb = 0.0f, vlb = 0.0f; // vehicle
float aver_v = 0.0f;

float position_increment = 0;

/* ================================================================ prototype ================================================================ */

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel);

/* ================================================================ function ================================================================ */

/************************************************
 * @brief os task
 *
 * @param argument
 ********************************/
void observer(void const *argument)
{
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();
  // 等待加速度收敛
  while (INS.ins_flag == 0)
  {
    osDelay(1);
  }

  xvEstimateKF_Init(&vaEstimateKF);

  while (1)
  {
    uint32_t current_time_ms = HAL_GetTick();

    motor_vel_r = DJI_MOTOR_SPEED(&m3508[0]);
    motor_vel_l = -DJI_MOTOR_SPEED(&m3508[1]);

    // 机体速度观测器

    // 右
    wr = motor_vel_r + INS.Gyro[0] + leg_r.d_alpha;                                                                       // 右轮速度
    vrb = wr * wheelRadius + leg_r.L0 * leg_r.d_theta * arm_cos_f32(leg_r.theta) + leg_r.d_L0 * arm_sin_f32(leg_r.theta); // 右机体速度

    // 左
    wl = motor_vel_l + INS.Gyro[0] + leg_l.d_alpha;                                                                       // 左轮速度
    vlb = wl * wheelRadius + leg_l.L0 * leg_l.d_theta * arm_cos_f32(leg_l.theta) + leg_l.d_L0 * arm_sin_f32(leg_l.theta); // 左机体速度

    // 总体互补滤波
    aver_v = (vrb + vlb) / 2.0f;                                      // 取平均
    xvEstimateKF_Update(&vaEstimateKF, INS.MotionAccel_n[1], aver_v); // ins 加速度 轮毂反馈速度 融合滤波

    // 原地自转的过程中v_filter和x_filter应该都是为0
    chassis.state.v_filter = vel_acc[0]; // 得到卡尔曼滤波后的速度

    position_increment = chassis.state.v_filter * (TASK_PERIOD_MS * 0.001f);

    chassis.state.x_filter += position_increment;

    /* ================================================================ 安全检测 ================================================================ */

    /// @brief 检测
    // RC_Offline_Detection(&rc_ctrl, TASK_PERIOD_MS);
    // Motor_Offline_Detection(&motor_status, TASK_PERIOD_MS);
    /// @brief 急停判断
    // if (RC_IS_OFFLINE(&rc_ctrl) || MOTOR_IS_OFFLINE(&motor_status) || (leg_l.theta >= (PI / 2)) || (leg_r.theta >= (PI / 2)))
    // {
    //   chassis.robo_status.status = ROBO_STATUS_EMERGENCY;
    // }

    /* ================================================================ / 安全检测 ================================================================ */

    // 精确周期控制
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

// 达妙例程速度观测器//

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
  Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

  memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
  memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
  memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
  memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
  memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{
  // 卡尔曼滤波器测量值更新
  EstimateKF->MeasuredVector[0] = vel; // 测量速度
  EstimateKF->MeasuredVector[1] = acc; // 测量加速度

  // 卡尔曼滤波器更新函数
  Kalman_Filter_Update(EstimateKF);

  // 提取估计值
  for (uint8_t i = 0; i < 2; i++)
  {
    vel_acc[i] = EstimateKF->FilteredValue[i];
  }
}
