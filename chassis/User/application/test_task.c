/************************************************
 * @file test_task.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-09-10
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 ********************************/

/**************************************************************** include ****************************************************************/

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "remote_control.h"
#include "kalman_filter.h"
#include "vmc.h"
#include "bsp_can.h"
#include "chassismotor.h"
#include "bsp_buzzer.h"
#include "motor.h"

/**************************************************************** define ****************************************************************/

/******************************** 轮毂电机准确速度获取 ********************************/

#define TASK_PERIOD_MS 4                // 任务周期4ms
#define POSITION_RANGE 25.0f            // 位置范围25弧度（-12.5到12.5）
#define MAX_EXPECTED_VELOCITY 30.0f     // 最大预期速度(rad/s)
#define MIN_DELTA_POS_THRESH 0.001f     // 最小位置变化阈值(rad)
#define MIN_TIME_DELTA_MS 1             // 最小有效时间差(ms)
#define FILTER_ALPHA 0.3f               // 低通滤波系数
#define ZERO_VELOCITY_THRESH 0.01f      // 零速阈值(rad/s)
#define ZERO_DRIFT_COMPENSATION 0.0001f // 零漂补偿系数

/******************************** / 轮毂电机准确速度获取 ********************************/

/**************************************************************** struct ****************************************************************/

/**************************************************************** const ****************************************************************/

/**************************************************************** var ****************************************************************/

KalmanFilter_t vaEstimateKF; // 这是一个卡尔曼滤波器对象

VelocityCalculator vel_calc_r = {0}; // 速度计算器
VelocityCalculator vel_calc_l = {0}; // 速度计算器

float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f}; // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f,
                           0.0f, 0.5f}; // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f, 100.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f}; // 设置矩阵H为常量

extern INS_t INS;
extern struct Chassis_State chassis;
extern struct VMC_Leg leg_l, leg_r;
extern struct Wheel_Leg_Target set;

float vel_acc_l[2];
float vel_acc_r[2];
float vel_filter_l;
float vel_filter_r;
float vel_acc[2];

float speed[2];
uint32_t OBSERVE_TIME = 4; // 任务周期是4ms

float current_pos_r;
float motor_vel_r;
float current_pos_l;
float motor_vel_l;

float wr, wl = 0.0f;          // wheel 轮毂速度
float vrb = 0.0f, vlb = 0.0f; // vehicle
float aver_v = 0.0f;

float position_increment = 0;

/**************************************************************** proto ****************************************************************/

void Phase_Update(struct Chassis_State *ch);
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel);
// static float normalize_position(float position);
// static float calculate_velocity(VelocityCalculator *calc, float current_pos, uint32_t current_time_ms);

/**************************************************************** func ****************************************************************/

/************************************************
 * @brief os task
 *
 * @param argument
 ********************************/
void test_task(void const *argument)
{
  TickType_t xLastWakeTime;

  chassis.robo_status.status = ROBO_STATE_INIT;

  xLastWakeTime = xTaskGetTickCount();
  // 等待加速度收敛
  while (INS.ins_flag == 0)
  {
    osDelay(1);
  }

  xvEstimateKF_Init(&vaEstimateKF);

  // Buzzer_Preset();

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
    wl = motor_vel_l + INS.Gyro[0] + chassis.state.d_alphal;                                                              // 左轮速度
    vlb = wl * wheelRadius + leg_l.L0 * leg_l.d_theta * arm_cos_f32(leg_l.theta) + leg_l.d_L0 * arm_sin_f32(leg_l.theta); // 左机体速度

    // 互补滤波
    aver_v = (vrb + vlb) / 2.0f;                                      // 取平均
    xvEstimateKF_Update(&vaEstimateKF, INS.MotionAccel_n[1], aver_v); // ins 加速度 轮毂反馈速度 融合滤波

    // 原地自转的过程中v_filter和x_filter应该都是为0
    chassis.state.v_filter = vel_acc[0]; // 得到卡尔曼滤波后的速度

    // 速度死区
    // if (fabsf(chassis.state.v_filter) >= 0.005f)
    // {
    //   chassis.state.x_filter += chassis.state.v_filter * (TASK_PERIOD_MS * 0.001f);
    // }

    chassis.state.x_filter += chassis.state.v_filter * (TASK_PERIOD_MS * 0.001f);

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

// 轮毂电机速度更新补偿 //周期4ms

//// 区间映射函数
// static float normalize_position(float position)
//{
//   // 将位置映射到 [-12.5f, 12.5f] 区间
//   if (position > 12.5f)
//   {
//     position -= POSITION_RANGE; // POSITION_RANGE = 25.0f
//   }
//   else if (position < -12.5f)
//   {
//     position += POSITION_RANGE;
//   }
//   return position;
// }

//// 速度计算函数
// static float calculate_velocity(VelocityCalculator *calc, float current_pos, uint32_t current_time_ms)
//{
//   // 1. 第一次调用时初始化
//   if (!calc->initialized)
//   {
//     calc->prev_position = current_pos;
//     calc->prev_tick = current_time_ms;
//     calc->filtered_velocity = 0.0f;
//     calc->zero_drift_offset = 0.0f;
//     calc->initialized = 1;
//     return 0.0f;
//   }

//  // 2. 计算时间差 (处理时间戳溢出)
//  uint32_t time_delta_ms = current_time_ms - calc->prev_tick;

//  // 时间差无效情况处理
//  if (time_delta_ms < MIN_TIME_DELTA_MS || time_delta_ms > 100)
//  {
//    // 重置状态并返回上次滤波值
//    calc->prev_position = current_pos;
//    calc->prev_tick = current_time_ms;
//    return calc->filtered_velocity;
//  }

//  // 3. 计算位置差 (使用改进的环形位置处理)
//  float delta_pos_raw = current_pos - calc->prev_position;
//  float delta_pos = delta_pos_raw;

//  // 环形位置处理: 检测边界穿越并修正
//  if (fabsf(delta_pos_raw) > 12.5f)
//  {
//    if (delta_pos_raw > 0)
//    {
//      delta_pos -= POSITION_RANGE; // 正向穿越处理
//    }
//    else
//    {
//      delta_pos += POSITION_RANGE; // 反向穿越处理
//    }
//  }

//  // 4. 时间差转换为秒
//  float delta_time = time_delta_ms * 0.001f;

//  // 5. 计算原始速度
//  float raw_velocity = delta_pos / delta_time;

//  // 6. 自适应零速检测
//  float adaptive_zero_thresh = ZERO_VELOCITY_THRESH;

//  // 当速度较低时，增加零速阈值减少噪声
//  if (fabsf(calc->filtered_velocity) < 0.5f)
//  {
//    adaptive_zero_thresh = ZERO_VELOCITY_THRESH * 2.0f;
//  }

//  // 零速检测: 消除微小变化引起的噪声
//  if (fabsf(raw_velocity) < adaptive_zero_thresh)
//  {
//    raw_velocity = 0.0f;
//  }

//  // 7. 速度限幅: 确保在合理范围内
//  if (fabsf(raw_velocity) > MAX_EXPECTED_VELOCITY)
//  {
//    raw_velocity = (raw_velocity > 0) ? MAX_EXPECTED_VELOCITY : -MAX_EXPECTED_VELOCITY;
//  }

//  // 8. 自适应滤波: 低速时使用更强滤波
//  float adaptive_alpha = FILTER_ALPHA;

//  // 当速度较低时，增加滤波强度
//  if (fabsf(calc->filtered_velocity) < 1.0f)
//  {
//    adaptive_alpha = FILTER_ALPHA * 0.5f;
//  }

//  // 一阶低通滤波: 平滑速度输出
//  calc->filtered_velocity = adaptive_alpha * raw_velocity +
//                            (1 - adaptive_alpha) * calc->filtered_velocity;

//  // 9. 零漂补偿: 持续微小补偿
//  if (fabsf(calc->filtered_velocity) < ZERO_VELOCITY_THRESH)
//  {
//    calc->filtered_velocity *= (1.0f - ZERO_DRIFT_COMPENSATION);
//  }

//  // 10. 更新状态
//  calc->prev_position = current_pos;
//  calc->prev_tick = current_time_ms;

//  // 返回滤波后的速度
//  return calc->filtered_velocity;
//}
