/************************
 * @file update_task.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-12-15
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

/* ================================================================ import ================================================================*/

#include "update_task.h"
#include "chassis_task.h"
#include "robo_behavior.h"
#include "remote_control.h"
#include "dji_motor.h"
#include "motor.h"
#include "cmsis_os.h"
#include "vmc.h"

/* ================================================================ macro ================================================================*/

/* ================================================================ typedef ================================================================*/

/* ================================================================ variable ================================================================*/

extern struct Chassis_State chassis;
extern struct Wheel_Leg_Target set;
extern struct VMC_Leg leg_l, leg_r;

/* ================================================================ prototype ================================================================*/

void chassis_sys_calc(struct Chassis_State *ch);
void Phase_Update(struct Chassis_State *ch);

/* ================================================================ function ================================================================*/

/**
 * @brief
 *
 * @param argument
 */
void Update_Task(void const *argument)
{
  uint16_t i;

  for (i = 0; i < 1000; i += 1)
  {
    /// @brief 检测状态
    RC_Offline_Detection(&rc_ctrl, 1);
    Motor_Offline_Detection(&motor_status, 1);

    /// @brief 急停判断
    if (RC_IS_OFFLINE(&rc_ctrl) ||
        MOTOR_IS_OFFLINE(&motor_status) ||
        (chassis.state.thetal >= (PI / 2)) ||
        (chassis.state.thetar >= (PI / 2)))
    {
      chassis.robo_status.status = ROBO_STATE_EMERGENCY;

      // 如果电机离线，尝试重新使能电机
      if (MOTOR_IS_OFFLINE(&motor_status))
        Motor_Enable();
    }
    else
    {
      chassis.robo_status.status = ROBO_STATE_RUN;
    }

    osDelay(1);
  }

  while (1)
  {
    chassis_sys_calc(&chassis);
    Phase_Update(&chassis);

    /* ================================================================ 安全检测 ================================================================ */

    // 检测
    RC_Offline_Detection(&rc_ctrl, 1);
    Motor_Offline_Detection(&motor_status, 1);

    // 急停判断
    if (RC_IS_OFFLINE(&rc_ctrl) || MOTOR_IS_OFFLINE(&motor_status) || (chassis.state.thetal >= (PI / 2)) || (chassis.state.thetar >= (PI / 2)))
    {
      chassis.robo_status.status = ROBO_STATE_EMERGENCY;

      // 尝试重新使能电机
      Motor_Enable();
    }
    // 左摇杆在下时尝试手动恢复
    else if (chassis.rc_dt7.rc.s[S_L] == DOWN)
    {
      chassis.robo_status.status = ROBO_STATE_RUN;
    }

    // 状态清零
    if (chassis.rc_dt7.rc.s[S_L] == MID)
    {
      set.position_set = chassis.state.x_filter;
      set.yaw = chassis.IMU_DATA.total_yaw;
    }

    /* ================================================================ / 安全检测 ================================================================ */

    osDelay(1);
  }
}

void chassis_sys_calc(struct Chassis_State *ch)
{
  leg_l.phi1 = PI / 2.0f - ch->ak_fdb_ctrl[3].motor_ctrlpos + 4.25577f;
  leg_l.phi4 = PI / 2.0f - ch->ak_fdb_ctrl[2].motor_ctrlpos - 4.1704f;
  leg_r.phi1 = PI / 2.0f - ch->ak_fdb_ctrl[0].motor_ctrlpos + 1.0744f;
  leg_r.phi4 = PI / 2.0f - ch->ak_fdb_ctrl[1].motor_ctrlpos - 1.0363f;

  VMC_calc_1(&leg_l, &chassis, 3.0f / 1000.0f);
  VMC_calc_1(&leg_r, &chassis, 3.0f / 1000.0f);
}
