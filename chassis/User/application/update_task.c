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

/* ================================================================ macro ================================================================*/

/* ================================================================ typedef ================================================================*/

/* ================================================================ variable ================================================================*/

extern struct Chassis_State chassis;
extern struct Wheel_Leg_Target set;

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
    /// @brief ¼ì²â
    RC_Offline_Detection(&rc_ctrl, 1);
    Motor_Offline_Detection(&motor_status, 1);

    /// @brief ¼±Í£ÅÐ¶Ï
    if (RC_IS_OFFLINE(&rc_ctrl) ||
        MOTOR_IS_OFFLINE(&motor_status) ||
        (chassis.state.thetal >= (PI / 2)) ||
        (chassis.state.thetar >= (PI / 2)))
    {
      chassis.robo_status.status = ROBO_STATE_EMERGENCY;
    }
    else
    {
      chassis.robo_status.status = ROBO_STATE_RUN;
    }

    osDelay(1);
  }

  while (1)
  {
    osDelay(3000);
    chassis_sys_calc(&chassis);
    Phase_Update(&chassis);
    osDelay(1);
  }
}
