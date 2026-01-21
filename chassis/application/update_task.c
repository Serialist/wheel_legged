#include "update_task.h"
#include "bsp_delay.h"
#include "cmsis_os.h"
#include "data_transform.h"
#include "pid.h"
#include "INS_task.h"
#include <math.h>
#include "leg_conv_1.h"
#include "leg_spd_1.h"
#include "leg_pos_1.h"
#include "LQR_K.h"
#include "DM_VMC_test.h"

extern Chassis_t chassis;
extern Wheel_Leg_Target_t set;
extern struct VMC_Leg leg_l, leg_r;

void chassis_sys_calc(Chassis_t *ch);

void update_task(void const *argument)
{

  while (1)
  {
    chassis_sys_calc(&chassis);
    osDelay(3);
  }
}
