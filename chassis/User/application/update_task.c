#include "update_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"

extern Chassis_t chassis;

void chassis_sys_calc(Chassis_t *ch);

void update_task(void const *argument)
{

  while (1)
  {
    chassis_sys_calc(&chassis);
    osDelay(3);
  }
}
