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
extern struct Wheel_Leg_Target set;
extern struct VMC_Leg leg_l, leg_r;

static int32_t last_pos = 0; // 唯一状态变量
float v;
/**********  函数声明 开始 *************/
void chassis_sys_calc(Chassis_t *ch);
void phase_update(Chassis_t *ch);
float GetSpeed(float current_pos);
/**********  函数声明 结束 *************/

void update_task(void const *argument)
{

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(3));
    chassis_sys_calc(&chassis);
    phase_update(&chassis);
    osDelay(1);
  }
}

// 轮腿系统参量计算

// 姿态更新

float GetSpeed(float current_pos)
{
  float speed = (current_pos - last_pos) * 200.0f; // 假设200Hz固定频率
  last_pos = current_pos;
  return (speed * speed < 0.0025f) ? 0.0f : speed; // 平方比较避免浮点abs
}
