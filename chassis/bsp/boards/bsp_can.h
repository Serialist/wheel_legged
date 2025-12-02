#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "struct_typedef.h"
#include "chassismotor.h"
#include "dji_motor.h"

#define yaw1 0
#define O2 1
#define CHASSIS_Y_ID 0x304
#define CHASSIS_Z_ID 0x303
#define GIMBAL_TO_CHASSIS 0x302
#define M3508_WHELL_ID_1 0x206
#define M3508_WHELL_ID_2 0x207

#define YAW_ID 0x205
// #define MOMENTUM_WHEEL_1_ID 0x203
// #define MOMENTUM_WHEEL_2_ID 0x204
#define M_PI_2 1.57079632679489661923
#define SUPER_POWER 0x211

extern DJI_RxData_Def_t m3508[2];

// 获取电机数据
// void GetMotorFdb(Motor_Feedback_t* motor, uint32_t* trans_time, uint8_t num, uint8_t start_num);
void GetgimbalmotorFdb(MOTOR_RECEIVE_DATA *motor);
void GetAKMotor1Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2);
void GetAKMotor2Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2);
void GetAKMotor3Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2);
void GetAKMotor4Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2);
void GetAKMotor5Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2);
void GetAKMotor6Fdb(AK_motor_fdb_t *motor1, AK_motor_ctrl_fdb_t *motor2);

// 获取底盘三个方向速度设定值
void GetChassisYSpeedSet(fp32 *set);
void GetChassisZSpeedSet(fp32 *set);

void can_filter_init(void);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void TransmitChassisMotorCurrent(int16_t o1, int16_t o2);
void TransmitSuperPower(float power_);

void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);

void Sent_YAW_Data(void);

/***********************************************
 **                                           **
 **  ooooooooo.   oooooooooooo   .oooooo.     **
 **  `888   `Y88. `888'     `8  d8P'  `Y8b    **
 **   888   .d88'  888         888            **
 **   888ooo88P'   888oooo8    888            **
 **   888`88b.     888    "    888     ooooo  **
 **   888  `88b.   888       o `88.    .88'   **
 **  o888o  o888o o888ooooood8  `Y8bood8P'    **
 **                                           **
 ***********************************************/

#define REGISTER_FUNC_RUN_MAX 8

typedef struct Reg_Def_t
{
    uint8_t len;
    void (*Func[REGISTER_FUNC_RUN_MAX])(uint8_t flag, uint8_t *data);
} Reg_Def_t;

extern Reg_Def_t can_callback_reg;

void Reg_Init(Reg_Def_t *reg);
uint8_t Reg_Add(Reg_Def_t *reg, void (*Func)(uint8_t flag, uint8_t *data));
void Reg_Clear(Reg_Def_t *reg);
void Reg_Run(Reg_Def_t *reg, uint8_t flag, uint8_t *data);

#endif
