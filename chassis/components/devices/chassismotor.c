/************************
 * @file chassismotor.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-12-02
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#include "chassismotor.h"
#include "stdint.h"
#include "can.h"
#include "pid.h"
#include "chassis_task.h"
#include "math.h"

// 电机CAN发送//
CAN_TxHeaderTypeDef TX_message;
uint8_t send_data_message[8];
uint32_t send_mailbox1;
uint32_t send_mailbox2;
uint8_t can_tx_data[8];

///////////////////////////////////////////////////////////////////////////////////////////////////////

// dji motors//

void TransmitChassisMotorCurrent(int16_t o1, int16_t o2)
{
  uint32_t send_mail_box = CAN_TX_MAILBOX0;
  TX_message.StdId = 0x1FF;
  TX_message.IDE = CAN_ID_STD;
  TX_message.RTR = CAN_RTR_DATA;
  TX_message.DLC = 0x08;

  can_tx_data[0] = (uint8_t)(o1 >> 8); // 将16位数据分为高8位和低8位
  can_tx_data[1] = (uint8_t)o1;
  can_tx_data[2] = (uint8_t)(o2 >> 8);
  can_tx_data[3] = (uint8_t)o2;
  HAL_CAN_AddTxMessage(&hcan1, &TX_message, can_tx_data, &send_mail_box);
}

// cubemars motors//
// servo mode
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
{
  uint8_t i = 0;
  if (len > 8)
  {
    len = 8;
  }
  TX_message.StdId = 0;
  TX_message.IDE = CAN_ID_EXT;
  TX_message.ExtId = id;
  TX_message.RTR = CAN_RTR_DATA;
  TX_message.DLC = len;
  for (i = 0; i < len; i++)
  {
    can_tx_data[i] = data[i];
  }
  HAL_CAN_AddTxMessage(&hcan1, &TX_message, can_tx_data, &send_mailbox2); // CAN 口发送 TxMessage 数据
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index)
{
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void comm_can_set_current(uint8_t controller_id, float current)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
  comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                        buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_RPM << 8),
                        buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
  comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_POS << 8),
                        buffer, send_index);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode)
{
  int32_t send_index = 0;
  uint8_t buffer;
  buffer = set_origin_mode;
  comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8),
                        &buffer, send_index);
}

// motion controller
void controller_init(uint8_t id)
{
  uint32_t send_mail_box = CAN_TX_MAILBOX0;
  TX_message.StdId = id;
  TX_message.IDE = CAN_ID_STD;
  TX_message.RTR = CAN_RTR_DATA;
  TX_message.DLC = 0x08;

  can_tx_data[0] = 0xFF;
  can_tx_data[1] = 0xFF;
  can_tx_data[2] = 0xFF;
  can_tx_data[3] = 0xFF;
  can_tx_data[4] = 0xFF;
  can_tx_data[5] = 0xFF;
  can_tx_data[6] = 0xFF;
  can_tx_data[7] = 0xFC;

  HAL_CAN_AddTxMessage(&hcan1, &TX_message, can_tx_data, &send_mail_box);
}

void controller_close(uint8_t id)
{
  TX_message.StdId = id;
  TX_message.IDE = CAN_ID_STD;
  TX_message.RTR = CAN_RTR_DATA;
  TX_message.DLC = 0x08;

  can_tx_data[0] = 0xFF;
  can_tx_data[1] = 0xFF;
  can_tx_data[2] = 0xFF;
  can_tx_data[3] = 0xFF;
  can_tx_data[4] = 0xFF;
  can_tx_data[5] = 0xFF;
  can_tx_data[6] = 0xFF;
  can_tx_data[7] = 0xFD;

  HAL_CAN_AddTxMessage(&hcan1, &TX_message, can_tx_data, &send_mailbox1);
}

void controller_setorigin(uint8_t id)
{
  TX_message.StdId = id;
  TX_message.IDE = CAN_ID_STD;
  TX_message.RTR = CAN_RTR_DATA;
  TX_message.DLC = 0x08;

  can_tx_data[0] = 0xFF;
  can_tx_data[1] = 0xFF;
  can_tx_data[2] = 0xFF;
  can_tx_data[3] = 0xFF;
  can_tx_data[4] = 0xFF;
  can_tx_data[5] = 0xFF;
  can_tx_data[6] = 0xFF;
  can_tx_data[7] = 0xFE;

  HAL_CAN_AddTxMessage(&hcan1, &TX_message, can_tx_data, &send_mailbox1);
}

/************************
 * @brief float 转 uint，并限幅
 *
 * @param x
 * @param x_min
 * @param x_max
 * @param bits
 * @return int
 ************************/
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  if (x < x_min)
    x = x_min;
  else if (x > x_max)
    x = x_max;
  return (int)((x - x_min) * ((float)((1 << bits) / span)));
}

/************************
 * @brief 发送电机 MIT 模式控制
 *
 * @param id 电机 ID
 * @param p_des 目标位置
 * @param v_des 目标速度
 * @param kp 内置 PID
 * @param kd 内置 PID
 * @param t_ff 力矩前馈
 ************************/
void pack_cmd(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff)
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];
  uint32_t send_mail_box = CAN_TX_MAILBOX0;

  int v_int;
  int t_int;

  /// limit data to be within bounds ///

#define FLIMIT(num, min, max) (fminf(fmaxf((min), (num)), (max)))

  p_des = FLIMIT(p_des, P_MIN, P_MAX);
  v_des = FLIMIT(v_des, V_MIN, V_MAX);
  kp = FLIMIT(kp, Kp_MIN, Kp_MAX);
  kd = FLIMIT(kd, Kd_MIN, Kd_MAX);
  t_ff = FLIMIT(t_ff, T_MIN, T_MAX);

  /// convert floats to unsigned ints ///
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);

  if (id <= 4)
  {
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  }
  else
  {
    v_int = float_to_uint(v_des, V_MIN_2, V_MAX_2, 12);
    t_int = float_to_uint(t_ff, T_MIN_2, T_MAX_2, 12);
  }

  int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
  int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);

  /// pack ints into the can buffer ///
  header.StdId = id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 0x08;

  data[0] = p_int >> 8;
  data[1] = p_int & 0xFF;
  data[2] = v_int >> 4;
  data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  data[4] = kp_int & 0xFF;
  data[5] = kd_int >> 4;
  data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  data[7] = t_int & 0xff;

  HAL_CAN_AddTxMessage(&hcan1, &header, data, &send_mail_box);
}

float Uint_To_Float(int x_int, float x_min, float x_max, int bits)
{
	// 将无符号整数转换为浮点数，给定范围和位数
	return ((float)x_int) * (x_max - x_min) / ((float)((1 << bits) - 1)) + x_min;
}

void Motor_AK_MIT_Decode(struct Motor_AK_Rx_Data *rxData, uint8_t data[8], float pMax, float vMax, float tMax)
{
  rxData->id = data[0];
  rxData->position = Uint_To_Float((int)(data[1] << 8) | (data[2]), -pMax, pMax, 16);
  rxData->speed = Uint_To_Float((int)(data[3] << 4) | (data[4] >> 4), -vMax, vMax, 12);
  rxData->torque = Uint_To_Float((int)((data[4] & 0xF) << 8) | (data[5]), -tMax, tMax, 12);
  rxData->temp = (float)data[6] - 40.0f;
}
