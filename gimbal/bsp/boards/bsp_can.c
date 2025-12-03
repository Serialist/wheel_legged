
#include "cmsis_os.h"
#include "can.h"
#include "constrain_calc.h"
#include "referee.h"
#include "bsp_can.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_Feedback_t gimbal_motor_fdb[2];
Motor_Feedback_t shoot_FR_motor_fdb[2];
Motor_Feedback_t shoot_ST_motor_fdb;

Referee  Chassis_Can2_Referee;
fp32 motor_relative_angle_fdb[3];
fp32 Y_yaw_motor_relative_angle_fdb;
fp32 one_head_angle;
uint32_t gimbal_motor_time[4];



float chassis_speed[3];
/*	函数作用：初始化CAN通信的过滤器
参数：无
返回值：无
	函数注释：配置CAN1和CAN2的过滤器，启动CAN通信，并激活接收中断，我们学校在硬件过滤上
都采取直接取默认值的方式，后续可能要修改，我们都采用软件过滤*/
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

int can1_d = 0, can2_d =0;
int std_d = 0;
HAL_StatusTypeDef   HAL_Ret;
int shoot_test1 = 0,can_all_test = 0,can2_test = 0;	
HAL_StatusTypeDef   HAL_RetVal;
/*	
函数作用：CAN的接收中断回调函数
参数：hcan：CAN的句柄
返回值：无
函数说明：CAN1数据：pitch轴电机、摩擦轮1、摩擦轮2、拨盘电机
CAN2数据：底盘传上来的YAW轴电机数据和发射相关数据，发射相关数据做热量限制用
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
	can_all_test++;
	if(hcan == &hcan1)
	{ 
		if (HAL_CAN_GetRxMessage(&hcan1,  CAN_FILTER_FIFO0, &rx_header, rx_data)==HAL_OK)
		{
			can1_d++;
			std_d = rx_header.StdId;
			//PITCH轴数据
			if(rx_header.StdId == PITCH_MOTOR_ID)
			{	
				gimbal_motor_time[PITCH] = xTaskGetTickCount();
				
				gimbal_motor_fdb[PITCH].ecd_fdb =  (rx_data[0] << 8) | rx_data[1];
				gimbal_motor_fdb[PITCH].speed_rpm_fdb = (rx_data[2] << 8) | rx_data[3];
				gimbal_motor_fdb[PITCH].current_fdb = (rx_data[4] << 8) | rx_data[5];
				gimbal_motor_fdb[PITCH].temperate_fdb = rx_data[6];
				
				SectionTransform(&motor_relative_angle_fdb[PITCH], gimbal_motor_fdb[PITCH].ecd_fdb, ANGLE_MIN, ANGLE_ZERO, ANGLE_MAX, ECD_MIN, PITCH_ZERO_ECD, ECD_MAX);
			}
			//摩擦轮1的数据
			else if(rx_header.StdId == FR1_MOTOR_ID)
			{
				gimbal_motor_time[0] = xTaskGetTickCount();
				
				shoot_FR_motor_fdb[0].ecd_fdb =  (rx_data[0] << 8) | rx_data[1];
				shoot_FR_motor_fdb[0].speed_rpm_fdb = (rx_data[2] << 8) | rx_data[3];
				shoot_FR_motor_fdb[0].current_fdb = (rx_data[4] << 8) | rx_data[5];
				shoot_FR_motor_fdb[0].temperate_fdb = rx_data[6];
			}
			//摩擦轮2的数据
			else if(rx_header.StdId == FR2_MOTOR_ID) 
			{
				gimbal_motor_time[1] = xTaskGetTickCount();
				
				shoot_FR_motor_fdb[1].ecd_fdb =  (rx_data[0] << 8) | rx_data[1];
				shoot_FR_motor_fdb[1].speed_rpm_fdb = (rx_data[2] << 8) | rx_data[3];
				shoot_FR_motor_fdb[1].current_fdb = (rx_data[4] << 8) | rx_data[5];
				shoot_FR_motor_fdb[1].temperate_fdb = rx_data[6];
			}
			//拨盘的数据
			else if(rx_header.StdId == SHOOT_MOTOR_ID)
			{
			   shoot_ST_motor_fdb.ecd_fdb = (rx_data[0] << 8) | rx_data[1];
				 shoot_ST_motor_fdb.speed_rpm_fdb = (rx_data[2] << 8) | rx_data[3];
				 shoot_ST_motor_fdb.current_fdb = (rx_data[4] << 8) | rx_data[5];
				 shoot_ST_motor_fdb.temperate_fdb = rx_data[6];
			}
			
		} 
	}
	else if (hcan == &hcan2)
	{		
		if(HAL_CAN_GetRxMessage(&hcan2,  CAN_FILTER_FIFO0, &rx_header, rx_data) == HAL_OK)
		{   
			//底盘传上来的YAW轴电机的数据
			std_d = rx_header.StdId;
			if(rx_header.StdId == YAW_MOTOR_ID)
			{
				gimbal_motor_time[YAW] = xTaskGetTickCount();
				gimbal_motor_fdb[YAW].ecd_fdb =  (rx_data[0] << 8) | rx_data[1];
				gimbal_motor_fdb[YAW].speed_rpm_fdb = (rx_data[2] << 8) | rx_data[3];
				gimbal_motor_fdb[YAW].current_fdb = (rx_data[4] << 8) | rx_data[5];
				gimbal_motor_fdb[YAW].temperate_fdb = rx_data[6];
				Chassis_Can2_Referee.robot_ID   =  rx_data[7];//从底盘裁判系统传上来的机器人ID
			//一个yaw轴的编码器值解算成两个角度
			//一个用于云台底盘相对角度变化时旋转矩阵的变化
			//另一个用于云底联动，原因应该是云底联动只有两个方向，底盘只设置了两个正方向，如果用上一个
			//角度值出现了数据越界，即要跟的角度是正负180度，导致没办法正常做pid的运算
			SectionTransform(&motor_relative_angle_fdb[YAW], gimbal_motor_fdb[YAW].ecd_fdb, ANGLE_MIN, ANGLE_ZERO, ANGLE_MAX, ECD_MIN, YAW_ZERO_ECD, ECD_MAX);
			SectionTransform(&Y_yaw_motor_relative_angle_fdb, gimbal_motor_fdb[YAW].ecd_fdb, 0.f, 180.f,360.f, ECD_MIN, Y_YAW_ZERO_ECD, ECD_MAX);
			}
			else if(rx_header.StdId == CHASSIS_TO_GIMBAL)
		  {     can2_test++;
				std_d=rx_header.StdId;
				Chassis_Can2_Referee.shooter_barrel_heat_limit=(rx_data[0] << 8) | rx_data[1];//当前热量上限
				Chassis_Can2_Referee.shooter_id1_17mm_cooling_heat=(rx_data[2] << 8) | rx_data[3];//每秒冷却值
				Chassis_Can2_Referee.robot_level=(rx_data[4]<<8)|rx_data[5];//机器人等级
				Chassis_Can2_Referee.shooter_id1_17mm_speed=(rx_data[6]<<8)|rx_data[7];//17mm弹丸速度

		  }
			
		}
	}
}

//函数作用：发送数据给摩擦轮电机和拨盘电机
//输入参数：fr1  左摩擦轮电流值
//         fr2  右摩擦轮电流值
//         sht  拨盘电流值
void Sent_Shoot(int16_t fr1, int16_t fr2, int16_t sht, int16_t rev)
{
	CAN_TxHeaderTypeDef  tx_message;
	uint8_t              shoot_data[8];
	uint32_t 						 send_mail_box; 
	
	tx_message.StdId = 0x200;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
	
	shoot_data[0] = fr1 >> 8;
	shoot_data[1] = fr1;
	shoot_data[2] = fr2 >> 8;
	shoot_data[3] = fr2;
	shoot_data[4] = sht >> 8;
	shoot_data[5] = sht;
	shoot_data[6] = 0;
	shoot_data[7] = 0;
	
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, shoot_data, &send_mail_box);
}/*		
函数作用：发送PITCH轴控制数据
函数说明：
*/
 void sentGimbal(int16_t pitch)//xData data
{
	CAN_TxHeaderTypeDef  tx_message;
	uint8_t              pitch_data[8];
	uint32_t 						 send_mail_box;
	#if (ROBOT_ID==ROBOT_Infantry) 
	 tx_message.StdId = 0x1FF;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
	
	pitch_data[0] =0;
	pitch_data[1] =0;
	pitch_data[2] =0;
	pitch_data[3] =0;
	pitch_data[4] =0;
	pitch_data[5] =0;
	pitch_data[6] =pitch >> 8;
	pitch_data[7] =pitch;
#elif (ROBOT_ID==ROBOT_Sentry) 
  tx_message.StdId = 0x2FF;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
	
	pitch_data[0] =pitch >> 8;
	pitch_data[1] =pitch;
	pitch_data[2] =0;
	pitch_data[3] =0;
	pitch_data[4] =0;
	pitch_data[5] =0;
	pitch_data[6] =0;
	pitch_data[7] =0;
	#endif
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, pitch_data, &send_mail_box);
}

/*		
函数作用：发送底盘控制数据，三个方向的速度值
函数说明：
*/
 void sentAGVspeed(int16_t speed_x, int16_t speed_y, int16_t speed_w)
{
	CAN_TxHeaderTypeDef  tx_message;
	uint8_t              speed_data[6];
	uint32_t 						 send_mail_box;
	tx_message.StdId = CHASSIS_X_Y_W_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x06;
	
	speed_data[0] = (speed_x)>> 8;
	speed_data[1] = (speed_x);
	speed_data[2] = (speed_y) >> 8;
	speed_data[3] = (speed_y);
	speed_data[4] =  speed_w >> 8;
	speed_data[5] =  speed_w;
	
	HAL_CAN_AddTxMessage(&hcan2, &tx_message, speed_data, &send_mail_box);
}

/*==============================================================================
以下函数通过结构体和指针的方式在文件间传递数据
  ==============================================================================*/
uint32_t GetPitchMotorFdb(Motor_Feedback_t* motor)
{
	*motor = gimbal_motor_fdb[PITCH];
	return gimbal_motor_time[PITCH];
}
uint32_t GetYawMotorFdb(Motor_Feedback_t* motor)
{
	*motor = gimbal_motor_fdb[YAW];
	return gimbal_motor_time[YAW];
}
uint32_t GetMotorPitchRelativeAngle(fp32* angle)
{
	*angle = motor_relative_angle_fdb[PITCH];
	return gimbal_motor_time[PITCH];
}
uint32_t GetMotorYawRelativeAngle(fp32* angle)
{
	*angle = motor_relative_angle_fdb[YAW];
	return  gimbal_motor_time[YAW];
}
uint32_t GetFR1Motor(Motor_Feedback_t* motor)
{
	*motor = 	shoot_FR_motor_fdb[0];
	return gimbal_motor_time[0];
}
uint32_t GetFR2Motor(Motor_Feedback_t* motor)
{
	*motor = 	shoot_FR_motor_fdb[1];
	return gimbal_motor_time[1];
}
uint32_t GetSTMotor(Motor_Feedback_t* motor)
{
	*motor =  shoot_ST_motor_fdb;
	return gimbal_motor_time[2];
}
void Get_one_head_angle(fp32* One_Head_Angle)
{
	*One_Head_Angle=Y_yaw_motor_relative_angle_fdb;
}
void Get_robot_ID(uint8_t* robot_ID)
{
*robot_ID = Chassis_Can2_Referee.robot_ID ;
}
/*==============================================================================

  ==============================================================================*/