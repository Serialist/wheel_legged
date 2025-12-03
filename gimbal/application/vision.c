/*
----------------------------------------------------------
2024赛季视觉采用串口发送和接收
2025赛季视觉改成虚拟串口发送和接收，且把姿态角的获取方式改为WHX开源的扩展卡尔曼滤波
和视觉交互数据的主要任务是发送当前的姿态角和对方装甲板的颜色给工控机，
然后接收目标角度和当前角度的差值，利用差值实现自瞄
----------------------------------------------------------
 */
// 系统层
#include "cmsis_os.h"
// 应用硬件层
#include "bsp_can.h"
#include "usbd_cdc_if.h"
// 数学运算层
#include "average_filter.h"
// 任务层
#include "vision.h"
#include "INS_task.h"
#include "gimbal_task.h"

Vision_t Vision_Data;	   // 自瞄数据大结构体
uint8_t vision_robot_ID;   // 机器人ID，用于判断视觉识别红色还是蓝色
float vision_IMU_angle[3]; // 从INS_Task任务传来的云台当前三轴角度

// 姿态角滤波结构体定义，定义成全局变量以便查看滤波结果
median_Ave_type_def_t vision_Ave[2];
// 姿态角滤波结构体定义，定义成全局变量以便查看滤波结果

// 主任务，仅发送调用发送函数
void vision_task(void const *pvParameters)
{
	while (1)
	{
		Get_IMU_Angle_Data();
		transmit_vision_data();
		osDelay(1);
	}
}

/*
函数作用：将float型数据转换为4字节的字节数组
函数说明：要用到共用体先将float类型数据转换为
长整形再转化为四个8位的数据因为float类型的数据不能直接做位运算。
参数：f：float型数据
byte[]：4字节的字节数组
返回值：无
*/
void Float_to_Byte(float f, unsigned char byte[])
{
	FloatLongType fl;
	fl.fdata = f;
	byte[0] = (unsigned char)fl.ldata;
	byte[1] = (unsigned char)(fl.ldata >> 8);
	byte[2] = (unsigned char)(fl.ldata >> 16);
	byte[3] = (unsigned char)(fl.ldata >> 24);
}
// extern float vision_angle[3];
/*2025版本
函数作用：利用虚拟串口接收工控机传来的姿态角或哨兵接收移动的X,Y值
参数：Buf：接收到的串口数据
User_Buf：解析串口前的Buf数据不需要读取，直接存入User_Buf中
返回值：无
函数说明：此接收函数在usbd_cdc_if.c文件中调用
*/

void user_USB_Receive(uint8_t *Buf)
{
	Float_Transform_u U8_TO_FL[3] = {0}; // 解析串口前的Buf数据不需要读取，直接存入User_Buf中
										 // 自瞄相关
	if (Buf[0] == 0xff && Buf[15] == 0x0d)
	{
		Vision_Data.fire_flag = Buf[1]; /*后续将Buf的值付给四个8位的数组，分别是PITCH、yaw和raw*/

		U8_TO_FL[0].data[0] = Buf[2];
		U8_TO_FL[0].data[1] = Buf[3];
		U8_TO_FL[0].data[2] = Buf[4];
		U8_TO_FL[0].data[3] = Buf[5];
		Vision_Data.angle_set_err[YAW] = M_Ave_filter(&vision_Ave[0], U8_TO_FL[0].data_raw);

		U8_TO_FL[1].data[0] = Buf[6];
		U8_TO_FL[1].data[1] = Buf[7];
		U8_TO_FL[1].data[2] = Buf[8];
		U8_TO_FL[1].data[3] = Buf[9];
		Vision_Data.angle_set_err[PITCH] = M_Ave_filter(&vision_Ave[1], U8_TO_FL[1].data_raw);

		U8_TO_FL[2].data[0] = Buf[10];
		U8_TO_FL[2].data[1] = Buf[11];
		U8_TO_FL[2].data[2] = Buf[12];
		U8_TO_FL[2].data[3] = Buf[13];
		Vision_Data.angle_set_err[2] = U8_TO_FL[2].data_raw; // row轴数据没用
	}
	// 导航相关
	if (Buf[0] == 0xee)
	{
		Int_Transform_u temp;
		temp.data[0] = Buf[1];
		temp.data[1] = Buf[2];
		temp.data[2] = Buf[3];
		temp.data[3] = Buf[4];
		Vision_Data.y_go = temp.data_int;

		temp.data[0] = Buf[5];
		temp.data[1] = Buf[6];
		temp.data[2] = Buf[7];
		temp.data[3] = Buf[8];
		Vision_Data.x_go = temp.data_int;

		temp.data[0] = Buf[9];
		temp.data[1] = Buf[10];
		temp.data[2] = Buf[11];
		temp.data[3] = Buf[12];
		Vision_Data.navi_finish = Buf[13];
	}
}
// GetIMUPitchAngleFdb(&vision_IMU_angle[0]);

/*
函数作用：发送姿态角给工控机
参数：无
返回值：无
函数说明：
*/
void transmit_vision_data(void)
{
	uint8_t robo_color = 0; // 机器人颜色
	//
	/*
	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	这里姿态角的正负和先传PITCH轴还是YAW轴的数据要根据C板的安装方式和视觉那边的需要而定
	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!！！！！
	*/
	uint8_t byte_yaw[4] = {0};
	Float_to_Byte(vision_IMU_angle[YAW], byte_yaw);
	uint8_t byte_pitch[4] = {0};
	Float_to_Byte(-vision_IMU_angle[PITCH], byte_pitch);
	uint8_t byte_row[4] = {0};
	Float_to_Byte(0, byte_row);

	// 通过裁判系统判断机器人颜色
	if (vision_robot_ID >= 10)
	{
		robo_color = vision_red;
	}
	else
	{
		robo_color = vision_blue;
	}

	/**
	 * @date 2024-10-06
	 * @version 1.1.1
	 * @brief 改为导航发送数据协议
	 */
#if ROBOT_MODE == ROBOT_Sentry

	/// @brief 是下面是自瞄

	// 0xff和0x0d为帧头帧尾
	uint8_t total_byte[32];
	total_byte[0] = 0xff;
	total_byte[1] = robo_color; // 自瞄装甲板颜色判断 识别蓝装甲板1  红0
	total_byte[2] = byte_row[0];
	total_byte[3] = byte_row[1];
	total_byte[4] = byte_row[2];
	total_byte[5] = byte_row[3];

	total_byte[6] = byte_pitch[0];
	total_byte[7] = byte_pitch[1];
	total_byte[8] = byte_pitch[2];
	total_byte[9] = byte_pitch[3];

	total_byte[10] = byte_yaw[0];
	total_byte[11] = byte_yaw[1];
	total_byte[12] = byte_yaw[2];
	total_byte[13] = byte_yaw[3];

	total_byte[14] = 0x00;
	total_byte[15] = 0x00;

	/// @brief 下面是导航

	uint8_t game_begin = 0;
	uint8_t quit_navi = 0;
	uint16_t sentry_HP = 400;
	uint8_t outpost_state = 1;
	uint8_t projectile_allowance = 0;
	uint8_t game_remain_90s = 39;

	total_byte[16] = 0xA6;
	total_byte[17] = game_begin;
	total_byte[18] = quit_navi;
	total_byte[19] = sentry_HP;
	total_byte[20] = sentry_HP >> 8;
	total_byte[21] = outpost_state;
	total_byte[22] = projectile_allowance; // 允许发单量标志位
	total_byte[23] = 0;
	total_byte[24] = game_remain_90s;
	total_byte[25] = (uint8_t)(Vision_Data.angle_set_err[2] * 10);
	total_byte[26] = 0;
	total_byte[27] = 0;
	total_byte[28] = 0;
	total_byte[29] = 0;
	total_byte[30] = 0;
	total_byte[31] = 0x0d;
#elif ROBOT_MODE == ROBOT_Infantry
	uint8_t total_byte[16] = {0xff, robo_color, byte_row[0], byte_row[1], byte_row[2], byte_row[3], byte_pitch[0], byte_pitch[1], byte_pitch[2], byte_pitch[3], byte_yaw[0], byte_yaw[1], byte_yaw[2], byte_yaw[3], 0x00, 0x0d};
#endif
	CDC_Transmit_FS(total_byte, sizeof(total_byte));
}
// 函数作用：从INS_task中获取当前云台的三轴角度
void Get_IMU_Angle_Data(void)
{
	GetIMUPitchAngleFdb(&vision_IMU_angle[PITCH]);
	GetIMUYawAngleFdb(&vision_IMU_angle[YAW]);
	Get_robot_ID(&vision_robot_ID);
}
// 获取视觉大结构体
void Get_vision_Data(Vision_t *vision_data)
{
	*vision_data = Vision_Data;
}
