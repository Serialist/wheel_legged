//系统层
#include "main.h"
#include "cmsis_os.h"
//外设层
#include "usbd_cdc_if.h"
#include "usart.h"
#include "can.h"
#include "tim.h"
//数学运算层
#include "data_transform.h"
#include "constrain_calc.h"
#include "pid.h"
#include "average_filter.h"
//应用硬件层
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_laser.h"
#include "referee.h"
#include "remote_control.h"
#include "bsp_referee.h"
#include "remote_control.h"
//任务层
#include "shoot_task.h"
#include "INS_task.h"
#include "shoot_task.h"
#include "vision.h"
#include "gimbal_task.h"


//结构体定义
Shoot_t shoot;
remote_data_t shoot_new_rc_data;
Vision_t sh_vision_data;
Referee_info_t sh_referee;
//结构体定义

shoot_heat_limit SHOOT_LIMIT;//热量限制结构体

pid_type_def   FR_motor_pid[2];//速度环pid
pid_type_def   ST_motor_pid;
extern uint8_t User_RE_Buf[14];



//2025赛季用来获取两边摩擦轮转速的方差和均值等数据
Var_Mean  fr_Var_Mean[2];

int16_t L_M_speed;
int16_t R_M_speed;

uint8_t shoot_state;

static void shoot_init(void);
void shootModeSet(void);
static void ShootFRModeSet(Shoot_t* FR_mode);
static void Shoot_Speed_Set(Shoot_t *speed);
static void Shoot_Pid_Calc(Shoot_t *ST_Pid);
static void GetFRMotorData(Shoot_t *data_fdb);
void Shoot_Heat_limit(shoot_heat_limit* Shoot_limit,Shoot_t* SHOOT);
void data__var_cal(Var_Mean* Var_Mean,int16_t data);


void shoot_task(void const *pvParameters)
{
	shoot_init();
	while(1)
	{
		//数据获取
		  GetFRMotorData(&shoot); 
		
		Shoot_Heat_limit(&SHOOT_LIMIT,&shoot);
		
		shootModeSet();
		//拨盘摩擦轮设置
		Shoot_Speed_Set(&shoot);
		//拨盘摩擦轮pid计算
		Shoot_Pid_Calc(&shoot);
		//发送电流
		Sent_Shoot(shoot.FR_motor_set[APP_FR1].current_set, shoot.FR_motor_set[APP_FR2].current_set,shoot.ST_motor_set.current_set, 0);
		//Sent_Shoot(0,0,0,0);
		osDelay(1);
	}
}

 static void shoot_init(void)
{
	shoot.shoot_mode = SHOOT_STOP;
	//摩擦轮转速
	shoot.FR_speed   =  6050;
	//拨盘转速
	shoot.ST_speed   =  4000;

	fp32 PID[3];
//摩擦轮单环速度环pid
	PID[0] = FRICTION_MOTOR_PID_KP;
	PID[1] = FRICTION_MOTOR_PID_KI;
	PID[2] = FRICTION_MOTOR_PID_KD;
	PID_init(&FR_motor_pid[APP_FR1], PID_POSITION, PID, FRICTION_MOTOR_PID_MAX_OUT, FRICTION_MOTOR_PID_MAX_IOUT);
	PID_init(&FR_motor_pid[APP_FR2], PID_POSITION, PID, FRICTION_MOTOR_PID_MAX_OUT, FRICTION_MOTOR_PID_MAX_IOUT);
//摩擦轮单环速度环pid
	
	PID[0] = SHOOT_MOTOR_PID_KP;
	PID[1] = SHOOT_MOTOR_PID_KI;
	PID[2] = SHOOT_MOTOR_PID_KD;
	PID_init(&ST_motor_pid, PID_POSITION, PID, SHOOT_MOTOR_PID_MAX_OUT, SHOOT_MOTOR_PID_MAX_IOUT);
	
//拨盘pid

}

static void GetFRMotorData(Shoot_t *data_fdb)
{
	GetRCData(&(data_fdb->rc_data));
	Get_New_Rc_data(&shoot_new_rc_data);
	GetFR1Motor(&(data_fdb->FR_motor_fdb[APP_FR1]));
	GetFR2Motor(&(data_fdb->FR_motor_fdb[APP_FR2]));
	GetSTMotor(&(data_fdb->ST_motor_fdb));
	Get_FR_State(&(data_fdb->FR_state));
	Get_New_rc_FR_State(&(data_fdb->new_rc_FR_state));
	Get_vision_Data(&sh_vision_data);
	Get_Referee_Data(&sh_referee);
}

void shootModeSet(void)
{
	ShootFRModeSet(&shoot);
}

int shoot_motor_speed_set=0;
int reversal_time=0;
static void Shoot_Speed_Set(Shoot_t *speed)
{    
    //左右摩擦轮转速检测
	L_M_speed=-speed->FR_motor_fdb[0].speed_rpm_fdb;
	R_M_speed=speed->FR_motor_fdb[1].speed_rpm_fdb;
	//左右摩擦轮转速检测
	
	  //导航模式下发射先停止
//		if((speed->rc_data.rc.s[1] == 1 && speed->rc_data.rc.s[0] != 1) || speed->shoot_mode == SHOOT_NORMAL||shoot.new_rc_FR_state)//摩擦轮开关标志位
	if(( 0 && speed->rc_data.rc.s[0] != 1) || speed->shoot_mode == SHOOT_NORMAL||shoot.new_rc_FR_state)
		{
				shoot_state=1;
				speed->FR_motor_set[APP_FR1].speed_rpm_set =-speed->FR_speed; 
				speed->FR_motor_set[APP_FR2].speed_rpm_set = speed->FR_speed;
			if(shoot.rc_data.rc.ch[4] >= 600 || ((speed->rc_data.mouse.press_l > 0||sh_referee.RemoteControl.left_button_down>0) ))
				{ 	//shoot_new_rc_data.trigger==1||				
					if(gimbal.robot_mode_set == VISION_FOLLOW)
					{
						if(sh_vision_data.fire_flag)//自瞄开火控制
						{
						shoot_motor_speed_set = speed->ST_speed;
						}
						else
						{ 
						shoot_motor_speed_set =0;
						}	
					}
					else 
					{
					shoot_motor_speed_set=speed->ST_speed;
					}					
				}

      			//***********退弹模式************//			
				else if((shoot_new_rc_data.fn_2==1)||(shoot.rc_data.rc.ch[4] <= -600)|| (sh_referee.RemoteControl.keyboard_value & KEY_PRESSED_OFFSET_R) > 0)
				{
					reversal_time++;
					//shoot_motor_speed_set =-2000;
					shoot_motor_speed_set =-2000;
					if(reversal_time > 500)
					{
						shoot_motor_speed_set = 0;
						reversal_time = 0;
					}
				}
				//***********退弹模式************//	
				else 
				{
				shoot_motor_speed_set = 0;
				}
			speed->ST_motor_set.speed_rpm_set = -shoot_motor_speed_set;//锟斤拷锟斤拷转锟斤拷锟劫革拷值
		}
		//******无力模式*****//
	  else if(speed->rc_data.rc.s[1] != 1)  
	  {
			speed->FR_motor_set[APP_FR1].speed_rpm_set = 0;
			speed->FR_motor_set[APP_FR2].speed_rpm_set = 0;
			speed->ST_motor_set.speed_rpm_set = 0;
	  }
		//******锟斤拷锟斤拷模式*****//
}
//函数作用：设置摩擦轮pid
//输入参数：FR_motor_pid[2]  速度环pid
//输出参数：无
static void Shoot_Pid_Calc(Shoot_t *ST_Pid)
{
	   
  ST_Pid->ST_motor_set.current_set      = PID_calc(&ST_motor_pid, ST_Pid->ST_motor_set.speed_rpm_set,ST_Pid->ST_motor_fdb.speed_rpm_fdb);
  ST_Pid->FR_motor_set[APP_FR1].current_set = PID_calc(&FR_motor_pid[APP_FR1], Int16ToFloat(ST_Pid->FR_motor_set[APP_FR1].speed_rpm_set), Int16ToFloat(ST_Pid->FR_motor_fdb[APP_FR1].speed_rpm_fdb));
  ST_Pid->FR_motor_set[APP_FR2].current_set = PID_calc(&FR_motor_pid[APP_FR2], Int16ToFloat(ST_Pid->FR_motor_set[APP_FR2].speed_rpm_set), Int16ToFloat(ST_Pid->FR_motor_fdb[APP_FR2].speed_rpm_fdb));
}



//函数作用：同济大学开源的枪口热量限制，通过计算拨盘转速并获取任务运行时间得到当前剩余枪口热量
//根据剩余枪口热量控制拨盘转速
//
float test_limit_power;
int allowable_number = 0, shoot_time = 0, current_shoot_heat, detect_heat, final_bullet;
int16_t test_detla=10;
float s_limit_k=0.62;
float s_l_k[2]={0.08,0.60};

void Shoot_Heat_limit(shoot_heat_limit* Shoot_limit,Shoot_t* SHOOT)
{ 
	if(gimbal.key_state[key_Q])
		{
Shoot_limit->a = (float)(shoot.referee.shooter_id1_17mm_cooling_heat);//每锟斤拷锟斤拷却
Shoot_limit->m = (float)(shoot.referee.shooter_barrel_heat_limit);//锟斤拷锟斤拷锟斤拷锟斤拷
Shoot_limit->d = 10.0f; //每锟斤拷锟斤拷锟侥碉拷锟斤拷锟斤拷
	s_limit_k  = (float)((shoot.referee.robot_level)*s_l_k[0]+s_l_k[1]);
	if(Shoot_limit-> shoot_time!=0)
 Shoot_limit->shoot_fre  = 270.f * ((float)((10*Shoot_limit->m - Shoot_limit->a )/ (10*((float)(Shoot_limit-> shoot_time)/90)) + Shoot_limit->a/Shoot_limit->d));
  SHOOT->ST_speed =  (float)(Shoot_limit->shoot_fre * s_limit_k);
 
 if(SHOOT->ST_speed>=4000)
 {
 SHOOT->ST_speed = 4000;
 }	
if(sh_referee.RemoteControl.left_button_down==0)
{
Shoot_limit-> shoot_time = 0;
}
else
{
 Shoot_limit-> shoot_time++;
}
           }
}
static void ShootFRModeSet(Shoot_t* FR_mode)
{
	if(FR_mode->FR_state==1)
	   {
       FR_mode->shoot_mode = SHOOT_NORMAL;
       }
		else
		{
	  FR_mode->shoot_mode = SHOOT_STOP;
		}	
}
