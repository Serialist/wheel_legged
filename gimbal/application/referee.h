#ifndef __REFEREE_H
#define __REFEREE_H
#include "stdint.h"
#include "string.h"
#include "stdbool.h"


#define JUDGE_FRAME_HEADER 0xA5         //֡ͷ


#define TRUE 1
#define FALSE 0

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16


//ͨ��Э���ʽ
typedef enum  
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
}JudgeFrameOffset;

// frame_header ��ʽ
typedef enum
{
	SOF_t          = 0,//��ʼλ
	DATA_LENGTH_t  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ_t          = 3,//�����
	CRC8_t         = 4 //CRC8	
}	FrameHeaderOffset;

typedef enum
{
	ID_game_state       = 0x0001,
	ID_game_result      = 0x0002,
	ID_game_robot_survivors       	= 0x0003,//����������Ѫ��
	
	ID_event_data  					= 0x0101,//�����¼����� 
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_supply_warm 	= 0x0104,//����ϵͳ��������
	ID_missile_shoot_time =0x0105  , //���ڷ����������
	
	ID_game_robot_state    			= 0x0201,//������������ϵ����
	ID_power_heat_data    			= 0x0202,//ʵʱ����&ǹ����������
	ID_game_robot_pos        		= 0x0203,//������λ������
	ID_buff_musk					= 0x0204,//��������������
	ID_aerial_robot_time			= 0x0205,//���л�����ʱ������
	ID_robot_hurt					= 0x0206,//�˺�״̬����
	ID_shoot_data					= 0x0207,//ʵʱ�������
	ID_bullet_remaining          = 0x0208,//����������
	ID_rfid_status									= 0x0209,//������RFID״̬��3Hz
	
	ID_dart_client_directive        = 0x020A,//���ڻ����˿ͻ���ָ������, 3Hz
	ID_robot_location								=	0x020B,//���������λ������,1Hz
	ID_radar_sign_progress					=	0x020C,//�״��ǽ�������
	ID_sentry_autodecision_message_synchronization = 0x020D,//�ڱ�����������Ϣͬ��
	ID_radar_autodecision_message_synchronization = 0x020E,//�״�����������Ϣͬ��

	ID_robot_interactive_header_data			= 0x0301,//�����˽������ݣ��������ͷ������������� 10Hz
	ID_controller_interactive_header_data = 0x0302,//�Զ���������������ݽӿڣ�ͨ�������ͻ��˴����������� 30Hz
	ID_map_interactive_header_data        = 0x0303,//�ͻ���С��ͼ�������ݣ������������͡���
	ID_keyboard_information               = 0x0304,//���̡������Ϣ��ͨ������ͼ�����ڡ�������
	ID_map_receive_radar									=	0x0305,//ѡ�ֶ�С��ͼ�����״�����
	ID_user_defined_remot_data            = 0x0306,//�Զ����������������
	ID_map_receive_sentry									=	0x0307,//ѡ�ֶ�С��ͼ�����ڱ�����
	ID_map_receive_robot									=	0x0308//ѡ�ֶ�С��ͼ���ջ���������

	
}CmdID;

typedef enum
{
	/* Std */
	LEN_FRAME_HEAD 	                 = 5,	// ֡ͷ����
	LEN_CMD_ID 		                   = 2,	// �����볤��
	LEN_FRAME_TAIL 	                 = 2,	// ֡βCRC16
	/* Ext */  

	LEN_game_state       				=  11,	//0x0001
	LEN_game_result       			=  1,	//0x0002
	LEN_game_robot_survivors    =  32,	//0x0003  ����������Ѫ������
	LED_game_buff               =11 , //0X0005
	
	LEN_event_data  						=  4,	//0x0101  �����¼����� 
	LEN_supply_projectile_action   =  4,	//0x0102���ز���վ������ʶ����
	LEN_supply_warm        			=3, //����ϵͳ���� 0x0104
	LEN_missile_shoot_data 			=3, //���ڷ������� 0x0105
	
	LEN_game_robot_state    		= 13,	//0x0201������״̬����
	LEN_power_heat_data   			= 16,	//0x0202ʵʱ������������
	LEN_game_robot_pos        	= 16,	//0x0203������λ������
	LEN_buff_musk        				=  6,	//0x0204��������������
	LEN_aerial_robot_energy     =  2,	//0x0205���л�����֧Ԯʱ��
	LEN_robot_hurt        			=  1,	//0x0206�˺�״̬����
	LEN_shoot_data       				=  7,	//0x0207	ʵʱ�������
	LEN_bullet_remaining        = 6,//ʣ�෢����
  
	LEN_rfid_status					    = 4,//RFID״̬
	LEN_dart_client_directive   = 6,//0x020A���ڻ����˿ͻ���ָ������
	LEN_robot_location					=40,//0x020B ���������λ������
	LEN_radar_sign_progress			= 6,//0x020C �״��ǽ�������
	LEN_sentry_autodecision_message_synchronization 	=4,//0x020D �ڱ�����������Ϣͬ��
	LEN_radar_autodecision_message_synchronization 		=1,// 0x020E �״�����������Ϣͬ��
	
	LEN_robot_interactive_header_data				=128,// 0x0301 �����˽������ݣ��������ͷ������������� 10Hz
	LEN_controller_interactive_header_data 	=30,// 0x0302 �Զ���������������ݽӿڣ�ͨ�������ͻ��˴����������� 30Hz
	LEN_map_interactive_header_data        	=15,// 0x0303 �ͻ���С��ͼ�������ݣ������������͡���
	LEN_keyboard_information               	=12,// 0x0304 ���̡������Ϣ��ͨ������ͼ�����ڡ�������
	LEN_map_receive_radar										=10,//	0x0305 ѡ�ֶ�С��ͼ�����״�����
	LEN_user_defined_remot_data            	=8,// 0x0306 �Զ����������������
	LEN_map_receive_sentry									=103,//	0x0307 ѡ�ֶ�С��ͼ�����ڱ�����
	LEN_map_receive_robot										=34//	0x0308 ѡ�ֶ�С��ͼ���ջ���������

}JudgeDataLength;

/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;	
} xFrameHeader;

/* ID: 0x0001  Byte:  11    ����״̬���� */
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
}game_status_t;


/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct
{
 uint8_t winner;
}game_result_t;


/* ID: 0x0003  Byte:  32    ����������Ѫ������ */
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP;
 uint16_t red_3_robot_HP;
 uint16_t red_4_robot_HP;
 uint16_t red_5_robot_HP;
 uint16_t red_7_robot_HP;
 uint16_t red_outpost_HP;
 uint16_t red_base_HP;
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP;
 uint16_t blue_3_robot_HP;
 uint16_t blue_4_robot_HP;
 uint16_t blue_5_robot_HP;
 uint16_t blue_7_robot_HP;
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
}game_robot_HP_t;
 

/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	 uint32_t event_type;
}event_data_t; 


/* ID: 0x0102  Byte:  4    ���ز���վ������ʶ���� */
typedef __packed struct
{
	 uint8_t reserved;
	 uint8_t supply_robot_id;
	 uint8_t supply_projectile_step;
	 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* ID: 0x0104  Byte: 3   ����ϵͳ������Ϣ */
typedef __packed struct
{
	 uint8_t level;
	 uint8_t offending_robot_id;
	 uint8_t count;
}referee_warning_t;

/* ID: 0x0105  Byte:3  ���ڷ���ڵ���ʱ */
typedef __packed struct
{
	 uint8_t dart_remaining_time;
	 uint16_t dart_info;
}dart_info_t;

/* ID: 0X0201  Byte: 13    ������״̬���� */
typedef __packed struct 
{ 
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t current_HP;
		uint16_t maximum_HP;
		uint16_t shooter_barrel_cooling_value;
		uint16_t shooter_barrel_heat_limit;
		uint16_t chassis_power_limit;
		uint8_t power_management_gimbal_output : 1;
		uint8_t power_management_chassis_output : 1;
		uint8_t power_management_shooter_output : 1;
}robot_status_t;

/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct 
{ 
	 uint16_t chassis_voltage;
	 uint16_t chassis_current;
	 float chassis_power;
	 uint16_t buffer_energy;
	 uint16_t shooter_17mm_1_barrel_heat;
	 uint16_t shooter_17mm_2_barrel_heat;
	 uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

/* ID: 0x0203  Byte: 12    ������λ������ */
typedef __packed struct 
{   
	 float x;
	 float y;
	 float angle;
}robot_pos_t;

/* ID: 0x0204  Byte:  6    �������������� */
typedef __packed struct 
{
	 uint8_t recovery_buff;
	 uint8_t cooling_buff;
	 uint8_t defence_buff;
	 uint8_t vulnerability_buff;
	 uint16_t attack_buff;
}buff_t;

/* ID: 0x0205  Byte:  2    ���л���������״̬���� */
typedef __packed struct
{
	 uint8_t airforce_status;
	 uint8_t time_remain;
}air_support_data_t;

/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	 uint8_t armor_id : 4;
	 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef __packed struct 
{
	 uint8_t bullet_type;
	 uint8_t shooter_number;
	 uint8_t launching_frequency;
	 float initial_speed;
}shoot_data_t;


/* ID: 0x0208  Byte:  6    �ӵ�ʣ������ */
typedef __packed struct 
{
	 uint16_t projectile_allowance_17mm;
	 uint16_t projectile_allowance_42mm;
	 uint16_t remaining_gold_coin;
}projectile_allowance_t;


/* ID: 0x0209  Byte:  4 	������RFID״̬ */
typedef __packed struct
{
	uint32_t rfid_status;
} rfid_status_t;


/* ID: 0x020A  Byte:  16 	���ڻ����˿ͻ���ָ������ */
typedef __packed struct
{
	 uint8_t dart_launch_opening_status;
	 uint8_t reserved;
	 uint16_t target_change_time;
	 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;


/* ID: 0x020B  Byte:  40 	���������λ������ */
typedef __packed struct
{
	 float hero_x;
	 float hero_y;
	 float engineer_x;
	 float engineer_y;
	 float standard_3_x;
	 float standard_3_y;
	 float standard_4_x;
	 float standard_4_y;
	 float standard_5_x;
	 float standard_5_y;
}ground_robot_position_t;


/* ID: 0x020C  Byte:  6 	�״��ǽ������� */
typedef __packed struct
{
	 uint8_t mark_hero_progress;
	 uint8_t mark_engineer_progress;
	 uint8_t mark_standard_3_progress;
	 uint8_t mark_standard_4_progress;
	 uint8_t mark_standard_5_progress;
	 uint8_t mark_sentry_progress;
}radar_mark_data_t;


/* ID: 0x020D  Byte:  4 	�ڱ�����������Ϣͬ�� */
typedef __packed struct
{
	uint32_t sentry_info;
} sentry_info_t;


/* ID: 0x020E  Byte:  1 	�״�����������Ϣͬ�� */
typedef __packed struct
{
 uint8_t radar_info;
} radar_info_t;


/* 
	
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)�� 
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)�� 

	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/

/* �������ݽ�����Ϣ��0x0301  */

/****************************����ID����********************/
#define UI_Data_ID_Del 0x0100 
#define UI_Data_ID_Draw1 0x0101
#define UI_Data_ID_Draw2 0x0102
#define UI_Data_ID_Draw5 0x0103
#define UI_Data_ID_Draw7 0x0104
#define UI_Data_ID_DrawChar 0x0110
#define	Sentry_AutoDecision 0x0120
#define	Radar_AutoDecision 0x0121
/****************************�췽������ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************����������ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************�췽������ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************����������ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************ɾ������***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************ͼ�����ò���__ͼ�β���********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************ͼ�����ò���__ͼ������********************/
#define UI_Graph_Line 0         //ֱ��
#define UI_Graph_Rectangle 1    //����
#define UI_Graph_Circle 2       //��Բ
#define UI_Graph_Ellipse 3      //��Բ
#define UI_Graph_Arc 4          //Բ��
#define UI_Graph_Float 5        //������
#define UI_Graph_Int 6          //����
#define UI_Graph_Char 7         //�ַ���
/***************************ͼ�����ò���__ͼ����ɫ********************/
#define UI_Color_Main 0         //������ɫ
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //�Ϻ�ɫ
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //��ɫ
#define UI_Color_Black 7
#define UI_Color_White 8

/* ������ID: 0x0100  Byte:  2 	ѡ�ֶ�ɾ��ͼ�� */
typedef __packed struct
{
	uint8_t delete_type;
	uint8_t layer;
}interaction_layer_delete_t;


/* ������ID: 0x0101  Byte:  15 	ѡ�ֶ˻���һ��ͼ�� */
typedef __packed struct
{
	uint8_t figure_name[3];
	uint32_t operate_tpye:3;
	uint32_t figure_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t details_a:9;
	uint32_t details_b:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t details_c:10;
	uint32_t details_d:11;
	uint32_t details_e:11;
}interaction_figure_t;


/* ������ID: 0x0102  Byte:  30  ѡ�ֶ˻�������ͼ�� */
typedef __packed struct
{
	interaction_figure_t interaction_figure[2];
}interaction_figure_2_t;


/* ������ID: 0x0103  Byte:  75  ѡ�ֶ˻������ͼ�� */
typedef __packed struct
{
	interaction_figure_t interaction_figure[5];
}interaction_figure_3_t;


/* ������ID: 0x0104  Byte:  105 ѡ�ֶ˻����߸�ͼ�� */
typedef __packed struct
{
	interaction_figure_t interaction_figure[7];
}interaction_figure_4_t;

/* ������ID: 0x0110  Byte:  45  ѡ�ֶ˻����ַ�ͼ�� */
typedef __packed struct
{
	interaction_figure_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;


/* ������ID: 0x0120  Byte:  4   �ڱ���������ָ�� */
typedef __packed struct
{
	uint32_t sentry_cmd;
} sentry_cmd_t;


/* ������ID: 0x0121  Byte:  1   �״���������ָ�� */
typedef __packed struct
{
	uint8_t radar_cmd;
} radar_cmd_t;


/* ��������:�������ݷ�װΪһ���ṹ�� */
typedef  struct
{
	interaction_layer_delete_t          InteractionLayerDelete;//0x0100  	ѡ�ֶ�ɾ��ͼ��
	interaction_figure_t								InteractionFigure;//0x0101  	ѡ�ֶ˻���һ��ͼ��
	interaction_figure_2_t							InteractionFigure2;//0x0102   ѡ�ֶ˻�������ͼ�� 
	interaction_figure_3_t							InteractionFigure3;//0x0103   ѡ�ֶ˻������ͼ�� 
	interaction_figure_4_t							InteractionFigure4;//0x0104   ѡ�ֶ˻����߸�ͼ�� 
	ext_client_custom_character_t				ExtClientCustomCharacter;//0x0110 ѡ�ֶ˻����ַ�ͼ��
	sentry_cmd_t												SentryCmd;//0x0120  �ڱ���������ָ��
	radar_cmd_t													RadarCmd;//0x0121   �״���������ָ��
} sum_subcontent_t;


typedef __packed struct 
{ 
	 uint16_t data_cmd_id;
	 uint16_t sender_id;
	 uint16_t receiver_id;
	 uint8_t user_data[113];
}robot_interaction_data_t;


/*�Զ��������*/

/* ID: 0x0302  Byte:  30   �Զ���������������ݽӿ� */
typedef __packed struct
{
	uint8_t data[30];
}custom_robot_data_t;



/* 
��̨�ֿ�ͨ��ѡ�ֶ˴��ͼ������˷��͹̶����ݡ�

1.������Ϊ 0x0303������ʱ���ͣ����η��ͼ�����õ��� 0.5 �롣
	���ͷ�ʽһ��
	�� �������������ͷ��
	�ڣ���ѡ������һ�����̰��������Է�������ͷ��
	�۵��С��ͼ����λ�á��÷�ʽ�򼺷�ѡ���Ļ����˷��͵�ͼ�������ݣ�������Է�������ͷ������Ŀ
	������� ID �����������ݡ�
	���ͷ�ʽ����
	�٣���ѡ������һ�����̰��������Է�������ͷ��
	�ڵ��С��ͼ����λ�á��÷�ʽ�򼺷����л����˷��͵�ͼ�������ݣ�������Է�������ͷ������Ŀ��
	������ ID �����������ݡ�
	ѡ����Զ����Ʒ�ʽ�Ļ����˶�Ӧ�Ĳ����ֿ�ͨ��ѡ�ֶ˴��ͼ������˷��͹̶����ݡ�
2.������Ϊ 0x0303������ʱ���ͣ����η��ͼ�����õ��� 3 �롣
	���ͷ�ʽ��
	�٣���ѡ������һ�����̰��������Է�������ͷ��
	�ڵ��С��ͼ����λ�á��÷�ʽ������ֶ�Ӧ�Ļ����˷��͵�ͼ�������ݣ�������Է�������ͷ������
	Ŀ������� ID �����������ݡ�
	һ̨ѡ����Զ����Ʒ�ʽ�Ļ����˼ȿ��Խ�����̨�ַ��͵���Ϣ��Ҳ���Խ��ն�Ӧ�����ֵ���Ϣ��������
	Ϣ����Դ�����±��С���Ϣ��Դ���н�������
*/

/* ID: 0x0303  Byte:  12   �ͻ���С��ͼ�������� */
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
}map_command_t;



/*����&ң��������*/
/* ID: 0x0304  Byte:  12   ���̡������Ϣ */
typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}remote_control_t;



/* ID: 0x0305  Byte:  10   ѡ�ֶ�С��ͼ�����״����� */
typedef __packed struct
{
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
}map_robot_data_t;



/*����·����*/

/* ID: 0x0306  Byte:  8   �Զ���������������� */
typedef __packed struct
{
	uint16_t key_value;
	uint16_t x_position:12;
	uint16_t mouse_left:4;
	uint16_t y_position:12;
	uint16_t mouse_right:4;
	uint16_t reserved;
}custom_client_data_t;



/* ID: 0x0307  Byte:  105   ѡ�ֶ�С��ͼ�����ڱ����� */
typedef __packed struct
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
}map_data_t;


/* ID: 0x0308  Byte:  34   ѡ�ֶ�С��ͼ���ջ��������� */
typedef __packed struct
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

/*
----------------------------------------------------------
���������ֲ���ϵͳ�Ľṹ��
Referee_info_t ��ͨ��ͼ����·�����ģ���ң�ص���Ϣ
Referee        ��ͨ�����̴����ģ����̽��ղ���ϵͳ�����ݣ�������̨��Ҫ�Ĳ���ϵͳ���ݴ�����̨
��ô����ԭ����ͼ����·�͵�Դ����ģ�鴫�����ϵͳ�������ǲ�һ���ģ��������ϵͳ�ֲ�
-----------------------------------------------------------
*/


typedef struct{
	xFrameHeader										FrameHeader;				// ֡ͷ��Ϣ
	
//	game_status_t 									GameState;				// 0x0001           ����״̬����
//	game_result_t 									GameResult;				// 0x0002         �����������
//	game_robot_HP_t 								GameRobotHP;			// 0x0003         ������Ѫ������
//	
//	event_data_t										EventData;					// 0x0101         �����¼�����
//	ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102 ����վ������ʶ
//	referee_warning_t								RefereeWarning;		// 0x0104         ���о�����Ϣ
//	dart_info_t											DartInfo;// 0x0105         ����״̬
//	
//	robot_status_t									GameRobotStat;	// 0x0201         ����������״̬
//	power_heat_data_t								PowerHeatData;		// 0x0202         ʵʱ������������
//	robot_pos_t											GameRobotPos;			// 0x0203         ������λ��
//	buff_t													Buff;								// 0x0204     ����������
//	air_support_data_t							AerialRobotEnergy;// 0x0205             ���л���������״̬
//	hurt_data_t											RobotHurt;					// 0x0206         �˺�״̬
//	shoot_data_t										ShootData;					// 0x0207         ʵʱ�����Ϣ(��Ƶ  ����  �ӵ���Ϣ)
//	projectile_allowance_t					ProjectileAllowance;		// 0x0208	        �ӵ�ʣ�෢����
//	rfid_status_t										RfidStatus;				// 0x0209	        RFID��Ϣ
//	dart_client_cmd_t           		DartClient;        // 0x020A         ���ڿͻ���
//	ground_robot_position_t					GroundRobotPosition;	//0x020B 			���������λ������
//	radar_mark_data_t								RadarMarkData;//0x020C   �״��ǽ������� 
//	sentry_info_t                   SentryInfo;//0x020D      �ڱ�����������Ϣͬ��
	radar_info_t										RadarInfo;//0x020E	�״�����������Ϣͬ��
	
	robot_interaction_data_t				RobotInteractionData;// 0x0301 �����˽�������
	sum_subcontent_t 								SumSubcontent;			//0x0301 �����˽������������ݻ���

  remote_control_t	              RemoteControl;
	uint16_t                        self_client;        //�����ͻ���
	
	bool flag;
	
} Referee_info_t;

typedef struct
{
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed;
	uint16_t shooter_barrel_heat_limit;
	uint8_t robot_level;
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 uint8_t bullet_freq_max;
 uint8_t robot_ID;
 float bullet_speed;
}Referee;

extern Referee_info_t REF;



#endif


