#ifndef __JUDGE_H
#define __JUDGE_H
#include "config.h"
#include "stm32f4xx_hal.h"



/****************************Judge***********************************/

#define myCRC8_INIT											0xff
#define ADRESS                          6
#define JudgeBufferLength               150
#define JudgeFrameLength_1              17  //����������״̬���ݳ���
#define JudgeFrameLength_2              15  //ʵʱ�����Ϣ
#define JudgeFrameLength_3              29  //ʵʱ����������Ϣ
#define JudgeFrameHeader                0xA5        //֡ͷ 
#define UP_REG_ID                       0xA0  //up layer regional id
#define DN_REG_ID                       0xA5  //down layer regional id
#define HEADER_LEN                      sizeof(frame_header_t)
#define CMD_LEN                         2    //cmdid bytes
#define CRC_LEN                         2    //crc16 bytes
#define ChassisLimitCurrent             2750            //���̵������Ƽ���
#define CHASSISMAXPOWER                 80.0F       //���������
#define CHASSISMAXPOWERRATE             0.82F       //�������Ƽ��޹��ʣ�80W������(�����ֵΪ0.9����ʵ�����ƹ���Ϊ0.9*80=72W��


 /** 
  * @brief  judgement data command id
  */
typedef enum
{
	GAME_STATE_ID                      =0x0001,// ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz 
	GAME_RESULT_ID                     =0x0002,//����������ݣ�0x0002������Ƶ�ʣ������������� 
	GAME_ROBOT_SURVIVORS_ID            =0x0003,//������Ѫ�����ݣ�0x0003������Ƶ�ʣ�3Hz 
	EVENT_DADA_ID                      =0x0101,//�����¼����ݣ�0x0101������Ƶ�ʣ�1Hz
//	SUPPLY_PROJECTILE_ACTION_ID        =0x0102,//����վ������ʶ��0x0102������Ƶ�ʣ������ı���� 
	REFEREE_WARNING_ID                 =0x0104,//���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ�1Hz 
	DART_FIRING_STATE                  =0x0105,//���ڷ���������ݣ�0x0105������Ƶ�ʣ�1Hz      
	GAME_ROBOT_STATE_ID                =0x0201,// ����������������ϵ���ݣ�0x0201������Ƶ�ʣ�10Hz 
	POWER_HEAT_DATA_ID                 =0x0202,//ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�10Hz 
	GAME_ROBOT_POS_ID                  =0x0203,//������λ�ã�0x0203������Ƶ�ʣ�1Hz 
	BUFF_MUSK_ID                       =0x0204,// ����������͵����������ݣ�0x0204������Ƶ�ʣ�3Hz 
//	AERIAL_ROBOT_ENERGY_ID             =0x0205,// ���л���������״̬��0x0205������Ƶ�ʣ�10Hz 
	ROBOT_HURT_ID                      =0x0206,//�˺�״̬��0x0206������Ƶ�ʣ��˺��������� 
	SHOOT_DATA_ID                      =0x0207,//ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������� 
	BULLET_REMAINING_ID                =0x0208,//�ӵ�������������0x0208������Ƶ�ʣ�10Hz ���ڷ��ͣ����л������Լ��ڱ����������ط��� 
	
	STUDENT_INTERACTIVE_HEADER_DATA_ID =0x0301,//�������ݽ�����Ϣ��0x0301������Ƶ�ʣ����� 30Hz 
	CLIENT_CUSTOM_DATA_ID              =0x0301,//�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180������Ƶ�ʣ����� 10Hz 
	ROBOT_INTERACTIVE_DATA_ID          =0x0301,//�������� �����˼�ͨ�ţ�0x0301������Ƶ�ʣ�����10Hz 
	CLIENT_GRAPHIC_DRAW_ID             =0x0301,//�ͻ����Զ���ͼ�� �����˼�ͨ�ţ�0x0301������Ƶ�ʣ����� 10Hz 
	ROBOT_COMMAND_ID                   =0x0303,
} judge_data_id_e;
/** 
  * @brief  2021���ڽӿ�Э��˵�� 
  */
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}game_status_t;
	
	typedef __packed struct //����������ݣ�0x0002������Ƶ�ʣ������������� 
	{ 
		uint8_t winner;
	} game_result_t; 
	
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t reserved;	
// uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
// uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} game_robot_HP_t;

	
	
	typedef __packed struct //�����¼����ݣ�0x0101������Ƶ�ʣ��¼��ı���� 
	{  
	  //   uint32_t event_type; 
		uint32_t event_data;
	} event_data_t; 
	
	typedef __packed struct //����վ������ʶ��0x0102������Ƶ�ʣ������ı���� 
	{  
		uint8_t reserved;
		uint8_t supply_robot_id;
		uint8_t supply_projectile_step;
		uint8_t supply_projectile_num;
	} ext_supply_projectile_action_t;
	

	typedef __packed struct //���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢������ 
{
		uint8_t level;
		uint8_t offending_robot_id;
		uint8_t count;
}referee_warning_t;

typedef __packed struct //���ڷ���������ݣ�0x0105 .����Ƶ�ʣ�1Hz
{
   uint8_t dart_remaining_time;
   uint16_t dart_info;
}dart_info_t;

	typedef __packed struct
{
	uint8_t robot_id; //��������ID
	uint8_t robot_level;//�����˵ȼ�
	uint16_t current_HP;//�����˵�ǰѪ��
	uint16_t maximum_HP;//������Ѫ������
	uint16_t shooter_barrel_cooling_value;//�������������ÿ����ȴֵ
	uint16_t shooter_barrel_heat_limit;//�����������������
	uint16_t chassis_power_limit;//�����˵��̹�������
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
}robot_status_t;

	typedef __packed struct//ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz 
	{   
	 uint16_t chassis_volt; 
	 uint16_t chassis_current; 
     float    chassis_power; 
     uint16_t chassis_power_buffer; 
     uint16_t shooter_id1_17mm_cooling_heat;
     uint16_t shooter_id2_17mm_cooling_heat;
     uint16_t shooter_id1_42mm_cooling_heat;
	} power_heat_data_t; 
 
	typedef __packed struct //������λ�ã�0x0203������Ƶ�ʣ�10Hz 
	{ 
		float x;  
		float y; 
		float z;  
		float angle; 
	} robot_pos_t; 
	
	typedef __packed struct
{
		uint8_t recovery_buff;
		uint8_t cooling_buff;
		uint8_t defence_buff;
		uint8_t vulnerability_buff;
		uint16_t attack_buff;
	    uint8_t remaining_energy;
}buff_t;


	typedef __packed struct    //���л���������״̬��0x0205������Ƶ�ʣ�10Hz 
{
		uint8_t airforce_status;
		uint8_t time_remain;
}air_support_data_t; 

	typedef __packed struct //�˺�״̬��0x0206������Ƶ�ʣ��˺��������� 
	{
		uint8_t armor_id : 4;
		uint8_t HP_deduction_reason : 4;
	}hurt_data_t;
 
	typedef __packed struct// ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������� 
	{   
		 uint8_t bullet_type;
     uint8_t shooter_id;
		 uint8_t bullet_freq;
     float bullet_speed;
	} ext_shoot_data_t; 
	
	typedef __packed struct
 {
     uint16_t bullet_remaining_num_17mm;//17mm ��������������
     uint16_t bullet_remaining_num_42mm;
     uint16_t coin_remaining_num;//ʣ��������
 } ext_bullet_remaining_t;
	


	
	typedef __packed struct//�ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180
	{ 
		float data1;
		float data2;
		float data3; 
		uint8_t masks; 
	} client_custom_data_t ;
	
/*****************************�����˼佻������**************************************/	
	
	typedef __packed struct//�������ݽ�����Ϣ��0x0301������Ƶ�ʣ����� 10Hz 
	{  
		uint16_t data_cmd_id;  
		uint16_t send_ID;  
		uint16_t receiver_ID;
	}ext_student_interactive_header_data_t; 
	
	typedef __packed struct //ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF 
	{ 
		uint8_t data[6];
	} robot_interactive_data_t ;
	
	
	typedef __packed struct  //�ͻ���ɾ��ͼ��
{
	uint8_t operate_tpye; 
	uint8_t layer; 
} ext_client_custom_graphic_delete_t;


typedef __packed struct //ͼ������  �ͻ����Զ���ͼ�� �����˼�ͨ�ţ�0x0301������Ƶ�ʣ����� 10Hz 
{ 
uint8_t graphic_name[3]; 
uint32_t operate_type:3; 
uint32_t graphic_type:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11; 
}graphic_data_struct_t;
	

typedef __packed struct  //�ͻ��˻���һ��ͼ��
{
 graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;


typedef __packed struct  //�ͻ��˻�������ͼ��
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

typedef __packed struct  ////�ͻ��˻������ͼ��
{
  graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

typedef __packed struct  //�ͻ��˻����ַ�
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;


typedef __packed struct  //�ͻ��˻����߸�ͼ��
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

typedef __packed struct
{
float target_position_x;
float target_position_y;
float target_position_z;
uint8_t commd_keyboard;
uint16_t target_robot_ID;
} ext_robot_command_t;

typedef __packed struct
{
uint16_t target_robot_ID;
float target_position_x;
float target_position_y;
} ext_client_map_command_t;

/*********************�����˼佻������end***************************/



/** 
  * @brief  student custom data
  */
typedef __packed struct
{
  float data1;
  float data2;
  float data3;
	uint8_t mask;
} client_show_data_t;//client_show_data

typedef enum
{
  unkown = 0,
  blue = 1,
  red  = 2,
} robot_color_e;

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{ 
		game_status_t                     game_state;//����״̬����
		game_result_t                     game_result;//�����������
		game_robot_HP_t                   game_robot_HP;//�����˴������
		event_data_t                      event_data;//�����¼�����
		ext_supply_projectile_action_t        supply_projectile_action;//����վ������ʶ
	    referee_warning_t                 referee_warning;//���о�����Ϣ
		robot_status_t                game_robot_state;//����������״̬
		power_heat_data_t                 power_heat_data;//ʵʱ������������
		robot_pos_t                  game_robot_pos;//������λ��
		buff_t                       buff_musk;//����������
		air_support_data_t                 aerial_robot_energy;//���л���������״̬
		hurt_data_t                      robot_hurt;//�˺�״̬
		ext_shoot_data_t                      shoot_data;// ʵʱ�����Ϣ
	  ext_bullet_remaining_t                ext_bullet_remaining;

		ext_student_interactive_header_data_t student_interactive_header_data;//�������ݽ�����Ϣ
		client_custom_data_t                  client_custom_data;//�ͻ��� �ͻ����Զ�������
	    robot_interactive_data_t              robot_interactive_data;//�������� �����˼�ͨ��
	    graphic_data_struct_t             graphic_data_struct;//�ͻ����Զ���ͼ�� 
	   ext_robot_command_t                 ext_robot_command;
		 ext_client_map_command_t             ext_client_map_command;

		 robot_color_e                         robot_color;
//		game_robot_state_t game_information;
//		robot_hurt_data_t  blood_changed_data;
//		real_shoot_t       real_shoot_data;
//		PowerHeatData_t    PowerHeatData_data;
//		rfid_detect_t      rfid_data;
//		game_result_t      game_result_data;
//		get_buff_t         get_buff_data;
//		GameRobotPos_t     GameRobotPos_data;
//		server_to_user_t   student_download_data;
} receive_judge_t;

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;



unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
unsigned char get_crc8(unsigned char* data, unsigned int length);

void judgement_data_handle(uint8_t *p_frame,u16 rec_len);

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);

void data_upload_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);

extern receive_judge_t judge_rece_mesg;




#endif
