#ifndef __CHASSIS_TASK
#define __CHASSIS_TASK
#include <stdint.h>
#include "PID.h"
#include "CAN_Send_Task.h"
#include "Remote_Task.h"
#include "Judge_System.h"


#define   MOTOR_REV_TO_WHELL_VELOCITY        0.0001847f              //������ٶ�rpmת��Ϊ�������ٶ�m/s��ϵ��
#define   YAW_ENCODER_OFFSET                 4346                    //Yaw������ƫ�ƣ�����������������̨�ĽǶ�
#define   ENCODER_TO_ARC                     0.00076699039f          //������ԭʼ��е�Ƕ�ת��Ϊ���ȵ�ת��ϵ��

#define   CHASSIS_VX_RC_SEN                  0.004f                  //ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define   CHASSIS_VY_RC_SEN                  0.004f                  //ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���

#define   RC_DEADLINE                        10                      //ҡ������

#define CHASSIS_MAX_SPEED                    2.64                   //����ٶ�2.6m/s

#define CHASSIS_CLOCKWISE_ROTATE_REV         -7.0f                 //С����ת��
#define CHASSIS_ANTI_CLOCKWISE_ROTATE_REV    7.0f  
#define MOTOR_DISTANCE_TO_CENTER             0.15f   //����ͶӰ�㵽������ת���ĵľ��� 0.15m

#define CHASSIS_CONTROL_TIME                 0.001f  //����������Ƽ�� 0.001s
#define CHASSIS_ACCEL_X_NUM                  0.8f
#define CHASSIS_ACCEL_Y_NUM                  0.8f

#define CHASSIS_WARNING_POWER                60.0f //����Ԥ�����ʣ�������������Ҫ�޸�

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
/**********************************���̽ṹ��*************************************/
typedef enum
{
    CHASSIS_RELAX = 0,              //ʧ��ģʽ
    CHASSIS_FALLOW_YAW_MOTOR,       //���̸�����̨ģʽ
    CHASSIS_NOT_FALLOW_YAW_MOTOR,   //���̲�������̨ģʽ
    CHASSIS_CLOCKWISE_ROTATE,       //˳ʱ��С����ģʽ
    CHASSIS_ANTI_CLOCKWISE_ROTATE,  //��ʱ��С����ģʽ
}Chassis_Mode_e; 

typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num;       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

typedef struct
{
    float k0;//�������ģ�͵�6��ϵ��
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
}Motor_Power_Model_t;

typedef struct
{
   float K_in;//�����ٶ�����ϵ��
   float K_out;//�����������ϵ��
    
   float Motor_Power;//����ģ����ϵĹ���
   float Power_Max;//�������ÿ�ŵ�������ƹ���
}Power_Control_t;

typedef struct
{
    float V_x_Ref;//����ˮƽ�����ٶȲο�ֵ
    float V_y_Ref;//������ֱ�����ٶȲο�ֵ
    
    float V_x_Fdb;//����ˮƽ�����ٶȷ���ֵ
    float V_y_Fdb;//������ֱ�����ٶȷ���ֵ
    
    float Omega_Ref;//���̽��ٶȲο�ֵ����λrad/s
    float Omega_Fdb;//���̽��ٶȷ���ֵ����λrad/s
    
    Chassis_Mode_e	Chassis_Mode;//����ģʽ�ṹ��
    Chassis_Mode_e	Chassis_Mode_Last;//��һʱ�̵���ģʽ�ṹ��
    
    float Whell_Speed_Ref[4];//������ٶȲο�ֵ����λrpm   
	float Whell_Speed_Fdb[4];//������ٶȷ���ֵ����λrpm
    
    PID_t Chassis_Motor_Speed_PID[4];//���̵��PID
    PID_t Chassis_Angle_PID;//���̽Ƕ�PID�����ڵ��̸�����̨
    
    int16_t Chassis_Motor_Set_Current[4];//���̵����������ֵ
    
    Encoder Chassis_M2006_Encoder[4];//���̵������������
    float Relative_Angle_Ref;//�͵�����ԽǶȵĲο�ֵ
    float Relative_Angle_Fdb;//�͵�����ԽǶȵķ���ֵ
    
    first_order_filter_type_t chassis_cmd_slow_set_vx;//ʹ��һ�׵�ͨ�˲������趨ֵ
    first_order_filter_type_t chassis_cmd_slow_set_vy;//ʹ��һ�׵�ͨ�˲������趨ֵ
    
    Power_Control_t Power_Control[4];//�ĸ��ֵĹ��ʿ���
    float Chassis_Power;//�����ܹ���
    uint16_t Chassis_Power_Limit;//�Ӳ���ϵͳ���ĵ��̹�������
    uint16_t Chassis_Warning_Pwoer;//����Ԥ�����ʣ������Ϳ�ʼ��������
    Motor_Power_Model_t Motor_Power_Model;//���̵������ģ�Ͳ���
    float Current_Max;//��������ǽʱ�������ʱ����
}Chassis_t;
/**********************************���̽ṹ��*************************************/

extern Chassis_t Chassis;

/***********************************��������******************************************/
void Chassis_Mode_Select(Chassis_t* Chassis,Remote_DT7_t* Remote_DT7,Control_Mode_e* Control_Mode);
void Chassis_Feedback_Update(Chassis_t* Chassis,Encoder* Yaw_Motor_Encoder);
void Chassis_Reference_Update(Chassis_t* Chassis,Remote_DT7_t* Remote_DT7_Data,Control_Mode_e* Control_Mode);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, float num);
float float_clamp(float Value, float minValue, float maxValue);
int16_t int16_clamp(int16_t Value, int16_t minValue, int16_t maxValue);
void Chassis_Control_Loop(Chassis_t* Chassis,Control_Mode_e* Control_Mode,receive_judge_t* receive_judge);
void Solve_Max_Current(Chassis_t* Chassis);
float Power_Calc(Chassis_t* Chassis,float Current,float Omega);
void Power_Control(Chassis_t* Chassis,receive_judge_t* receive_judge);
void Input_Velocity_Restrict(Chassis_t* Chassis);
void Power_Error_Distribution(Chassis_t* Chassis);
void Output_Current_Restrict(Chassis_t* Chassis);
void Chassis_Task(Chassis_t* Chassis,Remote_DT7_t* Remote_DT7_Data,Control_Mode_e* Control_Mode,Encoder* Yaw_Motor_Encoder,receive_judge_t* receive_judge);
/***********************************��������******************************************/




#endif

