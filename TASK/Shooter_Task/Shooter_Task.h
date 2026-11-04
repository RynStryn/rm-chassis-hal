#ifndef __SHOOTER_TASK_H
#define __SHOOTER_TASK_H

#include "CAN_Send_Task.h"
#include "PID.h"
#include "Remote_Task.h"
#include "Judge_System.h"


#define         M2006_ENCODER_TO_DEG                  0.002197f
#define         FRIC_FIRE_CURRENT                     1.4f//��Ҫ����
#define         ACTIVATE_TO_HANDLE_TIME               1000
#define         POKE_JAMMED_CURRENT                   8.0f//��Ҫ����



/*********************************����ṹ��**********************************/
typedef enum
{
    SHOOTER_RELAX = 0,//�������ʧ��
    SINGLE_FIRE,
    BURST_FIRE,
    STOP_FIRE,
}Shooter_Mode_e;


typedef enum
{
    NORMAL = 0        ,
	JAMMED            ,//����
    JAMMED_HANDLE     ,//��������
}Shooter_State_e;


typedef enum
{
    FSM_RELAX = 0,
    ACTIVATE,
    HANDLE,
}CF_FSM_State_e;//ͣ������״̬�������ƿճ̺�˫����


typedef enum
{
    POKE_OFF = 0,
    POKE_ON,
}Poke_Mode_e;//����״̬��ֻ������


typedef enum
{
    FRIC_OFF = 0,
    FRIC_ON ,
}Fric_Mode_e;//Ħ����״̬��ֻ������



typedef enum
{
    STOP = 0,//ͣ��
    DETECT,//���
    CONFIRM,//ȷ�Ϸ���
}Heat_Detect_FSM_e;//�������״̬��


typedef struct
{
    Heat_Detect_FSM_e Heat_Detect_FSM;//�������״̬��
    
    uint16_t Now_Heat;//��ǰ����
//    uint16_t Warn_Heat;//����Ԥ����ֵ
    uint8_t Heat_CD;//��ȴ�����ٶȣ�����ϵͳ��
    uint16_t Residue_Heat;//ʣ������
    uint16_t Threshold_Heat;//����������ֵ
    uint16_t Max_Heat;//����ϵͳ���������
    
    uint16_t Heat_Detect_CNT;//����������
}Heat_Restrict_t;


typedef struct
{
    
    Shooter_Mode_e Shooter_Mode;//�������ģʽ
    Shooter_Mode_e Last_Shooter_Mode;//��һ�η������ģʽ
    Shooter_State_e Shooter_State;//�������״̬
    CF_FSM_State_e CF_FSM_State;//ͣ������״̬��
    Poke_Mode_e Poke_Mode;//����ģʽ
    Fric_Mode_e Fric_Mode;//Ħ����ģʽ
    
    Encoder Poke_M2006_Encoder;//���̵��������
    Encoder Fric_M3508_Encoder[2];//Ħ���ֵ��������
    
    PID_t Poke_Speed_PID;//�����ٶ�PID
    PID_t Poke_Angle_PID;//���̽Ƕ�PID
    PID_t Fric_Speed_PID[2];//Ħ�����ٶ�PID
    
    Heat_Restrict_t Heat_Restrict;//��������
    
    float Poke_Angle_Ref;//���̽ǶȲο�ֵ����λ ��
    float Poke_Angle_Fdb;//���̽Ƕȷ���ֵ����λ ��
    
    float Poke_Speed_Ref;//�����ٶȲο�ֵ����λrpm
    float Poke_Speed_Fdb;//�����ٶȷ���ֵ����λrpm
    
    float Fric_Speed_Ref[2];//Ħ�����ٶȲο�ֵ,��λrpm
    float Fric_Speed_Fdb[2];//Ħ�����ٶȷ���ֵ����λrpm
    
    uint16_t Poke_Jammed_CNT;//���̿������������̵����������Ӽ���
    uint16_t Poke_Jammed_Handle_CNT;//���̿�������������ʱ�䵽֮��ع鵽����ģʽ
    uint16_t Fric_Fire_CNT;//ͣ������״̬���ã�Ħ���ֵ��������Ӽ���
    uint16_t FSM_Activate_CNT;//״̬�������������ﵽĿ��ֱ��ת��״̬
    
    int16_t Poke_Motor_Set_Current;
    int16_t Fric_Motor_Ser_Current[2];
    
}Shooter_t;
/*********************************����ṹ��**********************************/




/*********************************�ⲿ����**********************************/
extern Shooter_t Shooter;
/*********************************�ⲿ����**********************************/




/*********************************��������************************************/
void Shooter_Mode_Select(Shooter_t* Shooter,Control_Mode_e* Control_Mode,Remote_DT7_t* Remote);
void Shooter_Feedback_Update(Shooter_t* Shooter);
void Shooter_State_Update(Shooter_t* Shooter);
float Bullet_Frequency_Select(receive_judge_t* receive_judge);
void Heat_Restrict(Shooter_t* Shooter,receive_judge_t* receive_judge);
void Shooter_Reference_Update(Shooter_t* Shooter,receive_judge_t* receive_judge);
void Shooter_Control_Loop(Shooter_t* Shooter);
void Shooter_Task(Shooter_t* Shooter,Control_Mode_e* Control_Mode,Remote_DT7_t* Remote,receive_judge_t* receive_judge);
/*********************************��������************************************/



#endif
