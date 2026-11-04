#include "main.h"



/**
  * @brief  �ܿ�������
  * @param  Chassis_t        ���̽ṹ��
  * @param  Gimbal_t         ��̨�ṹ��
  * @param  Shooter_t        ����ṹ��
  * @param  Remote_DT7_t     �ؽṹ��
  * @param  Control_Mode_e   ����ģʽ
  * @param  receive_judge_t  ����ϵͳ
  * @retval �޷���ֵ
  * @note   1000Hz���У�ȫ����������
  */
void Control_Task(Gimbal_t* Gimbal,Chassis_t* Chassis,Remote_DT7_t* Remote_DT7_Data,Shooter_t* Shooter,Control_Mode_e* Control_Mode,receive_judge_t* receive_judge)
{
    Control_Mode_Select(Remote_DT7_Data,Control_Mode);//����ģʽѡ��
    Key_Mouse_State_Update(Remote_DT7_Data);//����״̬����
    Remote_SwitchAction_Detect(Remote_DT7_Data);//���ֲ��˶������
    Chassis_Task(Chassis,Remote_DT7_Data,Control_Mode,&Gimbal->Yaw_Motor_Encoder,receive_judge);//���̿�������
    Gimbal_Task(Gimbal,Control_Mode,Remote_DT7_Data);//��̨��������
    Shooter_Task(Shooter,Control_Mode,Remote_DT7_Data,receive_judge);//���������������
}


