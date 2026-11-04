#include "main.h"

Gimbal_t Gimbal;

/**
  * @brief  云台模式选择
  * @param  Gimbal_t        云台结构体
  * @param  Control_Mode_e  控制模式 
  * @retval 无返回值
  * @note   计算出最终要发送给电机的数据
  */
void Gimbal_Mode_Select(Gimbal_t* Gimbal,Control_Mode_e* Control_Mode)
{
    switch(*Control_Mode)
    {
        case RELAX:
            Gimbal->Gimbal_Mode = GIMBAL_RELAX;
            break;
        default :
            Gimbal->Gimbal_Mode = GIMBAL_FALLWO_GYRO;//其他行为模式时，云台都跟随陀螺仪
            break;
    }
}


/**
  * @brief  云台反馈值更新
  * @param  Gimbal_t    云台结构体
  * @retval 无返回值
  * @note   更新反馈值，为后续控制做准备
  */
void Gimbal_Feedback_Update(Gimbal_t* Gimbal)
{
    Gimbal->Yaw_Angle_To_Chassis = (Gimbal->Yaw_Motor_Encoder.raw_value - YAW_ENCODER_OFFSET)*ENCODER_TO_ARC;//相对角度
    Gimbal->Yaw_Angle_Fdb = Gimbal->CH040_Data.Yaw_Multi_Angle ;
    Gimbal->Yaw_Speed_Fdb = Gimbal->CH040_Data.Yaw_Gyro_Omega ;
    Gimbal->Pitch_Angle_Fdb = Gimbal->CH040_Data.Pitch_Angle ;
    Gimbal->Pitch_Speed_Fdb = Gimbal->CH040_Data.Pitch_Gyro_Omega ;
}


/**
  * @brief  云台控制值更新
  * @param  Gimbal_t    云台结构体
  * @param  Remote_DT7_t   
  * @param  Control_Mode_e    
  * @retval 无返回值
  * @note   更新控制值
  */
int16_t Yaw_Channel, Pitch_Channel;
void Gimbal_Reference_Update(Gimbal_t* Gimbal,Remote_DT7_t* Remote_DT7_Data,Control_Mode_e* Control_Mode)
{
    if(Gimbal->Gimbal_Mode == RELAX & Gimbal->Gimbal_Mode_Last != RELAX) //模式切换瞬间，保存当前数据作为目标值
    {
        Gimbal->Yaw_Angle_Ref = Gimbal->Yaw_Angle_Fdb ;
        Gimbal->Pitch_Angle_Ref = Gimbal->Pitch_Angle_Fdb ;
    }
    else if(Gimbal->Gimbal_Mode == GIMBAL_FALLWO_GYRO & Gimbal->Gimbal_Mode_Last != GIMBAL_FALLWO_GYRO)
    {
        Gimbal->Yaw_Angle_Ref = Gimbal->Yaw_Angle_Fdb ;
        Gimbal->Pitch_Angle_Ref = Gimbal->Pitch_Angle_Fdb ;
    }
    
    switch(*Control_Mode)
    {
        case RELAX:
            break;
        case KEY_MOUSE:
            Gimbal->Pitch_Angle_Ref -= Remote_DT7_Data->Remote_mouse.y * MOUSE_SENSITIVITY;  //灵敏度和+-号没调
            Gimbal->Yaw_Angle_Ref -= Remote_DT7_Data->Remote_mouse.x * MOUSE_SENSITIVITY;
            break;
        case REMOTE:
            rc_deadband_limit(Remote_DT7_Data->Remote_clicker.ch3,Pitch_Channel,RC_DEADLINE);//输入死区
            rc_deadband_limit(Remote_DT7_Data->Remote_clicker.ch2,Yaw_Channel,RC_DEADLINE);
            Gimbal->Pitch_Angle_Ref -=Pitch_Channel * REMOTE_SENSITIVITY;
            Gimbal->Yaw_Angle_Ref -= Yaw_Channel * REMOTE_SENSITIVITY;
            break;
    }
     Gimbal->Gimbal_Mode_Last = Gimbal->Gimbal_Mode;//保存本次模式作为下一次的上一次
}


/**
  * @brief  云台控制
  * @param  Gimbal_t    云台结构体
  * @param  Control_Mode_e    
  * @retval 无返回值
  * @note   计算直接输入电机的数值，准备发送
  */
void Gimbal_Control_Loop(Gimbal_t* Gimbal,Control_Mode_e* Control_Mode)
{
    switch(*Control_Mode)
    {
        
        case REMOTE :
        case KEY_MOUSE :
            Gimbal->Yaw_Speed_Ref = PID_Calc(&Gimbal->Gimbal_Motor_Angle_PID[0],Gimbal->Yaw_Angle_Fdb , Gimbal->Yaw_Angle_Ref);
            Gimbal->Gimbal_Motor_Set_Current[0] = PID_Calc(&Gimbal->Gimbal_Motor_Speed_PID[0],Gimbal->Yaw_Speed_Fdb , Gimbal->Yaw_Speed_Ref);
            Gimbal->Pitch_Speed_Ref = PID_Calc(&Gimbal->Gimbal_Motor_Angle_PID[1],Gimbal->Pitch_Angle_Fdb , Gimbal->Pitch_Angle_Ref);
            Gimbal->Gimbal_Motor_Set_Current[1] = PID_Calc(&Gimbal->Gimbal_Motor_Speed_PID[1],Gimbal->Pitch_Speed_Fdb , Gimbal->Pitch_Speed_Ref);
            break;
        case RELAX:
            for(uint8_t i=0 ;i < 2;i++) Gimbal->Gimbal_Motor_Set_Current[i] = 0;
            break;
    }
}


/**
  * @brief  云台任务
  * @param  Gimbal_t        云台结构体
  * @param  Control_Mode_e  控制模式
  * @param  Remote_DT7_t    控结构体
  * @retval 无返回值
  * @note   
  */
void Gimbal_Task(Gimbal_t* Gimbal,Control_Mode_e* Control_Mode,Remote_DT7_t* Remote_DT7_Data)
{
    Gimbal_Mode_Select(Gimbal,Control_Mode);//选择控制模式
    Gimbal_Feedback_Update(Gimbal);//反馈数据更新
    Gimbal_Reference_Update(Gimbal,Remote_DT7_Data,Control_Mode);//控制数据更新
    Gimbal_Control_Loop(Gimbal,Control_Mode);//PID计算
}

