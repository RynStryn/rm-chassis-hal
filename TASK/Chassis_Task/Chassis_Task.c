#include "main.h"
Chassis_t Chassis;    

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
}



/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, float num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}



//限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}



//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}


/***************************************************************功率控制**********************************************************************/
/**
  * @brief  功率计算
  * @param  Chassis_t   底盘结构体
  * @param  Current     电流，单位A
  * @param  Omega       角速度，单位rpm
  * @retval 无返回值
  * @note   根据电机功率模型来计算单个电机当前功率
  */
float Power_Calc(Chassis_t* Chassis,float Current,float Omega)
{
    float Power;
    Power = Chassis->Motor_Power_Model.k0  + Chassis->Motor_Power_Model.k1*Current + Chassis->Motor_Power_Model.k2*Omega +
    Chassis->Motor_Power_Model.k3*Current*Omega + Chassis->Motor_Power_Model.k4*Current*Current + Chassis->Motor_Power_Model.k5*Omega*Omega ;
    return Power;
}



/**
  * @brief  求解最大电流
  * @param  Chassis_t   底盘结构体
  * @retval 无返回值
  * @note   根据当前的功率上限，和当前各电机角速度来计算允许的最大电流
  */
void Solve_Max_Current(Chassis_t* Chassis)
{
    float Detla;//求解一元二次方程判别式
    float Omega;//求和以方便求解
    float Omega_Squared;

    float a;//一元二次方程的三个系数
    float b;
    float c;

    Omega = Chassis->Chassis_M2006_Encoder[0].Omega + Chassis->Chassis_M2006_Encoder[1].Omega +
    Chassis->Chassis_M2006_Encoder[2].Omega + Chassis->Chassis_M2006_Encoder[3].Omega ;

    Omega_Squared = Chassis->Chassis_M2006_Encoder[0].Omega*Chassis->Chassis_M2006_Encoder[0].Omega +
    Chassis->Chassis_M2006_Encoder[1].Omega*Chassis->Chassis_M2006_Encoder[1].Omega +
    Chassis->Chassis_M2006_Encoder[2].Omega*Chassis->Chassis_M2006_Encoder[2].Omega +
    Chassis->Chassis_M2006_Encoder[3].Omega*Chassis->Chassis_M2006_Encoder[3].Omega ;

    a = Chassis->Motor_Power_Model.k4;
    b = Chassis->Motor_Power_Model.k3*Omega + Chassis->Motor_Power_Model.k1;
    c = 4.0f*Chassis->Motor_Power_Model.k0 + Chassis->Motor_Power_Model.k2*Omega + Chassis->Motor_Power_Model.k5*Omega_Squared-CHASSIS_WARNING_POWER;

    Detla = b*b - 4.0f*a*c;

    if(Detla>1e-5)
    {
        Chassis->Current_Max = (-(Chassis->Motor_Power_Model.k3*Omega + Chassis->Motor_Power_Model.k1)*(Chassis->Motor_Power_Model.k3*Omega + Chassis->Motor_Power_Model.k1) +
        sqrtf(Detla))/(2.0f*Chassis->Motor_Power_Model.k4);//暂时未考虑多种情况，不知道会不会出问题
    }
    else if(Detla<=1e-5)
        Chassis->Current_Max = 0.5f;//无解给0.5
}



/**
  * @brief  输入速度限制
  * @param  Chassis_t   底盘结构体
  * @retval 无返回值
  * @note   根据允许最大电流逆PID计算解出速度的限制系数，再通过限制系数限制输入速度
  */
void Input_Velocity_Restrict(Chassis_t* Chassis)
{
    for(uint8_t i;i<4;i++)
    {   //先代换，方便写代码
        float a = (Chassis->Chassis_Motor_Speed_PID[i].Kp + Chassis->Chassis_Motor_Speed_PID[i].Kd)*Chassis->Chassis_Motor_Speed_PID[i].set ;//系数a
        float b = -(Chassis->Chassis_Motor_Speed_PID[i].Kp + Chassis->Chassis_Motor_Speed_PID[i].Kd)*Chassis->Chassis_Motor_Speed_PID[i].fdb +
            Chassis->Chassis_Motor_Speed_PID[i].Iout - Chassis->Chassis_Motor_Speed_PID[i].Kd*Chassis->Chassis_Motor_Speed_PID[i].error[1];
        Chassis->Power_Control[i].K_in = (Chassis->Current_Max - b)/a;//求出输入限制系数
    }
    for(uint8_t i;i<4;i++)
    {
        Chassis->Whell_Speed_Ref[i] = Chassis->Power_Control[i].K_in * Chassis->Whell_Speed_Ref[i];
    }
}



/**
  * @brief  功率误差分配
  * @param  Chassis_t   底盘结构体
  * @retval 无返回值
  * @note   通过限制后目标速度和实际速度的误差进行误差分配，误差大的分得功率大
  */
void Power_Error_Distribution(Chassis_t* Chassis)
{
    float Wholl_Error = Chassis->Chassis_Motor_Speed_PID[0].error[0] + Chassis->Chassis_Motor_Speed_PID[1].error[0] +
        Chassis->Chassis_Motor_Speed_PID[2].error[0] + Chassis->Chassis_Motor_Speed_PID[3].error[0];
    for(uint8_t i ; i<4 ; i++)
    {
        Chassis->Power_Control[i].Power_Max = Chassis->Chassis_Motor_Speed_PID[i].error[0]/Wholl_Error*Chassis->Chassis_Warning_Pwoer;
    }
}



/**
  * @brief  输出电流限制
  * @param  Chassis_t   底盘结构体
  * @retval 无返回值
  * @note   通过分配给各电机的功率，求出最后的电流，和输出限制系数
  */
void Output_Current_Restrict(Chassis_t* Chassis)
{
    for(uint8_t i ; i<4 ; i++)
    {
        float a = Chassis->Motor_Power_Model.k4;
        float b = Chassis->Motor_Power_Model.k3 * Chassis->Whell_Speed_Fdb[i] + Chassis->Motor_Power_Model.k1;
        float c = Chassis->Motor_Power_Model.k0 + Chassis->Motor_Power_Model.k2*Chassis->Whell_Speed_Fdb[i] +
            Chassis->Motor_Power_Model.k5*Chassis->Whell_Speed_Fdb[i]*Chassis->Whell_Speed_Fdb[i] - Chassis->Power_Control[i].Power_Max;
        float Detla = b*b - 4.0f*a*c;
        if(Detla>1e-5)
        {
            Chassis->Chassis_Motor_Set_Current[i] = (-b + sqrtf(Detla))/(2.0f*a) * 1000.0f;//实际电流值转换为发给电机的数，对于多解的处理还不完善
        }
        else if(Detla<=1e-5)
        {
            Chassis->Chassis_Motor_Set_Current[i] = 0;
        }
        Chassis->Power_Control[i].K_out = Chassis->Chassis_Motor_Set_Current[i]/Chassis->Chassis_Motor_Speed_PID[i].out ;
    }
}



/**
  * @brief  功率控制总函数
  * @param  Chassis_t   底盘结构体
  * @retval 无返回值
  * @note   无
  */
void Power_Control(Chassis_t* Chassis,receive_judge_t* receive_judge)
{
        Solve_Max_Current(Chassis);
        Input_Velocity_Restrict(Chassis);
        for(uint8_t i ; i<4 ; i++)
        {
            PID_Calc(&Chassis->Chassis_Motor_Speed_PID[i],Chassis->Whell_Speed_Fdb[i],Chassis->Whell_Speed_Ref[i]);
        }
        Power_Error_Distribution(Chassis);
        Output_Current_Restrict(Chassis);

}
/***************************************************************功率控制**********************************************************************/



/**
  * @brief  选择底盘模式
  * @param  Remote_DT7_t   控结构体
  * @param  Control_Mode_e 控制模式结构体
  * @param  Chassis_t   底盘结构体
  * @retval 无返回值
  * @note   实现不同控制模式下底盘模式的选择
  */
void Chassis_Mode_Select(Chassis_t* Chassis,Remote_DT7_t* Remote_DT7,Control_Mode_e* Control_Mode)
{
    switch(*Control_Mode)
    {
        case RELAX://失能模式
            Chassis->Chassis_Mode = CHASSIS_RELAX;
            break;
        case REMOTE://手柄模式
            if(Remote_DT7->Remote_clicker.s2 == UP)             Chassis->Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE;
            else if(Remote_DT7->Remote_clicker.s2 == MIDDLE)    Chassis->Chassis_Mode = CHASSIS_FALLOW_YAW_MOTOR;
            else if(Remote_DT7->Remote_clicker.s2 == DOWN)      Chassis->Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE;
            break;
        case KEY_MOUSE://键鼠模式
            if(Remote_DT7->key.Key_Q_Action.Toggle_Press_Flag == 1 & Remote_DT7->key.Key_E_Action.Toggle_Press_Flag == 0)  //这里判断两个按键，防止俩同时按下
            Chassis->Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE;
            else if(Remote_DT7->key.Key_E_Action.Toggle_Press_Flag == 1 & Remote_DT7->key.Key_Q_Action.Toggle_Press_Flag == 0)
            Chassis->Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE;
            else Chassis->Chassis_Mode = CHASSIS_FALLOW_YAW_MOTOR;
            break;
    }
}



/**
  * @brief  底盘反馈数据更新
  * @param  Chassis_t   底盘结构体
  * @param  Gimbal_t    云台结构体
  * @retval 无返回值
  * @note   更新底盘相关反馈数据，包括速度和角度，为后续控制做准备
  */
void Chassis_Feedback_Update(Chassis_t* Chassis,Encoder* Yaw_Motor_Encoder)
{
    for(uint8_t i = 0;i < 4;i++)   Chassis->Whell_Speed_Fdb[i] = Chassis->Chassis_M2006_Encoder[i].Omega;
    //更新底盘纵向速度 x， 平移速度y，旋转速度wz
    Chassis->V_x_Fdb = (  Chassis->Whell_Speed_Fdb[3] + Chassis->Whell_Speed_Fdb[0] - Chassis->Whell_Speed_Fdb[1] - Chassis->Whell_Speed_Fdb[2] )*0.3535f*MOTOR_REV_TO_WHELL_VELOCITY;
    //0.3535是2^（1/2）/4的结果，这里化简，节约算力
    Chassis->V_y_Fdb = ( - Chassis->Whell_Speed_Fdb[3] + Chassis->Whell_Speed_Fdb[0] + Chassis->Whell_Speed_Fdb[1] - Chassis->Whell_Speed_Fdb[2] )*0.3535f*MOTOR_REV_TO_WHELL_VELOCITY;
    //0.3535是2^（1/2）/4的结果，这里化简，节约算力
    Chassis->Omega_Fdb = ( Chassis->Whell_Speed_Fdb[3] + Chassis->Whell_Speed_Fdb[0] + Chassis->Whell_Speed_Fdb[1] + Chassis->Whell_Speed_Fdb[2] )*MOTOR_REV_TO_WHELL_VELOCITY/0.6f;
    //0.6是0.15m/4的结果，这里化简，节省算力
    Chassis->Relative_Angle_Fdb = (Yaw_Motor_Encoder->raw_value - YAW_ENCODER_OFFSET) * ENCODER_TO_ARC;
}



/**
  * @brief  底盘参考数据更新
  * @param  Chassis_t         底盘结构体
  * @param  Control_Mode_e    控制模式
  * @param  Remote_DT7_t      遥控器数据
  * @retval 无返回值
  * @note   更新底盘数据参考值，但和最终输入到电机的数据有差异
  */
float sin_yaw = 0.0f, cos_yaw = 0.0f;
float V_x,V_y;//遥控器设置速度，以云台为准，单位m/s
int16_t Vx_Channel, Vy_Channel;
void Chassis_Reference_Update(Chassis_t* Chassis,Remote_DT7_t* Remote_DT7_Data,Control_Mode_e* Control_Mode)
{
    sin_yaw = sinf(Chassis->Relative_Angle_Fdb);
    cos_yaw = cosf(Chassis->Relative_Angle_Fdb);

    switch(*Control_Mode)
    {
        case RELAX:
            break;
        case REMOTE:
            rc_deadband_limit(Remote_DT7_Data->Remote_clicker.ch1,Vx_Channel,RC_DEADLINE);//输入死区，以防遥控器体质差异
            rc_deadband_limit(Remote_DT7_Data->Remote_clicker.ch0,Vy_Channel,RC_DEADLINE);
            V_x = Vx_Channel * CHASSIS_VX_RC_SEN;                   //遥控器乘灵敏度为速度 m/s
            V_y = Vy_Channel * CHASSIS_VY_RC_SEN;
            Chassis->V_x_Ref =  cos_yaw * V_x - sin_yaw * V_y;      //云台设置速度映射到底盘，旋转矩阵
            Chassis->V_y_Ref =  sin_yaw * V_x + cos_yaw * V_y;


            first_order_filter_cali(&Chassis->chassis_cmd_slow_set_vx , Chassis->V_x_Ref);//一阶低通滤波做斜坡缓起步
            first_order_filter_cali(&Chassis->chassis_cmd_slow_set_vy , Chassis->V_y_Ref);
            if(Chassis->V_x_Ref < RC_DEADLINE * CHASSIS_VX_RC_SEN & Chassis->V_x_Ref > -RC_DEADLINE * CHASSIS_VX_RC_SEN)//停止时不需要缓慢停止，直接停下
                Chassis->chassis_cmd_slow_set_vx.out = 0.0f;
            if(Chassis->V_y_Ref < RC_DEADLINE * CHASSIS_VY_RC_SEN & Chassis->V_y_Ref > -RC_DEADLINE * CHASSIS_VY_RC_SEN)
                Chassis->chassis_cmd_slow_set_vy.out = 0.0f;
            if(Chassis->Chassis_Mode == CHASSIS_CLOCKWISE_ROTATE)
            {
                Chassis->Omega_Ref = CHASSIS_CLOCKWISE_ROTATE_REV;
            }
            else if(Chassis->Chassis_Mode == CHASSIS_ANTI_CLOCKWISE_ROTATE)
            {
                Chassis->Omega_Ref =  CHASSIS_ANTI_CLOCKWISE_ROTATE_REV;
            }
            else if(Chassis->Chassis_Mode == CHASSIS_FALLOW_YAW_MOTOR)
            {
                Chassis->Omega_Ref = PID_Calc(&Chassis->Chassis_Angle_PID,Chassis->Relative_Angle_Fdb,0.0f);//底盘跟随云台，相对角度参考值为0
            }
            break;
        case KEY_MOUSE:
            V_x = (Remote_DT7_Data->key.Key_W_Action.Long_Press_Flag - Remote_DT7_Data->key.Key_S_Action.Long_Press_Flag) * CHASSIS_MAX_SPEED;
            V_y = (Remote_DT7_Data->key.Key_A_Action.Long_Press_Flag - Remote_DT7_Data->key.Key_D_Action.Long_Press_Flag) * CHASSIS_MAX_SPEED;
            Chassis->V_x_Ref =  cos_yaw * V_x - sin_yaw * V_y;      //云台设置速度映射到底盘
            Chassis->V_y_Ref =  sin_yaw * V_x + cos_yaw * V_y;

            first_order_filter_cali(&Chassis->chassis_cmd_slow_set_vx , Chassis->V_x_Ref);//一阶低通滤波做斜坡缓起步
            first_order_filter_cali(&Chassis->chassis_cmd_slow_set_vy , Chassis->V_y_Ref);
            if(Chassis->V_x_Ref < RC_DEADLINE * CHASSIS_VX_RC_SEN & Chassis->V_x_Ref > -RC_DEADLINE * CHASSIS_VX_RC_SEN)//停止时不需要缓慢停止，直接停下
                Chassis->chassis_cmd_slow_set_vx.out = 0.0f;
            if(Chassis->V_y_Ref < RC_DEADLINE * CHASSIS_VY_RC_SEN & Chassis->V_y_Ref > -RC_DEADLINE * CHASSIS_VY_RC_SEN)
                Chassis->chassis_cmd_slow_set_vy.out = 0.0f;
            if(Chassis->Chassis_Mode == CHASSIS_CLOCKWISE_ROTATE)
            {
                Chassis->Omega_Ref = CHASSIS_CLOCKWISE_ROTATE_REV;

            }
            else if(Chassis->Chassis_Mode == CHASSIS_ANTI_CLOCKWISE_ROTATE)
            {
                Chassis->Omega_Ref = CHASSIS_ANTI_CLOCKWISE_ROTATE_REV;

            }
            else if(Chassis->Chassis_Mode == CHASSIS_FALLOW_YAW_MOTOR)
            {
                Chassis->Omega_Ref = PID_Calc(&Chassis->Chassis_Angle_PID,Chassis->Relative_Angle_Fdb,0.0f);//底盘跟随云台，相对角度参考值为0
            }
            break;
    }
}



/**
  * @brief  底盘控制循环
  * @param  Chassis_t         底盘结构体
  * @param  receive_judge_t   裁判系统
  * @retval 无返回值
  * @note   计算出最终要发送给电机的数据
  */
void Chassis_Control_Loop(Chassis_t* Chassis,Control_Mode_e* Control_Mode,receive_judge_t* receive_judge)
{
    switch (*Control_Mode)
      {
         case REMOTE:
         case KEY_MOUSE :

            Chassis->Whell_Speed_Ref[0] = ( 0.707f * Chassis->chassis_cmd_slow_set_vx.out
            - 0.707f * Chassis->chassis_cmd_slow_set_vy.out + MOTOR_DISTANCE_TO_CENTER * Chassis->Omega_Ref)/MOTOR_REV_TO_WHELL_VELOCITY;//左后

            Chassis->Whell_Speed_Ref[1] = (-0.707f * Chassis->chassis_cmd_slow_set_vx.out
            - 0.707f * Chassis->chassis_cmd_slow_set_vy.out + MOTOR_DISTANCE_TO_CENTER * Chassis->Omega_Ref)/MOTOR_REV_TO_WHELL_VELOCITY;//右后

            Chassis->Whell_Speed_Ref[2] = (-0.707f * Chassis->chassis_cmd_slow_set_vx.out
            + 0.707f * Chassis->chassis_cmd_slow_set_vy.out + MOTOR_DISTANCE_TO_CENTER * Chassis->Omega_Ref)/MOTOR_REV_TO_WHELL_VELOCITY;//右前

            Chassis->Whell_Speed_Ref[3] = ( 0.707f * Chassis->chassis_cmd_slow_set_vx.out
            + 0.707f * Chassis->chassis_cmd_slow_set_vy.out + MOTOR_DISTANCE_TO_CENTER * Chassis->Omega_Ref)/MOTOR_REV_TO_WHELL_VELOCITY;//左前
            //全向轮速度逆解，解算到轮组

            //计算功率，判断是否需要进行功控
            Chassis->Chassis_Power_Limit = receive_judge->game_robot_state.chassis_power_limit ;
            Chassis->Chassis_Warning_Pwoer = Chassis->Chassis_Power_Limit - 10 ;//以最大功率减10为安全功率

            for(uint8_t i=0;i<4;i++) Chassis->Power_Control[i].Motor_Power =
            Power_Calc(Chassis , Chassis->Chassis_M2006_Encoder[i].current , Chassis->Chassis_M2006_Encoder[i].Omega);

            Chassis->Chassis_Power = Chassis->Power_Control[0].Motor_Power + Chassis->Power_Control[1].Motor_Power +
            Chassis->Power_Control[2].Motor_Power + Chassis->Power_Control[3].Motor_Power ;//计算底盘总功率

            if(Chassis->Chassis_Power >= Chassis->Chassis_Warning_Pwoer)//超过预警功率就开始控制
            {
                Power_Control(Chassis,receive_judge);
            }
            else//没超过预警功率就正常
            {
                for(uint8_t i=0 ; i<4 ; i++)
                Chassis->Chassis_Motor_Set_Current[i] = PID_Calc ( &Chassis->Chassis_Motor_Speed_PID[i] , Chassis->Whell_Speed_Fdb[i] , Chassis->Whell_Speed_Ref[i]);
            }
            break;
         case RELAX:
            for(uint8_t i=0 ; i<4 ; i++)  Chassis->Chassis_Motor_Set_Current[i] = 0;
            break;
    }
}




/**
  * @brief  底盘总控制任务
  * @param  Chassis_t        底盘结构体
  * @param  Control_Mode_e   控制模式
  * @param  Encoder          Yaw轴编码器
  * @param  receive_judge_t  裁判系统
  * @retval 无返回值
  * @note
  */
int16_t CNT;
void Chassis_Task(Chassis_t* Chassis,Remote_DT7_t* Remote_DT7_Data,Control_Mode_e* Control_Mode,Encoder* Yaw_Motor_Encoder,receive_judge_t* receive_judge)
{
        Chassis_Mode_Select(Chassis,Remote_DT7_Data,Control_Mode);//模式选择
        Chassis_Feedback_Update(Chassis,Yaw_Motor_Encoder);//反馈值更新
        Chassis_Reference_Update(Chassis,Remote_DT7_Data,Control_Mode);//控制值更新
        Chassis_Control_Loop(Chassis,Control_Mode,receive_judge);//最终发送电流计算
}



