#include "main.h"
Shooter_t Shooter;

/**
  * @brief  发射机构模式选择
  * @param  Shooter_t        发射机构
  * @param  Remote_DT7_t     控结构体
  * @param  Control_Mode_e   控制模式
  * @retval 无返回值
  * @note   无
  */
void Shooter_Mode_Select(Shooter_t* Shooter,Control_Mode_e* Control_Mode,Remote_DT7_t* Remote)
{
    /**********************************************发射机构模式选择**********************************************************/
    switch (*Control_Mode)
    {
        case RELAX:
            Shooter->Shooter_Mode = SHOOTER_RELAX;
            break;
        case REMOTE:
            if(Shooter->Fric_Mode == FRIC_ON)   //关摩擦轮一定是失能状态
            {
                if(Remote->Remote_clicker.Trigger_Down_Action.Short_Press_Flag == 1)
                {
                    Shooter->Shooter_Mode = SINGLE_FIRE ;
                }
                else if(Remote->Remote_clicker.Trigger_Down_Action.Long_Press_Flag == 1)
                {
                    Shooter->Shooter_Mode = BURST_FIRE ;
                }
                else 
                    Shooter->Shooter_Mode = STOP_FIRE ;
           }
           else if(Shooter->Fric_Mode == FRIC_OFF)
           {
               Shooter->Shooter_Mode = SHOOTER_RELAX ;
           }
           break;
        case KEY_MOUSE:
            if(Shooter->Fric_Mode == FRIC_ON)
            {
                if(Remote->Remote_mouse.Press_L_Action.Short_Press_Flag == 1)
                {
                    Shooter->Shooter_Mode = SINGLE_FIRE ;
                }
                else if(Remote->Remote_mouse.Press_L_Action.Long_Press_Flag  == 1)
                {
                    Shooter->Shooter_Mode = BURST_FIRE ;
                }
                else 
                    Shooter->Shooter_Mode = STOP_FIRE ;
           }
           else if(Shooter->Fric_Mode == FRIC_OFF)
           {
               Shooter->Shooter_Mode = SHOOTER_RELAX ;
           }
           break;
        default :
            Shooter->Shooter_Mode = SHOOTER_RELAX;
            break;
    }
    /**********************************************发射机构模式选择**********************************************************/
    
    
    /***********************************************摩擦轮开关选择*********************************************************/
    switch (*Control_Mode)
    {
        case RELAX:
            Shooter->Fric_Mode = FRIC_OFF;
            break;
        case REMOTE:
            if(Remote->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag == 1)
            {
                Shooter->Fric_Mode = FRIC_ON;
            }
            else if(Remote->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag == 0)
            {
                Shooter->Fric_Mode = FRIC_OFF;
            }
            break;
        case KEY_MOUSE:
            if(Remote->key.Key_C_Action.Toggle_Press_Flag  == 1)
            {
                Shooter->Fric_Mode = FRIC_ON;
            }
            else if(Remote->key.Key_C_Action.Toggle_Press_Flag == 0)
            {
                Shooter->Fric_Mode = FRIC_OFF;
            }
            break;
    }
    /***********************************************摩擦轮开关选择*********************************************************/
    
    
    /***********************************************拨盘开关选择*********************************************************/
    switch (Shooter->Shooter_Mode)
    {
        case SHOOTER_RELAX :
            Shooter->Poke_Mode = POKE_OFF;
            break;
        case STOP_FIRE :
        case SINGLE_FIRE :
        case BURST_FIRE :
            Shooter->Poke_Mode = POKE_ON;
            break;
        default :
            Shooter->Poke_Mode = POKE_OFF;
            break;
    }
    /***********************************************拨盘开关选择*********************************************************/
    
}




/**
  * @brief  发射机构反馈值更新
  * @param  Shooter_t        发射机构
  * @retval 无返回值
  * @note   无
  */
void Shooter_Feedback_Update(Shooter_t* Shooter)
{
    Shooter->Fric_Speed_Fdb[0] = Shooter->Fric_M3508_Encoder[0].Omega ;
    Shooter->Fric_Speed_Fdb[1] = Shooter->Fric_M3508_Encoder[1].Omega ;
    
    Shooter->Poke_Angle_Fdb = Shooter->Poke_M2006_Encoder.ecd_value * M2006_ENCODER_TO_DEG;
    Shooter->Poke_Speed_Fdb = Shooter->Poke_M2006_Encoder.Omega ; 
}




/**
  * @brief  弹频选择
  * @param  receive_judge_t        裁判系统
  * @retval 无返回值
  * @note   根据机器人的等级，选择不同弹频，单位n/s，颗每秒
  */
float Bullet_Frequency_Select(receive_judge_t* receive_judge)
{
    float Bullet_Frequency;
    switch (receive_judge->game_robot_state.robot_level)
    {
        case 1:
            Bullet_Frequency = 1;//没改
            break;
        case 2:
            Bullet_Frequency = 2;
            break;
        case 3:
            Bullet_Frequency = 3;
            break;
        case 4:
            Bullet_Frequency = 4;
            break;
        case 5:
            Bullet_Frequency = 5;
            break;
        case 6:
            Bullet_Frequency = 6;
            break;
        case 7:
            Bullet_Frequency = 7;
            break;
        case 8:
            Bullet_Frequency = 8;
            break;
        case 9:
            Bullet_Frequency = 9;
            break;
        case 10:
            Bullet_Frequency = 10;
            break;
    }
    return Bullet_Frequency;
}



/**
  * @brief  发射机构状态改变
  * @param  Shooter_t        发射机构
  * @retval 无返回值
  * @note   检测，切换发射机构的状态，包括停火有限状态机，卡弹检测，热量限制打出子弹检测
  */
void Shooter_State_Update(Shooter_t* Shooter)
{
    //停火有限状态机
    switch (Shooter->CF_FSM_State)//失能模式下，点鼠标准备单发打子弹的瞬间，激活停火有限状态机，准备检测打弹
    {
        case FSM_RELAX:
            if(Shooter->Shooter_Mode == STOP_FIRE & Shooter->Last_Shooter_Mode == SINGLE_FIRE)
            {
                Shooter->CF_FSM_State = ACTIVATE ;
            }
            break;
        case ACTIVATE:
            Shooter->Fric_Fire_CNT++;
            if((Shooter->Fric_M3508_Encoder[0].current + Shooter->Fric_M3508_Encoder[1].current)/2.0f >= FRIC_FIRE_CURRENT)
            {
                Shooter->CF_FSM_State = HANDLE ;
                Shooter->Fric_Fire_CNT = 0;
            }
            else if(Shooter->Fric_Fire_CNT == ACTIVATE_TO_HANDLE_TIME)
            {
                Shooter->CF_FSM_State = HANDLE ;
                Shooter->Fric_Fire_CNT = 0;
            }
            break;
        case HANDLE:
            Shooter->Shooter_Mode = STOP_FIRE ;         //打出子弹后立刻停火
            Shooter->CF_FSM_State = FSM_RELAX ;
            break;
    }
    
    
    //发射机构状态
    switch (Shooter->Shooter_State)
    {
        case NORMAL:
            if(Shooter->Poke_M2006_Encoder.current >= POKE_JAMMED_CURRENT)
            {
                Shooter->Poke_Jammed_CNT ++;
            }
            else if(Shooter->Poke_M2006_Encoder.current < POKE_JAMMED_CURRENT)
            {
                Shooter->Poke_Jammed_CNT = 0;
            }
            
            if(Shooter->Poke_Jammed_CNT == 300)
            {
                Shooter->Shooter_State = JAMMED;
                Shooter->Poke_Jammed_CNT = 0;
            }
            break;
        case JAMMED:
            Shooter->Shooter_State = JAMMED_HANDLE;
            break;
        case JAMMED_HANDLE:
            Shooter->Poke_Jammed_Handle_CNT ++;
            if(Shooter->Poke_Jammed_Handle_CNT == 200)
            {
                Shooter->Shooter_State = NORMAL;
                Shooter->Poke_Jammed_Handle_CNT = 0;
            }
            break;
            
    }
    
    //热量检测状态机
    switch(Shooter->Heat_Restrict.Heat_Detect_FSM)
    {
        case STOP:
            if(Shooter->Fric_Mode == FRIC_ON)
            {
                Shooter->Heat_Restrict.Heat_Detect_FSM = DETECT;//开摩擦轮后准备检测发弹
            }
            break;
        case DETECT:
            if((Shooter->Fric_M3508_Encoder[0].current + Shooter->Fric_M3508_Encoder[1].current)/2.0f >= FRIC_FIRE_CURRENT)
            {
                Shooter->Heat_Restrict.Heat_Detect_CNT ++;
            }
            
            if(Shooter->Heat_Restrict.Heat_Detect_CNT == 20)
            {
                Shooter->Heat_Restrict.Heat_Detect_CNT = 0;
                Shooter->Heat_Restrict.Heat_Detect_FSM = CONFIRM;
            }
            
            if(Shooter->Fric_Mode == FRIC_OFF)
            {
                Shooter->Heat_Restrict.Heat_Detect_FSM = STOP;//关摩擦轮状态机关
            }
            break;
        case CONFIRM://确认打出弹后立刻接上检测状态，确认打弹的状态处理在Heat_Restrict
            Shooter->Heat_Restrict.Heat_Detect_FSM = DETECT;
            break;
    }    
}




/**
  * @brief  热量限制
  * @param  Shooter_t             发射机构
  * @param  receive_judge_t       裁判系统
  * @retval 无返回值
  * @note   从裁判系统读最大热量和热量冷却，打出子弹加热量，根据时间冷却热量，若达到热量门限阈值，发射机构切入停火模式
  */
void Heat_Restrict(Shooter_t* Shooter,receive_judge_t* receive_judge)
{
    
    Shooter->Heat_Restrict.Max_Heat = receive_judge->game_robot_state.shooter_barrel_heat_limit;
    Shooter->Heat_Restrict.Threshold_Heat = 30;
    Shooter->Heat_Restrict.Heat_CD = receive_judge->game_robot_state.shooter_barrel_cooling_value;
    //根据等级不同，限制的热量也会不同，之后加
    
    if(Shooter->Heat_Restrict.Heat_Detect_FSM == CONFIRM)//确认状态只会出现一遍
    {
        Shooter->Heat_Restrict.Now_Heat += 10;
    }
    
    if(Shooter->Heat_Restrict.Now_Heat != 0)
    {
        Shooter->Heat_Restrict.Now_Heat -= Shooter->Heat_Restrict.Heat_CD * 0.001f;
    }
    
    Shooter->Heat_Restrict.Residue_Heat = Shooter->Heat_Restrict.Max_Heat - Shooter->Heat_Restrict.Now_Heat;
    if( Shooter->Heat_Restrict.Residue_Heat <= Shooter->Heat_Restrict.Threshold_Heat)
    {
        Shooter->Shooter_Mode = STOP_FIRE;//达到热量门限阈值，发射机构切入停火模式
    }
    
}


/**
  * @brief  发射机构参考值更新
  * @param  Shooter_t     发射机构
  * @retval 无返回值
  * @note   包括拨盘正常的速度控制，停火的瞬间定角度，卡弹时回拨一个角度和摩擦轮的定速控制
  */
void Shooter_Reference_Update(Shooter_t* Shooter,receive_judge_t* receive_judge)
{
    switch (Shooter->Shooter_State)
    {
        case NORMAL:
             if(Shooter->Shooter_Mode == SHOOTER_RELAX & Shooter->Last_Shooter_Mode != SHOOTER_RELAX)//切入失能模式，全给0
             {
                 Shooter->Fric_Speed_Ref[0] = 0.0f;
                 Shooter->Fric_Speed_Ref[1] = 0.0f;
                 Shooter->Poke_Speed_Ref = 0.0f;
             }
             else if(Shooter->Shooter_Mode == SINGLE_FIRE & Shooter->Last_Shooter_Mode != SINGLE_FIRE)
             {
                 Shooter->Fric_Speed_Ref[0] = 111.0f;//还没调，要改,还会有热量限制
                 Shooter->Fric_Speed_Ref[1] = -111.0f;
                 Shooter->Poke_Speed_Ref =  111.0f;
             }
             else if(Shooter->Shooter_Mode == BURST_FIRE & Shooter->Last_Shooter_Mode != BURST_FIRE)
             {
                 Shooter->Fric_Speed_Ref[0] = 111.0f;//还没调，要改,还会有热量限制
                 Shooter->Fric_Speed_Ref[1] = -111.0f;
                 Shooter->Poke_Speed_Ref = 111.0f;//Bullet_Frequency_Select(receive_judge);//电机参考值与选择的弹频和齿轮比有关，还没修改
             }                                    //牛魔联盟赛没有等级
             else if(Shooter->Shooter_Mode == STOP_FIRE & Shooter->Last_Shooter_Mode != STOP_FIRE)
             {
                 Shooter->Fric_Speed_Ref[0] = 111.0f;//还没调，要改,还会有热量限制
                 Shooter->Fric_Speed_Ref[1] = -111.0f;
                 Shooter->Poke_Angle_Ref = Shooter->Poke_Angle_Fdb ;
             }
             break;
        case JAMMED://JAMMED状态只会进一次，下一次就会进到JAMMED_HANDLE模式，所以这里不再用if判断
             Shooter->Poke_Angle_Ref = Shooter->Poke_Angle_Fdb - 100.0f;//后续再改
             break;
        default :
             break;
    }
    Shooter->Last_Shooter_Mode = Shooter->Shooter_Mode ;//保存此次状态作为上一次状态
}





/**
  * @brief  发射机构控制循环
  * @param  Shooter_t     发射机构
  * @retval 无返回值
  * @note   计算最终发送给各电机的数
  */
void Shooter_Control_Loop(Shooter_t* Shooter)
{
    switch (Shooter->Shooter_State)
    {
        case NORMAL:
            if(Shooter->Shooter_Mode == RELAX)
            {
                Shooter->Poke_Motor_Set_Current = 0;
                Shooter->Fric_Motor_Ser_Current[0] = 0;
                Shooter->Fric_Motor_Ser_Current[1] = 0;
            }
            else if(Shooter->Shooter_Mode == SINGLE_FIRE)
            {
                Shooter->Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter->Fric_Speed_PID[0],Shooter->Fric_Speed_Fdb[0],Shooter->Fric_Speed_Ref[0]);
                Shooter->Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter->Fric_Speed_PID[1],Shooter->Fric_Speed_Fdb[1],Shooter->Fric_Speed_Ref[1]);
                Shooter->Poke_Motor_Set_Current = PID_Calc(&Shooter->Poke_Speed_PID,Shooter->Poke_Speed_Fdb,Shooter->Poke_Speed_Ref);
            }
            else if(Shooter->Shooter_Mode == BURST_FIRE)
            {
                Shooter->Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter->Fric_Speed_PID[0],Shooter->Fric_Speed_Fdb[0],Shooter->Fric_Speed_Ref[0]);
                Shooter->Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter->Fric_Speed_PID[1],Shooter->Fric_Speed_Fdb[1],Shooter->Fric_Speed_Ref[1]);
                Shooter->Poke_Motor_Set_Current = PID_Calc(&Shooter->Poke_Speed_PID,Shooter->Poke_Speed_Fdb,Shooter->Poke_Speed_Ref);
            }
            else if(Shooter->Shooter_Mode == STOP_FIRE)
            {
                Shooter->Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter->Fric_Speed_PID[0],Shooter->Fric_Speed_Fdb[0],Shooter->Fric_Speed_Ref[0]);
                Shooter->Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter->Fric_Speed_PID[1],Shooter->Fric_Speed_Fdb[1],Shooter->Fric_Speed_Ref[1]);
                Shooter->Poke_Speed_Ref = PID_Calc(&Shooter->Poke_Angle_PID,Shooter->Poke_Angle_Fdb,Shooter->Poke_Angle_Ref);
                Shooter->Poke_Motor_Set_Current = PID_Calc(&Shooter->Poke_Speed_PID,Shooter->Poke_Speed_Fdb,Shooter->Poke_Speed_Ref);
            }
            break;
        case JAMMED://JAMMED只有一瞬间
        case JAMMED_HANDLE:
            Shooter->Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter->Fric_Speed_PID[0],Shooter->Fric_Speed_Fdb[0],Shooter->Fric_Speed_Ref[0]);
            Shooter->Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter->Fric_Speed_PID[1],Shooter->Fric_Speed_Fdb[1],Shooter->Fric_Speed_Ref[1]);
            Shooter->Poke_Speed_Ref = PID_Calc(&Shooter->Poke_Angle_PID,Shooter->Poke_Angle_Fdb,Shooter->Poke_Angle_Ref);
            Shooter->Poke_Motor_Set_Current = PID_Calc(&Shooter->Poke_Speed_PID,Shooter->Poke_Speed_Fdb,Shooter->Poke_Speed_Ref);
            break;
        default:
            break;
    }
}






/**
  * @brief  发射机构任务
  * @param  Shooter_t        发射机构
  * @param  Remote_DT7_t     控结构体
  * @param  Control_Mode_e   控制模式
  * @param  receive_judge_t  裁判系统
  * @retval 无返回值
  * @note   无
  */
void Shooter_Task(Shooter_t* Shooter,Control_Mode_e* Control_Mode,Remote_DT7_t* Remote,receive_judge_t* receive_judge)
{
    Shooter_Mode_Select(Shooter,Control_Mode,Remote);//发射机构模式选择
    Shooter_Feedback_Update(Shooter);//发射机构反馈更新
    Shooter_State_Update(Shooter);//发射机构状态更新
    Heat_Restrict(Shooter,receive_judge);//热量限制
    Shooter_Reference_Update(Shooter,receive_judge);//发射机构参考值更新
    Shooter_Control_Loop(Shooter);//最终发送给电机
}







