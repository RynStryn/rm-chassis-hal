#include "Remote_Control.h"
#include "Chassis_Task.h"
#include "RC_USART1.h"
#include "FS_i6.h"
#include "main.h"

RC_DT7_Data_t g_RC_data = {0};
RC_Mode_e     RC_Mode;
float         yaw_ang_inc = 0;

/**
 *@Brief： DT7遥控器数据处理函数
 *@Cal：   内部或外部
 *@param:  遥控的结构体
 *@Note:   放在对应串口空闲中断中确保信息的实时性
 *@RetVal: 无
 */

void DT7_RC_Data_Dispose(uint8_t* raw_Data, RC_DT7_Data_t* RC_data)
{
    if (raw_Data == NULL)
    {
        return;
    }
    RC_data->stick.ch0
        = ((int16_t) raw_Data[0] | ((int16_t) raw_Data[1] << 8)) & 0x07FF;
    RC_data->stick.ch1
        = (((int16_t) raw_Data[1] >> 3) | ((int16_t) raw_Data[2] << 5))
        & 0x07FF;
    RC_data->stick.ch2
        = (((int16_t) raw_Data[2] >> 6) | ((int16_t) raw_Data[3] << 2)
              | ((int16_t) raw_Data[4] << 10))
        & 0x07FF;
    RC_data->stick.ch3
        = (((int16_t) raw_Data[4] >> 1) | ((int16_t) raw_Data[5] << 7))
        & 0x07FF;
    RC_data->toggle_SW.ch4
        = ((int16_t) raw_Data[16] >> 0 | ((int16_t) raw_Data[17] << 8))
        & 0x07FF;
    RC_data->toggle_SW.s1 = ((raw_Data[5] >> 4) & 0x000C) >> 2;
    RC_data->toggle_SW.s2 = ((raw_Data[5] >> 4) & 0x0003);  // 模式切换

    RC_data->Remote_mouse.x
        = ((int16_t) raw_Data[6]) | ((int16_t) raw_Data[7] << 8);
    RC_data->Remote_mouse.y
        = ((int16_t) raw_Data[8]) | ((int16_t) raw_Data[9] << 8);
    RC_data->Remote_mouse.z
        = ((int16_t) raw_Data[10]) | ((int16_t) raw_Data[11] << 8);
    RC_data->Remote_mouse.Press_L_Action.Raw_Short_Press = raw_Data[12];
    RC_data->Remote_mouse.Press_R_Action.Raw_Short_Press = raw_Data[13];

    RC_data->key.Key_W_Action.Raw_Short_Press     = (raw_Data[14]) & 0x01;
    RC_data->key.Key_S_Action.Raw_Short_Press     = (raw_Data[14] >> 1) & 0x01;
    RC_data->key.Key_A_Action.Raw_Short_Press     = (raw_Data[14] >> 2) & 0x01;
    RC_data->key.Key_D_Action.Raw_Short_Press     = (raw_Data[14] >> 3) & 0x01;
    RC_data->key.Key_SHIFT_Action.Raw_Short_Press = (raw_Data[14] >> 4) & 0x01;
    RC_data->key.Key_CTRL_Action.Raw_Short_Press  = (raw_Data[14] >> 5) & 0x01;
    RC_data->key.Key_Q_Action.Raw_Short_Press     = (raw_Data[14] >> 6) & 0x01;
    RC_data->key.Key_E_Action.Raw_Short_Press     = (raw_Data[14] >> 7) & 0x01;

    RC_data->key.Key_R_Action.Raw_Short_Press
        = (raw_Data[15]) & 0x01;  // 手册疏于维护，实际以代码为准
    RC_data->key.Key_F_Action.Raw_Short_Press = (raw_Data[15] >> 1) & 0x01;
    RC_data->key.Key_G_Action.Raw_Short_Press = (raw_Data[15] >> 2) & 0x01;
    RC_data->key.Key_Z_Action.Raw_Short_Press = (raw_Data[15] >> 3) & 0x01;
    RC_data->key.Key_X_Action.Raw_Short_Press = (raw_Data[15] >> 4) & 0x01;
    RC_data->key.Key_C_Action.Raw_Short_Press = (raw_Data[15] >> 5) & 0x01;
    RC_data->key.Key_V_Action.Raw_Short_Press = (raw_Data[15] >> 6) & 0x01;
    RC_data->key.Key_B_Action.Raw_Short_Press = (raw_Data[15] >> 7) & 0x01;

    RC_data->stick.ch0 -= RC_CH_VALUE_OFFSET;  // 减去偏移，归化到-660——660
    RC_data->stick.ch1 -= RC_CH_VALUE_OFFSET;
    RC_data->stick.ch2 -= RC_CH_VALUE_OFFSET;
    RC_data->stick.ch3 -= RC_CH_VALUE_OFFSET;
    RC_data->toggle_SW.ch4 -= RC_CH_VALUE_OFFSET;
}

/**
 * @brief  按键状态检测函数，用于判断按键状态
 * @param  Key_Mouse_Action_t 按键结构体
 * @retval 无返回值
 * @note   无
 */
void Key_Mouse_Action_Detect(Press_Action_t* Key_Mouse_Action)
{
    uint8_t diff;  // 检测上升沿
    diff = Key_Mouse_Action->Raw_Short_Press
        - Key_Mouse_Action->last_Raw_Short_Press;
    Key_Mouse_Action->last_Raw_Short_Press = Key_Mouse_Action->Raw_Short_Press;
    if (Key_Mouse_Action->Raw_Short_Press == 1)  // 当按下时，开始计数
    {
        Key_Mouse_Action->tick++;
        // Key_Mouse_Action->Short_Press_Flag =
        // 1;//这里短按逻辑有问题，应该达成按下一次只置一次1的效果，这里一直置1，会打出很多发弹
    }
    else
    {
        Key_Mouse_Action->tick       = 0;  // 当松手时，清零长按计数和长按标志位
        Key_Mouse_Action->Long_Press = 0;
    }

    if (Key_Mouse_Action->tick >= LONG_PRESS_THRESHOLD)
    {
        Key_Mouse_Action->tick        = LONG_PRESS_THRESHOLD;
        Key_Mouse_Action->Long_Press  = 1;  // 长按标志位置1时，短按标志位置0
        Key_Mouse_Action->Short_Press = 0;
    }

    if (diff == 1)  // 当按下按键，检测到一次上升沿时，翻转一次标志位
    {
        Key_Mouse_Action->Short_Press = 1;

        if (Key_Mouse_Action->Toggle_Press == 0)
            Key_Mouse_Action->Toggle_Press = 1;
        else
            Key_Mouse_Action->Toggle_Press = 0;
    }
}

/**
 * @brief  更新所有按键的长短按状态
 * @param  Remote_DT7_t 控结构体
 * @retval 无返回值
 * @note   无
 */
void Key_Mouse_State_Update(RC_DT7_Data_t* Remote_data)
{
    Key_Mouse_Action_Detect(&Remote_data->Remote_mouse
            .Press_L_Action);  // 每个按键都调用一次检测函数
    Key_Mouse_Action_Detect(&Remote_data->Remote_mouse.Press_R_Action);

    Key_Mouse_Action_Detect(&Remote_data->key.Key_W_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_S_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_A_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_D_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_SHIFT_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_CTRL_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_Q_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_E_Action);

    Key_Mouse_Action_Detect(&Remote_data->key.Key_R_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_F_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_G_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_Z_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_X_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_C_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_V_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_B_Action);
}

/**
 * @brief  选择控制模式
 * @param  Remote_DT7_t   控结构体
 * @param  Control_Mode_e 控制模式结构体
 * @retval 无返回值
 * @note   无
 */
void RC_Mode_Select( RC_Mode_e* RC_Mode)
{
    //switch (RC_data->toggle_SW.s1)
    switch (Remote_Data.swa)
    {
        case DOWN:
            *RC_Mode = CONTORLLER;//_
            break;
        case MIDDLE:
            *RC_Mode = CONTORLLER;
            break;
        case UP:
            *RC_Mode = OFFLINE;
            break;
        default:
            *RC_Mode = OFFLINE;
            break;
    }
}

/**
 * @brief  拨杆拨轮动作检测(Toggle ,Slider
 * @param  Remote_DT7_t   控结构体
 * @retval 无返回值
 * @note   无
 */
int8_t s2_Diff;
void   RC_Toggle_Action_Detect(RC_DT7_Data_t* RC_data)
{
    s2_Diff = RC_data->toggle_SW.s2 - RC_data->toggle_SW.s2_last;
    switch (s2_Diff)
    {
        case 0:  // 根据手册，拨杆在中间为3，在上是1，在下是2，作差可判断拨杆动作
            RC_data->toggle_SW.s2_Action = KEEP;
            break;
        case 1:
            RC_data->toggle_SW.s2_Action = DOWN_TO_MIDDLE;
            break;
        case -2:
            RC_data->toggle_SW.s2_Action = MIDDLE_TO_UP;
            break;
        case 2:
            RC_data->toggle_SW.s2_Action = UP_TO_MIDDLE;
            break;
        case -1:
            RC_data->toggle_SW.s2_Action = MIDDLE_TO_DOWN;
            break;
        default:
            break;
    }

    // 拨轮向上或向下拨一次，再回正，视为一次动作，执行按键状态翻转
    if (RC_data->toggle_SW.ch4 == 660)
        RC_data->toggle_SW.Trigger_Up = 1;
    else if (RC_data->toggle_SW.ch4 == -660)
        RC_data->toggle_SW.Trigger_Down = 1;

    if (RC_data->toggle_SW.ch4 == 0 & RC_data->toggle_SW.Trigger_Up == 1)
    {
        RC_data->toggle_SW.Trigger_Up = 0;
        if (RC_data->toggle_SW.Trigger_Up_Action.Toggle_Press == 1)
            RC_data->toggle_SW.Trigger_Up_Action.Toggle_Press = 0;
        else
            RC_data->toggle_SW.Trigger_Up_Action.Toggle_Press = 1;
    }

    if (RC_data->toggle_SW.ch4 == 0 & RC_data->toggle_SW.Trigger_Down == 1)
    {
        RC_data->toggle_SW.Trigger_Down = 0;
        if (RC_data->toggle_SW.Trigger_Down_Action.Toggle_Press == 1)
            RC_data->toggle_SW.Trigger_Down_Action.Toggle_Press = 0;
        else
            RC_data->toggle_SW.Trigger_Down_Action.Toggle_Press = 1;
    }

    // 向上或向下一直拨不回正，视为长按
    if (RC_data->toggle_SW.ch4 == 660)
    {
        RC_data->toggle_SW.Trigger_Up_Action.Short_Press = 1;
        RC_data->toggle_SW.Trigger_Up_Action.tick++;
    }
    else if (RC_data->toggle_SW.ch4 == -660)
    {
        RC_data->toggle_SW.Trigger_Down_Action.Short_Press = 1;
        RC_data->toggle_SW.Trigger_Down_Action.tick++;
    }

    if (RC_data->toggle_SW.Trigger_Up_Action.tick >= LONG_PRESS_THRESHOLD)
    {
        RC_data->toggle_SW.Trigger_Up_Action.tick
            = LONG_PRESS_THRESHOLD;  // 长按时置长按标志位，清零短按，翻转标志位
        RC_data->toggle_SW.Trigger_Up_Action.Long_Press   = 1;
        RC_data->toggle_SW.Trigger_Up                     = 0;
        RC_data->toggle_SW.Trigger_Up_Action.Toggle_Press = 0;
        RC_data->toggle_SW.Trigger_Up_Action.Short_Press  = 0;
    }
    else if (RC_data->toggle_SW.Trigger_Down_Action.tick
        >= LONG_PRESS_THRESHOLD)
    {
        RC_data->toggle_SW.Trigger_Down_Action.tick = LONG_PRESS_THRESHOLD;
        RC_data->toggle_SW.Trigger_Down_Action.Long_Press   = 1;
        RC_data->toggle_SW.Trigger_Down                     = 0;
        RC_data->toggle_SW.Trigger_Down_Action.Toggle_Press = 0;
        RC_data->toggle_SW.Trigger_Down_Action.Short_Press  = 0;
    }

    if (RC_data->toggle_SW.ch4 == 0)
    {
        RC_data->toggle_SW.Trigger_Up_Action.tick         = 0;
        RC_data->toggle_SW.Trigger_Up_Action.Long_Press   = 0;
        RC_data->toggle_SW.Trigger_Down_Action.tick       = 0;
        RC_data->toggle_SW.Trigger_Down_Action.Long_Press = 0;
    }
    RC_data->toggle_SW.s2_last
        = RC_data->toggle_SW.s2;  // 保存本次数据作为下一次的上一次
}
static inline int16_t dead_zone(int16_t val, uint16_t zone)
{
    if (val > -zone && val < zone)
    {
        return 0;
    }
    return val;
}


void RC_Cmd_Interpreter(void)
{
    g_RC_data.stick.ch0   = Remote_Data.ch0;
    g_RC_data.stick.ch1   = Remote_Data.ch1;
    g_RC_data.stick.ch2   = Remote_Data.ch3;//_
    g_RC_data.stick.ch3   = Remote_Data.ch2;//_
    g_RC_data.toggle_SW.s1 = Remote_Data.swa;
    g_RC_data.toggle_SW.s2 = Remote_Data.swc;
    RC_Mode_Select(&RC_Mode);
    switch (RC_Mode)
    {
        case OFFLINE:
            g_Chassis.mode = CHASSIS_RELAX;
            break;
        case CONTORLLER:
        {
            int16_t ch1       = dead_zone(g_RC_data.stick.ch1, RC_DEAD_ZONE);
            int16_t ch0       = dead_zone(g_RC_data.stick.ch0, RC_DEAD_ZONE);
            int16_t ch2       = dead_zone(g_RC_data.stick.ch2, RC_DEAD_ZONE);
            int16_t ch3       = dead_zone(g_RC_data.stick.ch3, RC_DEAD_ZONE);
            g_Chassis.V_X_ref = (float) (ch2-1024) * CHASSIS_REMOTE_SENSITIVITY;
            g_Chassis.V_Y_ref = (float) (ch3-1024) * CHASSIS_REMOTE_SENSITIVITY;
            yaw_ang_inc      = (float) (ch0-1024) * CHASSIS_REMOTE_SENSITIVITY;
            //switch (g_RC_data.toggle_SW.s2)
            switch (Remote_Data.swc)
            {
                case UP://_
                    g_Chassis.mode = CHASSIS_RELAX;
                    break;
                case MIDDLE:
                    g_Chassis.mode = CHASSIS_INDEPENDENT;
                    break;
                case DOWN://_
                    g_Chassis.mode = CHASSIS_ROTATE;
                    break;
            }

            break;
        }
        case KEY_MOUSE:
        {
            g_Chassis.V_Y_ref = (float) g_RC_data.Remote_mouse.x;
            // TODO
            break;
        }
        default:
            break;
    }
    LimitMax(g_Chassis.V_X_ref, CHASSIS_VX_LIMIT);
    LimitMax(g_Chassis.V_Y_ref, CHASSIS_VY_LIMIT);
    LimitMax(yaw_ang_inc, CHASSIS_W_LIMIT);
}

