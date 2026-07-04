#include "inc/Remote_Control.h"

#include "inc/config.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

Remote         Remote_data;
Control_Mode_e RemoteCtrl_Mode = NUll;
uint8_t        usart1_rx_buf[DBUS_RX_BUF_SIZE];

static Control_Mode_e RemoteCtrl_Mode_Last = NUll;

static void Key_Single_Click_Cooling(Key_Mouse_Action_t* pAction);
static void Key_Single_Click_CD(Key_Mouse_Action_t* pAction);
static void Single_Key_Mouse_Action_Detection(Key_Mouse_Action_t* pAction, int State);
static void RmtCtrl_Input_Interpreter(void);
static void Chassis_Mode_Task(void);

void Remote_Init(void)
{
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_UART_Receive_DMA(&huart1, usart1_rx_buf, DBUS_RX_BUF_SIZE);
}

void Remote_Ctrl_Chassis(void)
{
    Remote_Switch_Action_Get(&Remote_data.Remote_clicker);
    Key_Mouse_Action_Get(&Remote_data.Remote_mouse, &Remote_data.key);

    if (Remote_data.Remote_clicker.s2 == 1)
        RemoteCtrl_Mode = REMOTE;
    else if (Remote_data.Remote_clicker.s2 == 3)
        RemoteCtrl_Mode = KEY_MOUSE;
    else
        RemoteCtrl_Mode = NUll;

    RmtCtrl_Input_Interpreter();
    Chassis_Mode_Task();

    RemoteCtrl_Mode_Last = RemoteCtrl_Mode;
}

static void Key_Single_Click_Cooling(Key_Mouse_Action_t* pAction)
{
    if (!pAction->Flag)
    {
        if (pAction->Cnt < KEY_LOW_LIMIT)
            pAction->Cnt++;
        if (pAction->Cnt == KEY_LOW_LIMIT)
            pAction->Flag = 1;
    }
}

static void Key_Single_Click_CD(Key_Mouse_Action_t* pAction)
{
    pAction->Flag = 0;
    pAction->Cnt  = 0;
}

static void Single_Key_Mouse_Action_Detection(
    Key_Mouse_Action_t* pAction,
    int                 State)
{
    if (State)
    {
        Key_Single_Click_Cooling(pAction);
    }
    else
    {
        Key_Single_Click_CD(pAction);
    }

    if (pAction->Flag)
    {
        pAction->Key_Mouse_Action = TAP_AND_HOLD;
        pAction->Hold_Cnt++;
    }
    else
    {
        if (pAction->Flag_Last)
        {
            if (pAction->Hold_Cnt > KEY_SHORT_LONG_BOUNDARY)
                pAction->Key_Mouse_Action = CLICK_LONG_TIME;
            else
                pAction->Key_Mouse_Action = CLICK_FAST;

            pAction->Hold_Cnt = 0;
        }
        else
        {
            pAction->Key_Mouse_Action = ZERO_INPUT;
        }
    }

    pAction->Flag_Last = pAction->Flag;
}

void Key_Mouse_Action_Get(Mouse_t* pMouse, Key_t* pKey)
{
    Single_Key_Mouse_Action_Detection(&pMouse->Press_L_Action, pMouse->press_l);
    Single_Key_Mouse_Action_Detection(&pMouse->Press_R_Action, pMouse->press_r);

    Single_Key_Mouse_Action_Detection(&pKey->Key_W_Action, pKey->v & KEY_PRESSED_OFFSET_W);
    Single_Key_Mouse_Action_Detection(&pKey->Key_S_Action, pKey->v & KEY_PRESSED_OFFSET_S);
    Single_Key_Mouse_Action_Detection(&pKey->Key_A_Action, pKey->v & KEY_PRESSED_OFFSET_A);
    Single_Key_Mouse_Action_Detection(&pKey->Key_D_Action, pKey->v & KEY_PRESSED_OFFSET_D);
    Single_Key_Mouse_Action_Detection(&pKey->Key_Q_Action, pKey->v & KEY_PRESSED_OFFSET_Q);
    Single_Key_Mouse_Action_Detection(&pKey->Key_E_Action, pKey->v & KEY_PRESSED_OFFSET_E);
    Single_Key_Mouse_Action_Detection(&pKey->Key_R_Action, pKey->v & KEY_PRESSED_OFFSET_R);
    Single_Key_Mouse_Action_Detection(&pKey->Key_F_Action, pKey->v & KEY_PRESSED_OFFSET_F);
    Single_Key_Mouse_Action_Detection(&pKey->Key_G_Action, pKey->v & KEY_PRESSED_OFFSET_G);
    Single_Key_Mouse_Action_Detection(&pKey->Key_Z_Action, pKey->v & KEY_PRESSED_OFFSET_Z);
    Single_Key_Mouse_Action_Detection(&pKey->Key_X_Action, pKey->v & KEY_PRESSED_OFFSET_X);
    Single_Key_Mouse_Action_Detection(&pKey->Key_C_Action, pKey->v & KEY_PRESSED_OFFSET_C);
    Single_Key_Mouse_Action_Detection(&pKey->Key_V_Action, pKey->v & KEY_PRESSED_OFFSET_V);
    Single_Key_Mouse_Action_Detection(&pKey->Key_B_Action, pKey->v & KEY_PRESSED_OFFSET_B);
    Single_Key_Mouse_Action_Detection(&pKey->Key_SHIFT_Action, pKey->v & KEY_PRESSED_OFFSET_SHIFT);
    Single_Key_Mouse_Action_Detection(&pKey->Key_CTRL_Action, pKey->v & KEY_PRESSED_OFFSET_CTRL);
}

void RemoteDataDispose(uint8_t* pData)
{
    if (pData == NULL)
        return;

    Remote_data.Remote_clicker.ch0
        = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    Remote_data.Remote_clicker.ch1
        = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    Remote_data.Remote_clicker.ch2
        = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2)
           | ((int16_t)pData[4] << 10)) & 0x07FF;
    Remote_data.Remote_clicker.ch3
        = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;
    Remote_data.Remote_clicker.ch4
        = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
    Remote_data.Remote_clicker.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    Remote_data.Remote_clicker.s2 = ((pData[5] >> 4) & 0x0003);

    Remote_data.Remote_mouse.x
        = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    Remote_data.Remote_mouse.y
        = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    Remote_data.Remote_mouse.z
        = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    Remote_data.Remote_mouse.press_l = pData[12];
    Remote_data.Remote_mouse.press_r = pData[13];
    Remote_data.key.v
        = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
}

void Remote_Switch_Action_Get(Clicker_t* pRemote)
{
    static int Diff_s1      = 0;
    static int Flag_Trigger = 0;

    Diff_s1 = pRemote->s1 - pRemote->s1_last;

    if (Diff_s1 == 0)
    {
        pRemote->s1_Action = Keep;
    }
    else if (Diff_s1 == 1)
    {
        pRemote->s1_Action = L_2_M;
    }
    else if (Diff_s1 == -2)
    {
        pRemote->s1_Action = M_2_H;
    }
    else if (Diff_s1 == 2)
    {
        pRemote->s1_Action = H_2_M;
    }
    else if (Diff_s1 == -1)
    {
        pRemote->s1_Action = M_2_L;
    }

    if (abs(pRemote->ch4 - 800) < 50)
    {
        Flag_Trigger = 1;
    }
    else if (pRemote->ch4 == 1684)
    {
        Flag_Trigger = -1;
    }
    if ((pRemote->ch4 - 1024) == 0)
    {
        if (Flag_Trigger == 1)
            pRemote->Trigger_H = (pRemote->Trigger_H + 1) % 2;
        else if (Flag_Trigger == -1)
            pRemote->Trigger_L = (pRemote->Trigger_L + 1) % 2;
        Flag_Trigger = 0;
    }

    pRemote->s1_last = pRemote->s1;
    pRemote->s2_last = pRemote->s2;
}

static void RmtCtrl_Input_Interpreter(void)
{
    Chassis.Chassis_Mode_Last = Chassis.Chassis_Mode;

    if (RemoteCtrl_Mode == NUll)
    {
        Chassis.Chassis_Mode = CHASSIS_RELAX;
        for (int i = 0; i < 4; i++)
        {
            pid_clear(&Chassis.wheel[i].Speed_Pid);
        }
        return;
    }
    if (RemoteCtrl_Mode == REMOTE || RemoteCtrl_Mode == KEY_MOUSE)
    {
        if (Chassis.Chassis_Mode == CHASSIS_RELAX)
        {
            Chassis.Chassis_Mode = CHASSIS_INDEPEND;
        }

        if (Remote_data.Remote_clicker.s1 == 2)
        {
            if (Remote_data.Remote_clicker.s1_Action == M_2_L)
            {
                if (Chassis.Chassis_Mode == CHASSIS_INDEPEND)
                    Chassis.Chassis_Mode = CHASSIS_CW_ROTATE;
                else
                    Chassis.Chassis_Mode = CHASSIS_INDEPEND;
            }
        }
    }
}

static void Chassis_Mode_Task(void)
{
    switch (Chassis.Chassis_Mode)
    {
        case CHASSIS_RELAX:
        {
            Chassis_Wheel_Task(&Chassis, 0, 0, 0);
            for (int i = 0; i < 4; i++)
            {
                pid_clear(&Chassis.wheel[i].Speed_Pid);
            }
        }
        break;

        case CHASSIS_STOP:
        {
            Chassis_Wheel_Task(&Chassis, 0, 0, 0.00001f);
        }
        break;

        case CHASSIS_INDEPEND:
        {
            Chassis_Dirction_Angle_Set(&Chassis, 0 + CHASSIS_DIR_BIAS);

            if (RemoteCtrl_Mode == REMOTE)
            {
                Chassis_Wheel_Task(
                    &Chassis,
                    -(float)(Remote_data.Remote_clicker.ch0 - 1024)
                        * CHASSIS_REMOTE_SENSITIVITY,
                    ((float)Remote_data.Remote_clicker.ch1 - 1024)
                        * CHASSIS_REMOTE_SENSITIVITY,
                    (float)(Remote_data.Remote_clicker.ch2 - 1024) * 0.001f);
            }
            else if (RemoteCtrl_Mode == KEY_MOUSE)
            {
                Chassis_Wheel_Task(
                    &Chassis,
                    -(float)(Remote_data.key.Key_D_Action.Flag
                             - Remote_data.key.Key_A_Action.Flag)
                        * CHASSIS_KW_X_SPEED
                        * (1
                           + Remote_data.key.Key_SHIFT_Action.Flag
                                 * CHASSIS_FAST_K),
                    (float)(Remote_data.key.Key_W_Action.Flag
                            - Remote_data.key.Key_S_Action.Flag)
                        * CHASSIS_KW_Y_SPEED
                        * (1
                           + Remote_data.key.Key_SHIFT_Action.Flag
                                 * CHASSIS_FAST_K),
                    (Remote_data.Remote_mouse.x) * CHASSIS_MOUSE_W_SENSITIVITY);
            }
        }
        break;

        case CHASSIS_CW_ROTATE:
        {
            Chassis_Dirction_Angle_Set(&Chassis, 0 + CHASSIS_DIR_BIAS);

            if (RemoteCtrl_Mode == REMOTE)
            {
                Chassis_Wheel_Task(
                    &Chassis,
                    -(float)(Remote_data.Remote_clicker.ch0 - 1024)
                        * CHASSIS_REMOTE_SENSITIVITY * 0.8f,
                    ((float)Remote_data.Remote_clicker.ch1 - 1024)
                        * CHASSIS_REMOTE_SENSITIVITY * 0.8f,
                    -ROTATE_SPEED_PATROL);
            }
            else if (RemoteCtrl_Mode == KEY_MOUSE)
            {
                Chassis_Wheel_Task(
                    &Chassis,
                    -(float)(Remote_data.key.Key_D_Action.Flag
                             - Remote_data.key.Key_A_Action.Flag)
                        * CHASSIS_KW_X_SPEED * 0.8f,
                    (float)(Remote_data.key.Key_W_Action.Flag
                            - Remote_data.key.Key_S_Action.Flag)
                        * CHASSIS_KW_Y_SPEED * 0.8f,
                    -ROTATE_SPEED_PATROL);
            }
        }
        break;

        case CHASSIS_CCW_ROTATE:
        {
            Chassis_Dirction_Angle_Set(&Chassis, 0 + CHASSIS_DIR_BIAS);

            if (RemoteCtrl_Mode == REMOTE)
            {
                Chassis_Wheel_Task(
                    &Chassis,
                    -(float)(Remote_data.Remote_clicker.ch0 - 1024)
                        * CHASSIS_REMOTE_SENSITIVITY * 0.8f,
                    ((float)Remote_data.Remote_clicker.ch1 - 1024)
                        * CHASSIS_REMOTE_SENSITIVITY * 0.8f,
                    ROTATE_SPEED_PATROL);
            }
            else if (RemoteCtrl_Mode == KEY_MOUSE)
            {
                Chassis_Wheel_Task(
                    &Chassis,
                    -(float)(Remote_data.key.Key_D_Action.Flag
                             - Remote_data.key.Key_A_Action.Flag)
                        * CHASSIS_KW_X_SPEED * 0.8f,
                    (float)(Remote_data.key.Key_W_Action.Flag
                            - Remote_data.key.Key_S_Action.Flag)
                        * CHASSIS_KW_Y_SPEED * 0.8f,
                    ROTATE_SPEED_PATROL);
            }
        }
        break;

        default:
        {
            Chassis_Wheel_Task(&Chassis, 0, 0, 0);
        }
        break;
    }
}
