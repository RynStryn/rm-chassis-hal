#include "inc/Control_Task.h"
#include "inc/Can_Motor.h"
#include "inc/Chassis_Task.h"
#include "inc/Remote_Control.h"
#include "led.h"

extern CAN_HandleTypeDef hcan2;

int  g_time = 0;
int  classis_offline_cnt = 0;
int  classis_offline_flag = 0;
void wheel_offline_detect (void) {
    if (wheelIsOK[0] == 0 || wheelIsOK[1] == 0
        || wheelIsOK[2] == 0 || wheelIsOK[3] == 0)
        classis_offline_cnt++;
    else
        classis_offline_cnt = 0;

    if (classis_offline_cnt > 4000)
        classis_offline_flag = 1;
    else
        classis_offline_flag = 0;
}
void Control_Task(void)
{
    g_time++;
    wheel_offline_detect();
    Chassis_Wheel_Fdb_Set(
        &Chassis,
        Chassis_Encoder_2_Real(&(Motor_c610c620_Encoder[0].Speed_dps)),
        Chassis_Encoder_2_Real(&(Motor_c610c620_Encoder[1].Speed_dps)),
        Chassis_Encoder_2_Real(&(Motor_c610c620_Encoder[2].Speed_dps)),
        Chassis_Encoder_2_Real(&(Motor_c610c620_Encoder[3].Speed_dps)));
    Remote_Ctrl_Chassis();
    if (g_time % 2000 == 1000)
        HARD_WDG_ON;
    if (g_time % 2000 == 0)
        HARD_WDG_OFF;
    if (g_time % 10000 == 0)
        g_time = 0;
}

void CAN_Send_Task(void)
{
    static int CAN_SEND_CNT;
    CAN_SEND_CNT++;

    if (CAN_SEND_CNT % 4 == 1)
    {
        if ((wheelIsOK[0] == 1 && wheelIsOK[1] == 1
            && wheelIsOK[2] == 1 && wheelIsOK[3] == 1)
            || classis_offline_flag == 1)
        {
            Motor_c610andc620_out1(
                &hcan2,
                Chassis.Pid_Speed[0].out,
                Chassis.Pid_Speed[1].out,
                Chassis.Pid_Speed[2].out,
                Chassis.Pid_Speed[3].out);
        }
    }

    if (CAN_SEND_CNT % 10000 == 0)
        CAN_SEND_CNT = 0;
}
