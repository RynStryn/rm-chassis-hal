#include "inc/Control_Task.h"
#include "inc/Can_Motor.h"
#include "inc/Chassis_Task.h"
#include "inc/Remote_Control.h"
#include "led.h"



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
    for (int i = 0; i < 4; i++)
        Chassis_Wheel_Fdb_Set(
            &Chassis.wheel[i],
            Wheel_angVel_to_linVel(Motor_Encoder[i].angVel_dps, Chassis.wheel[i].linVel));
    Remote_Ctrl_Chassis();
    if (g_time % 2000 == 1000)
        HARD_WDG_ON;
    if (g_time % 2000 == 0)
        HARD_WDG_OFF;
    if (g_time % 10000 == 0)
        g_time = 0;
}
