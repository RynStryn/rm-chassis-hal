#include "../inc/Control_Task.h"

#include "../inc/Can_Motor.h"
#include "../inc/Chassis_Task.h"
#include "../inc/Fault_Diagnosis_Task.h"
#include "../inc/RC_USART1.h"
#include "CH040_USART3.h"
#include "Remote_Control.h"

uint32_t g_time               = 0;
int      classis_offline_cnt  = 0;
int      chassis_offline_flag = 0;
void     wheel_offline_detect(void)
{
    if (wheel_online[0] == 0
        // || wheel_online[1] == 0
        // || wheel_online[2] == 0 || wheel_online[3] == 0
    )
        classis_offline_cnt++;
    else
        classis_offline_cnt = 0;

    if (classis_offline_cnt > 4000)
        chassis_offline_flag = 1;
    else
        chassis_offline_flag = 0;
}
void Control_Task(void)
{
    g_time++;
    CH040_Data_Get(&imu_data, &CH040DATA);
    wheel_offline_detect();

    Peripheral_State_Judge(&Peripheral_State, 0.001f);


    RC_Cmd_Interpreter();
    if (chassis_offline_flag == 1)
    {
        g_Chassis.mode = CHASSIS_RELAX;
    }
    // g_Chassis.mode=CHASSIS_INDEPENDENT;
    //   gimbal_task();
    chassis_task(&g_Chassis, &g_wheel, &g_gyro);
}
