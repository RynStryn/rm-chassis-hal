#include "main.h"

void TIM2_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Control_Task(&Gimbal, &Chassis, &Remote_DT7_data, &Shooter, &Control_Mode, &judge_rece_mesg);
        CAN_Send_Task(Gimbal.Gimbal_Motor_Set_Current, Chassis.Chassis_Motor_Set_Current);
    }
}
