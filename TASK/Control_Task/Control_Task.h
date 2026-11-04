#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Remote_Task.h"
#include "Shooter_Task.h"
#include "CAN_Send_Task.h"




void Control_Task(Gimbal_t* Gimbal,Chassis_t* Chassis,Remote_DT7_t* Remote_DT7_Data,Shooter_t* Shooter,Control_Mode_e* Control_Mode,receive_judge_t* receive_judge);



#endif
