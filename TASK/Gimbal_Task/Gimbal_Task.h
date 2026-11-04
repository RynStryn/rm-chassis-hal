#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "USART1.h"
#include "USART3.h"
#include "PID.h"
#include "CAN_Send_Task.h"
#include "Remote_Task.h"

/*********************************宏定义************************************/
#define      MOUSE_SENSITIVITY        0.0001       //鼠标灵敏度，转换后角速度单位deg/s
#define      REMOTE_SENSITIVITY       0.0004       //控制器灵敏度，转换后角速度单位deg/s
/*********************************宏定义************************************/



/*********************************云台结构体********************************/
typedef enum
{
    GIMBAL_RELAX,           //云台失能
    GIMBAL_FALLWO_GYRO,     //云台跟随陀螺仪
}Gimbal_Mode_e;


typedef struct
{
    float Yaw_Angle_Ref;//角度参考值
	float Yaw_Angle_Fdb;//角度反馈值
	float Yaw_Angle_To_Chassis;//相对底盘的yaw轴角度，范围:-180~180
    float Yaw_Speed_Ref;//速度参考值
 	float Yaw_Speed_Fdb;//速度反馈值
    
    float Pitch_Angle_Ref;//角度参考值
	float Pitch_Angle_Fdb;//角度反馈值
    float Pitch_Speed_Ref;//速度参考值
	float Pitch_Speed_Fdb;//速度反馈值
    
    Gimbal_Mode_e	Gimbal_Mode;//云台模式结构体
	Gimbal_Mode_e	Gimbal_Mode_Last;//云台模式结构体
    CH040DATA_t     CH040_Data;//陀螺仪数据结构体
    Encoder        Yaw_Motor_Encoder;//Yaw电机编码器
    Encoder        Pitch_Motor_Encoder;//Pitch电机结构体
    PID_t          Gimbal_Motor_Speed_PID[2];//云台电机PID，0 Yaw , 1 Pitch 
    PID_t          Gimbal_Motor_Angle_PID[2];//云台电机PID，0 Yaw , 1 Pitch 
    int16_t        Gimbal_Motor_Set_Current[2];//云台电机电流给定值，0 Yaw , 1 Pitch
}Gimbal_t;
/*********************************云台结构体**********************************/




/*********************************外部声明************************************/
extern Gimbal_t Gimbal;
/*********************************外部声明************************************/




/*********************************函数声明************************************/
void Gimbal_Mode_Select(Gimbal_t* Gimbal,Control_Mode_e* Control_Mode);
void Gimbal_Feedback_Update(Gimbal_t* Gimbal);
void Gimbal_Reference_Update(Gimbal_t* Gimbal,Remote_DT7_t* Remote_DT7_Data,Control_Mode_e* Control_Mode);
void Gimbal_Control_Loop(Gimbal_t* Gimbal,Control_Mode_e* Control_Mode);
void Gimbal_Task(Gimbal_t* Gimbal,Control_Mode_e* Control_Mode,Remote_DT7_t* Remote_DT7_Data);
/*********************************函数声明************************************/


#endif
