#include "main.h"

void Infantry_Init(void)
{
    for(uint8_t i=0;i<4;i++) 
    PID_Init(&Chassis.Chassis_Motor_Speed_PID[i]
    ,PID_POSITION,M2006_MOTOR_SPEED_PID_KP,M2006_MOTOR_SPEED_PID_KI,M2006_MOTOR_SPEED_PID_KD
    ,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT);//底盘电机速度pid
    
    PID_Init(&Chassis.Chassis_Angle_PID
    ,PID_POSITION,CHASSIS_FOLLOW_GIMBAL_PID_KP,CHASSIS_FOLLOW_GIMBAL_PID_KI,CHASSIS_FOLLOW_GIMBAL_PID_KD
    ,CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);//底盘跟随云台角度pid
    
    PID_Init(&Gimbal.Gimbal_Motor_Speed_PID[0]
    ,PID_POSITION,YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD
    ,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT);//云台yaw轴速度pid
    
    PID_Init(&Gimbal.Gimbal_Motor_Speed_PID[1]
    ,PID_POSITION,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD
    ,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);//云台pitch轴速度pid
    
    PID_Init(&Gimbal.Gimbal_Motor_Angle_PID[0]
    ,PID_POSITION,YAW_GYRO_ANGLE_PID_KP,YAW_GYRO_ANGLE_PID_KI,YAW_GYRO_ANGLE_PID_KD
    ,YAW_GYRO_ANGLE_PID_MAX_OUT,YAW_GYRO_ANGLE_PID_MAX_IOUT);//云台yaw轴角度pid
    
    PID_Init(&Gimbal.Gimbal_Motor_Angle_PID[1]
    ,PID_POSITION,PITCH_GYRO_ANGLE_PID_KP,PITCH_GYRO_ANGLE_PID_KI,PITCH_GYRO_ANGLE_PID_KD
    ,PITCH_GYRO_ANGLE_PID_MAX_OUT,PITCH_GYRO_ANGLE_PID_MAX_IOUT);//云台pitch轴角度pid
    
    first_order_filter_init(&Chassis.chassis_cmd_slow_set_vx,CHASSIS_CONTROL_TIME,CHASSIS_ACCEL_X_NUM);//缓启动Vx
    first_order_filter_init(&Chassis.chassis_cmd_slow_set_vy,CHASSIS_CONTROL_TIME,CHASSIS_ACCEL_Y_NUM);//缓启动Vy
    
    USART1_Init(100000);
    USART3_Init(115200);
    USART6_Init(115200);
    CAN1_Init( );
    CAN2_Init( );
    TIM6_Init( );
    TIM2_Init( );
}

