#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "PID/pid.h"
#include "config.h"
#include "stm32f4xx_hal.h"

typedef enum {
    WHEEL_NORMAL,
    WHEEL_GET_TRAP,
} Chassis_Wheel_State_e;

typedef struct {
    float                linVel;
    float                Speed_Ref;
    float                Speed_Fdb;
    Chassis_Wheel_State_e State;
    pid_t                Speed_Pid;
} wheel_t;

typedef enum {
    CHASSIS_S_NORMAL,
    CHASSIS_S_FAST,
} Chassis_Speed_Mode_e;

typedef enum {
    BY_CURRENT,
    BY_SPEED,
} Chassis_Power_Limit_Mode_e;

typedef enum {
    CHASSIS_RELAX = 0,
    CHASSIS_STOP,
    CHASSIS_INDEPEND,
    CHASSIS_FOLLOW_GIMBLE,
    CHASSIS_FLYING_SLOPE,
    CHASSIS_CLIMB,
    CHASSIS_CW_ROTATE,
    CHASSIS_CCW_ROTATE,
    CHASSIS_CLOCKWISE_ROTATE_VAR_SPEED,
    CHASSIS_ANTI_CLOCKWISE_ROTATE_VAR_SPEED,
} Chassis_Mode_e;

typedef enum {
    CHAS_STATE_STOP,
    CHAS_STATE_RUN_READY,
    CHAS_STATE_RUN,
    CHAS_STATE_UP_SLOPE,
    CHAS_STATE_DOWN_LOW_SLOPE,
    CHAS_STATE_DOWN_HIGH_SLOPE,
} Chassis_Move_State_e;

typedef struct {
    float                V_x;
    float                V_y;
    float                V_x_Fdb;
    float                V_y_Fdb;
    float                w;
    float                Dirction_Angle_deg;

    Chassis_Speed_Mode_e Chassis_Speed_Mode;
    u8                   Flag_Out_Break;
    u8                   Flag_Power_Control_Auto;

    Chassis_Mode_e             Chassis_Mode;
    Chassis_Mode_e             Chassis_Mode_Last;
    Chassis_Move_State_e       Chassis_Move_State;
    wheel_t wheel[4];
} Chassis_t;

typedef struct {
    float                Power_Limit[4];
    float                Power_Limit_By_Current_k[4];
    float                Max_Power;

    Chassis_Power_Limit_Mode_e Chassis_Power_Limit_Mode;

}power_limit_t;

extern Chassis_t Chassis;

void Chassis_Wheel_Pid_Init(Chassis_t* Chassis);
void Chassis_Resolve_Wheel2Vel(Chassis_t* Chassis);
void Chassis_Wheel_Cal_Pid(Chassis_t* Chassis);
void Chassis_Wheel_Fdb_Set(
    Chassis_t* Chassis,
    float      fbd_0,
    float      fbd_1,
    float      fbd_2,
    float      fbd_3);
void Chassis_Wheel_Task(
    Chassis_t* Chassis,
    float      V_x,
    float      V_y,
    float      w);
void  Chassis_Speed_Cal(Chassis_t* Chassis);
void  Chassis_Dirction_Angle_Set(Chassis_t* Chassis, float Dirction_Angle);
void  Chassis_Out_Limit(Chassis_t* Chassis);
float Wheel_angVel_to_linVel(float spd_dps,float spd_mps);
void  Chassis_V_Ref_Set(Chassis_t* Chassis, float _V_x, float _V_y, float _w);
void  Chassis_State_Detect(Chassis_t* _Chassis);

#endif
