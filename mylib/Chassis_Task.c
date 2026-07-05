#include "inc/Chassis_Task.h"
#include "inc/Can_Motor.h"
#include "inc/config.h"
#include <math.h>
extern CAN_HandleTypeDef hcan2;

Chassis_t Chassis = {
    .wheel[0].State = WHEEL_NORMAL,
    .wheel[1].State = WHEEL_NORMAL,
    .wheel[2].State = WHEEL_NORMAL,
    .wheel[3].State = WHEEL_NORMAL,
    .Flag_Power_Control_Auto = 1,
};

void Chassis_Dirction_Angle_Set(Chassis_t* Chassis, float Dirction_Angle_deg)
{
    Chassis->Dirction_Angle_deg = Dirction_Angle_deg;
}

void Chassis_V_Ref_Set(Chassis_t* Chassis, float _V_x, float _V_y, float _w)
{
    Chassis->V_x = _V_x;
    Chassis->V_y = _V_y;
    Chassis->w   = _w;

    if ( Chassis->V_x > CHASSIS_VX_LIMIT )
        Chassis->V_x = CHASSIS_VX_LIMIT;
    else if ( Chassis->V_x < -CHASSIS_VX_LIMIT )
        Chassis->V_x = -CHASSIS_VX_LIMIT;

    if ( Chassis->V_y > CHASSIS_VY_LIMIT )
        Chassis->V_y = CHASSIS_VY_LIMIT;
    else if ( Chassis->V_y < -CHASSIS_VY_LIMIT )
        Chassis->V_y = -CHASSIS_VY_LIMIT;

    if ( Chassis->Chassis_Mode == CHASSIS_CW_ROTATE
         || Chassis->Chassis_Mode == CHASSIS_CCW_ROTATE
)
    {
    } else
    {
        if ( Chassis->w > CHASSIS_W_LIMIT )
            Chassis->w = CHASSIS_W_LIMIT;
        else if ( Chassis->w < -CHASSIS_W_LIMIT )
            Chassis->w = -CHASSIS_W_LIMIT;
    }
}

void Chassis_Auto_Escape(Chassis_t* _Chassis, float _T)
{
    int static _CNT[4];
    float static _Time[4];

    for ( int i = 0; i < 4; i++ )
    {
        if ( fabs(_Chassis->wheel[i].Speed_Ref - _Chassis->wheel[i].Speed_Fdb)
             > WHEEL_GET_TRAP_V_THRESHOLD )
        {
            if ( _CNT[i] < 20000 )
                _CNT[i]++;
        } else
            _CNT[i] = 0;

        _Time[i] = _CNT[i] * _T;
        if ( _Time[i] > WHEEL_GET_TRAP_TIME_THRESHOLD )
        {
            _Chassis->wheel[i].State = WHEEL_GET_TRAP;
        } else
        {
            _Chassis->wheel[i].State = WHEEL_NORMAL;
        }
    }

    if ( _Chassis->wheel[0].State == WHEEL_GET_TRAP
         || _Chassis->wheel[1].State == WHEEL_GET_TRAP
         || _Chassis->wheel[2].State == WHEEL_GET_TRAP
         || _Chassis->wheel[3].State == WHEEL_GET_TRAP )
    {
        _Chassis->V_x = _Chassis->V_x * CHASSIS_SPEED_UP_COEFFICIENT;
    }
}



void Chassis_Ramp(Chassis_t* _Chassis)
{
    float static V_x_Last, V_y_Last, W_Last;
    u16 static Ramp_CNT;
    u16 static Chassis_Fast_CNT_x, Chassis_Fast_CNT;

    Chassis_Fast_CNT = 0;
    if ( fabs(_Chassis->wheel[0].Speed_Fdb) > 0.5
         || fabs(_Chassis->wheel[1].Speed_Fdb) > 0.5
         || fabs(_Chassis->wheel[2].Speed_Fdb) > 0.5
         || fabs(_Chassis->wheel[3].Speed_Fdb) > 0.5 )
    {
        if ( Chassis_Fast_CNT_x < 1000 )
            Chassis_Fast_CNT++;
    } else
        Chassis_Fast_CNT = 0;

    if ( Ramp_CNT < 10 )
    {
        Ramp_CNT++;
        _Chassis->V_x = V_x_Last;
        _Chassis->V_y = V_y_Last;
        _Chassis->w   = W_Last;
    } else
    {
        Ramp_CNT = 0;
        if ( _Chassis->V_x != 0 && Chassis_Fast_CNT < 1000 )
        {
            if ( V_x_Last * _Chassis->V_x < 0 )
                _Chassis->V_x = 0;
            else
            {
                if ( _Chassis->V_x - V_x_Last > CHASSIS_RAMP_X_STEP_SIZE )
                    _Chassis->V_x = V_x_Last + CHASSIS_RAMP_X_STEP_SIZE;
                else if ( _Chassis->V_x - V_x_Last < -CHASSIS_RAMP_X_STEP_SIZE )
                    _Chassis->V_x = V_x_Last - CHASSIS_RAMP_X_STEP_SIZE;
            }
        }

        if ( _Chassis->V_y != 0 && Chassis_Fast_CNT < 1000 )
        {
            if ( V_y_Last * _Chassis->V_y < 0 )
                _Chassis->V_y = 0;
            else
            {
                if ( _Chassis->V_y - V_y_Last > CHASSIS_RAMP_Y_STEP_SIZE )
                    _Chassis->V_y = V_y_Last + CHASSIS_RAMP_Y_STEP_SIZE;
                else if ( _Chassis->V_y - V_y_Last < -CHASSIS_RAMP_Y_STEP_SIZE )
                    _Chassis->V_y = V_y_Last - CHASSIS_RAMP_Y_STEP_SIZE;
            }
        }

        if ( W_Last * _Chassis->w < 0 )
        {
            _Chassis->w = 0;
        } else
        {
            if ( _Chassis->w - W_Last > CHASSIS_RAMP_W_STEP_SIZE )
                _Chassis->w = W_Last + CHASSIS_RAMP_W_STEP_SIZE;
            else if ( _Chassis->w - W_Last < -CHASSIS_RAMP_W_STEP_SIZE )
                _Chassis->w = W_Last - CHASSIS_RAMP_W_STEP_SIZE;
        }
    }

    V_x_Last = _Chassis->V_x;
    V_y_Last = _Chassis->V_y;
    W_Last   = _Chassis->w;
}

float Wheel_angVel_to_linVel(float spd_dps,float spd_mps)
{
    spd_mps= (spd_dps / M3508_DEG_PER_REV)
           * (PI * CHASSIS_WHEEL_RADIUS) / M3508_REDUCTION_RATIO;
    return spd_mps;
}



void Chassis_Wheel_Pid_Init(Chassis_t* Chassis)
{
    for ( int i = 0; i < 4; i++ )
        PID_struct_init(
            &(Chassis->wheel[i].Speed_Pid), POSITION_PID, 15000, 6000, 35000, 0, 0);
}

void Chassis_Wheel_Fdb_Set(wheel_t* w, float fdb)
{
    w->Speed_Fdb = fdb;
}

void Chassis_Resolve_Wheel2Vel(Chassis_t* Chassis)
{
    float static Dirction_Angle_rad;
    Dirction_Angle_rad = Chassis->Dirction_Angle_deg * 3.1415926f / 180.0f;

    Chassis->wheel[0].Speed_Ref = -Chassis->V_y * sin(Dirction_Angle_rad)
                                  + Chassis->V_x * cos(Dirction_Angle_rad)
                                  - Chassis->w;
    Chassis->wheel[1].Speed_Ref = Chassis->V_y * cos(Dirction_Angle_rad)
                                  + Chassis->V_x * sin(Dirction_Angle_rad)
                                  - Chassis->w;
    Chassis->wheel[2].Speed_Ref = Chassis->V_y * sin(Dirction_Angle_rad)
                                  - Chassis->V_x * cos(Dirction_Angle_rad)
                                  - Chassis->w;
    Chassis->wheel[3].Speed_Ref = -Chassis->V_y * cos(Dirction_Angle_rad)
                                  - Chassis->V_x * sin(Dirction_Angle_rad)
                                  - Chassis->w;
}

void Chassis_Wheel_Cal_Pid(Chassis_t* Chassis)
{
    for ( int i = 0; i < 4; i++ )
    {
        pid_calc(
            &Chassis->wheel[i].Speed_Pid,
            Chassis->wheel[i].Speed_Fdb,
            Chassis->wheel[i].Speed_Ref);
    }
}

void Chassis_Wheel_CalSend() {
    Chassis_Wheel_Cal_Pid(&Chassis);
    C620_Out_Send(
    &hcan2,
    Chassis.wheel[0].Speed_Pid.out,
    Chassis.wheel[1].Speed_Pid.out,
    Chassis.wheel[2].Speed_Pid.out,
    Chassis.wheel[3].Speed_Pid.out);
}
void Chassis_Wheel_Set_Resolve(
    Chassis_t* Chassis,
    float      V_x,
    float      V_y,
    float      w) {
    Chassis_V_Ref_Set(Chassis, V_x, V_y, w);
    Chassis_Ramp(Chassis);
    Chassis_Resolve_Wheel2Vel(Chassis);
}
void Chassis_Wheel_Task(
    Chassis_t* Chassis,
    float      V_x,
    float      V_y,
    float      w)
{
    Chassis_Wheel_Set_Resolve(Chassis, V_x, V_y, w);
    Chassis_Wheel_CalSend();
}

void Chassis_Speed_Cal(Chassis_t* Chassis)
{
    float static Dirction_Angle_rad;
    Dirction_Angle_rad = Chassis->Dirction_Angle_deg * 3.1415926f / 180.0f;
    Chassis->V_x_Fdb      = Chassis->wheel[0].Speed_Fdb * cos(Dirction_Angle_rad)
                       + Chassis->wheel[1].Speed_Fdb * sin(Dirction_Angle_rad)
                       + Chassis->wheel[2].Speed_Fdb * -cos(Dirction_Angle_rad)
                       + Chassis->wheel[3].Speed_Fdb * -sin(Dirction_Angle_rad);

    Chassis->V_y_Fdb = Chassis->wheel[0].Speed_Fdb * -sin(Dirction_Angle_rad)
                       + Chassis->wheel[1].Speed_Fdb * cos(Dirction_Angle_rad)
                       + Chassis->wheel[2].Speed_Fdb * sin(Dirction_Angle_rad)
                       + Chassis->wheel[3].Speed_Fdb * -cos(Dirction_Angle_rad);
}

