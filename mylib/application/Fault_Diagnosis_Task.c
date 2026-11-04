#include "Fault_Diagnosis_Task.h"
#include "Remote_Control.h"
#include "Chassis_Task.h"

Peripheral_State_t Peripheral_State;

void Link_State_Set(Equipment_State_t* Equipment_State, Link_State_e Link_State)
{
    Equipment_State->Link_State = Link_State;
}

void Equipment_Cnt_Clear(Equipment_State_t* Equipment_State)
{
    Equipment_State->Counter = 0;
}

void Peripheral_State_Init(Peripheral_State_t* Peripheral_State)
{
    Peripheral_State->Remote_Control.Link_State = CONNECTED;
    Peripheral_State->Remote_Control.Counter    = 0;

    Peripheral_State->Gimbal_Yaw_Gyro.Link_State = CONNECTED;
    Peripheral_State->Gimbal_Yaw_Gyro.Counter    = 0;

    Peripheral_State->Equipment_Visual_Equipment_Auto_Aim.Link_State = CONNECTED;
    Peripheral_State->Equipment_Visual_Equipment_Auto_Aim.Counter    = 0;
}

void Equipment_State_Judge(Equipment_State_t* Equipment_State, float T)
{
    Equipment_State->Counter++;
    if (Equipment_State->Counter * T > 1)
    {
        Equipment_State->Link_State = DISCONNECTED;
Offline_Protect();
    }
    else
        Equipment_State->Link_State = CONNECTED;
}

void Peripheral_State_Judge(Peripheral_State_t* Peripheral_State, float T)
{
    Equipment_State_Judge(&Peripheral_State->Remote_Control, T);
  //  Equipment_State_Judge(&Peripheral_State->Gimbal_Yaw_Gyro, T);
   // Equipment_State_Judge(&Peripheral_State->Equipment_Visual_Equipment_Auto_Aim, T);
}

void Offline_Protect(void)
{
    RC_Mode = OFFLINE;
    g_Chassis.mode = CHASSIS_RELAX;
}
