#ifndef __FAULT_DIAGNOSIS_TASK_H__
#define __FAULT_DIAGNOSIS_TASK_H__

#include "stm32f4xx_hal.h"

typedef enum {
    DISCONNECTED,
    CONNECTED,
    INACTIVE,
} Link_State_e;

typedef struct {
    int Counter;
    Link_State_e Link_State;
} Equipment_State_t;

typedef struct {
    Equipment_State_t Remote_Control;
    Equipment_State_t Gimbal_Yaw_Gyro;
    Equipment_State_t Equipment_Visual_Equipment_Auto_Aim;
} Peripheral_State_t;

extern Peripheral_State_t Peripheral_State;

void Link_State_Set(Equipment_State_t* Equipment_State, Link_State_e Link_State);
void Equipment_Cnt_Clear(Equipment_State_t* Equipment_State);
void Peripheral_State_Init(Peripheral_State_t* Peripheral_State);
void Equipment_State_Judge(Equipment_State_t* Equipment_State, float T);
void Peripheral_State_Judge(Peripheral_State_t* Peripheral_State, float T);
void Offline_Protect(void);

#endif
