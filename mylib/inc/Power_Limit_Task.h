#ifndef __POWER_CONTROL_TASK_H__
#define __POWER_CONTROL_TASK_H__

#include "config.h"
#include "pid.h"

#define WHEEL_REDUCTION     15.7647f

#define I_TIMES_V_TO_WATT   0.004918f
#define FACTOR_2            1.9982e-07f
#define FACTOR_1           -5.169e-04f
#define FACTOR_0            0.1f

#define POWER_LIMIT_DEFAULT 80.0f
#define POWER_LIMIT_MIN      7.0f

typedef struct {
    float   Speed_Ref[4];
    float   Speed_Fdb[4];
    PID_t  *Pid;
    float   Max_Power;
    float   Rate_Vref;
    float   wheel_max_power[4];
    float   Rate_PidOut_iq[4];
} Power_Limit_t;

void  PowerLimit_GetTotalRate(Power_Limit_t *Power_limit);
void  PowerLimit_GetMotorRate(Power_Limit_t *Power_limit);
void  PowerLimit_Distribute(Power_Limit_t *Power_limit);
extern float Rate_PidOut_iq[4];

void  Max_Power_Get(void);
void  power_limit_apply(float *vel_ang_ref, float *vel_ang, PID_t *pid, float max_power);

#endif
