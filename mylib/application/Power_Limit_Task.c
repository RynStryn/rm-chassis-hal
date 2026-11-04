#include "Power_Limit_Task.h"
#include <math.h>
#include "Chassis_Task.h"
#include "JudgeSys.h"

#define VAL_LIMIT(x, min, max) \
    do                         \
    {                          \
        if ((x) < (min))       \
            (x) = (min);       \
        else if ((x) > (max))  \
            (x) = (max);       \
    } while (0)

typedef struct {
    uint8_t My_Recharge_Area;
    uint8_t My_Recharge_Area_;
} ext_rfid_status_t;
ext_rfid_status_t judge_rfid = {0};

typedef struct {
    float cap_voltage_filte;
} can_capacitance_message_t;
can_capacitance_message_t can_capacitance_message = {0};

static float LIMIT_RATE_DEFAULT = 0.5f;
float Rate_PidOut_iq[4] = {1.0f};

typedef enum {
    POWERLIMIT_OK = 0,
    POWERLIMIT_DEGRADATION,
    POWERLIMIT_NO_SOLUTION,
} PowerLimit_Solve_Status_e;

static uint8_t get_postive_root_quadratic(float a, float b, float c, float* root)
{
    if (fabsf(a) < 1e-6f)
    {
        if (fabsf(b) < 1e-6f)
            return POWERLIMIT_DEGRADATION;

        *root = -c / b;
        return POWERLIMIT_OK;
    }

    float delta = b * b - 4.0f * a * c;

    if (delta < 0.0f)
        return POWERLIMIT_NO_SOLUTION;

    if (delta == 0.0f)
    {
        *root = -b / (2.0f * a);
        return POWERLIMIT_OK;
    }

    *root = (-b + sqrtf(delta)) / (2.0f * a);
    return POWERLIMIT_OK;
}

void PowerLimit_GetTotalRate(Power_Limit_t* Power_limit)
{
    float a[4], b[4];

    for (int i = 0; i < 4; i++)
    {
        a[i] =
            Power_limit->Speed_Ref[i] * (Power_limit->Pid[i].Kp + Power_limit->Pid[i].Kd);
        b[i] =
            -(Power_limit->Pid[i].Kp + Power_limit->Pid[i].Kd) * Power_limit->Speed_Fdb[i]
            + Power_limit->Pid[i].Iout
            - Power_limit->Pid[i].Kd * Power_limit->Pid[i].error[1];
    }

    float M = 0.0f, N = 0.0f, L = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        M += FACTOR_2 * a[i] * a[i];

        N += 2.0f * FACTOR_2 * a[i] * b[i] + FACTOR_1 * a[i]
             + I_TIMES_V_TO_WATT * Power_limit->Speed_Fdb[i] * a[i];

        L += FACTOR_2 * b[i] * b[i] + FACTOR_1 * b[i] + I_TIMES_V_TO_WATT * Power_limit->Speed_Fdb[i] * b[i]
             + FACTOR_0;
    }

    float root = 0.5f;
    uint8_t status = get_postive_root_quadratic(M, N, L - Power_limit->Max_Power, &root);

    if (status == POWERLIMIT_DEGRADATION)
        root = 1.0f;
    else if (status == POWERLIMIT_NO_SOLUTION)
        root = 0.5f;

    if (root > 1.0f)
        root = 1.0f;
    if (root < 0.0f)
        root = 0.0f;

    Power_limit->Rate_Vref = root;
}

void PowerLimit_GetMotorRate(Power_Limit_t* Power_limit)
{
    for (int i = 0; i < 4; i++)
    {
        float root = 0.5f;




        float a = FACTOR_2 * Power_limit->Pid[i].out * Power_limit->Pid[i].out;
        float b =
            FACTOR_1 * Power_limit->Pid[i].out
            + I_TIMES_V_TO_WATT * Power_limit->Speed_Fdb[i] * Power_limit->Pid[i].out;
        float c = FACTOR_0 - Power_limit->wheel_max_power[i];

        uint8_t status = get_postive_root_quadratic(a, b, c, &root);

        if (status == POWERLIMIT_DEGRADATION)
            root = 1.0f;
        else if (status == POWERLIMIT_NO_SOLUTION)
            root = 0.5f;

        if (root > 1.0f)
            root = 1.0f;
        if (root < 0.0f)
            root = 0.0f;

        Power_limit->Rate_PidOut_iq[i] = root;
    }
}

void PowerLimit_Distribute(Power_Limit_t* Power_limit)
{
    static const float Power_Distribution_min = 7.0f;
    float Chassis_Error[4] = {0};
    float Chassis_Error_Sum = 0.0f;
    int Power_Surplus_CNT = 0;

    for (int i = 0; i < 4; i++)
    {
        Chassis_Error[i] =
            fabsf(Power_limit->Speed_Ref[i]) - fabsf(Power_limit->Speed_Fdb[i]);
        Chassis_Error[i] = (Chassis_Error[i] > 0) ? Chassis_Error[i] : 0;
        Chassis_Error_Sum += Chassis_Error[i];
        if (Chassis_Error[i] == 0)
            Power_Surplus_CNT++;
    }

    if (Chassis_Error_Sum < 1e-6f)
    {
        for (int i = 0; i < 4; i++)
            Power_limit->wheel_max_power[i] = Power_limit->Max_Power / 4;
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        if (Chassis_Error[i] == 0)
            Power_limit->wheel_max_power[i] = Power_Distribution_min;
        else
            Power_limit->wheel_max_power[i] =
                (Power_limit->Max_Power - Power_Distribution_min * Power_Surplus_CNT)
                * Chassis_Error[i] / Chassis_Error_Sum;
    }
}

void Max_Power_Get(void)
{
    if (judge_rfid.My_Recharge_Area
        || judge_rfid.My_Recharge_Area_)
    {
        g_Chassis.Flag_Out_Break = 0;
    }

    if (g_Chassis.Flag_Out_Break == 1 && can_capacitance_message.cap_voltage_filte > 5)
    {
        if (g_Chassis.mode == CHASSIS_FLYING_SLOPE)
            g_Chassis.Max_Power = 330;
        else
        {
            switch (g_Chassis.Chassis_Move_State)
            {
                case CHAS_STATE_RUN_READY:
                    g_Chassis.Max_Power =
                        judge_rece_mesg.game_robot_state.chassis_power_limit + 100;
                    break;
                case CHAS_STATE_RUN:
                    g_Chassis.Max_Power =
                        judge_rece_mesg.game_robot_state.chassis_power_limit + 50;
                    break;
                case CHAS_STATE_UP_SLOPE:
                    g_Chassis.Max_Power =
                        judge_rece_mesg.game_robot_state.chassis_power_limit + 80;
                    break;
                case CHAS_STATE_DOWN_LOW_SLOPE:
                case CHAS_STATE_DOWN_HIGH_SLOPE:
                    g_Chassis.Max_Power =
                        judge_rece_mesg.game_robot_state.chassis_power_limit;
                    break;
                default:
                    break;
            }
        }
    }
    else
    {
        static float Power_Up_V = 0, Power_Keep_V = 0;
        static int Power_Up_Flag = 0, Power_Keep_Flag = 0;
        int Power_Up_CNT = 0, Power_Keep_CNT = 0;

        Power_Up_V =
            1.45f + 0.022f * (judge_rece_mesg.game_robot_state.chassis_power_limit - 55);
        VAL_LIMIT(Power_Up_V, 0, CHASSIS_VX_LIMIT);
        Power_Keep_V = Power_Up_V + 0.2f;

        for (int i = 0; i < 4; i++)
        {
            if (fabsf(g_wheel.Vel_ang[i]) < Power_Up_V)
                Power_Up_CNT++;
            else if (fabsf(g_wheel.Vel_ang[i]) > Power_Keep_V)
                Power_Keep_CNT++;
        }

        Power_Up_Flag = (Power_Up_CNT >= 2) ? 1 : 0;
        Power_Keep_Flag = (Power_Keep_CNT >= 2) ? 1 : 0;

        if (can_capacitance_message.cap_voltage_filte > 5)
        {
            switch (g_Chassis.Chassis_Move_State)
            {
                case CHAS_STATE_RUN_READY:
                case CHAS_STATE_DOWN_LOW_SLOPE:
                case CHAS_STATE_DOWN_HIGH_SLOPE:
                case CHAS_STATE_STOP:
                    g_Chassis.Max_Power =
                        judge_rece_mesg.game_robot_state.chassis_power_limit;
                    break;

                case CHAS_STATE_UP_SLOPE:
                    if (judge_rece_mesg.game_robot_state.chassis_power_limit < 100)
                        g_Chassis.Max_Power = 100;
                    else
                        g_Chassis.Max_Power =
                            judge_rece_mesg.game_robot_state.chassis_power_limit;
                    break;

                case CHAS_STATE_RUN:
                    if (Power_Up_Flag)
                    {
                        if (can_capacitance_message.cap_voltage_filte > 8)
                            g_Chassis.Max_Power =
                                judge_rece_mesg.game_robot_state.chassis_power_limit
                                + (float)judge_rece_mesg.power_heat_data.chassis_power_buffer - 10;
                        else
                            g_Chassis.Max_Power =
                                judge_rece_mesg.game_robot_state.chassis_power_limit;
                    }
                    else if (Power_Keep_Flag)
                    {
                        if (g_Chassis.Chassis_Speed_Mode == CHASSIS_S_NORMAL)
                            g_Chassis.Max_Power =
                                judge_rece_mesg.game_robot_state.chassis_power_limit;
                        else if (g_Chassis.Chassis_Speed_Mode == CHASSIS_S_FAST)
                            g_Chassis.Max_Power =
                                judge_rece_mesg.game_robot_state.chassis_power_limit + 10;
                    }
                    break;

                default:
                    break;
            }
        }
        else
        {
            g_Chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit;
        }
    }

    VAL_LIMIT(g_Chassis.Max_Power, 0,
              can_capacitance_message.cap_voltage_filte * 16
                  + judge_rece_mesg.game_robot_state.chassis_power_limit);
}

void power_limit_apply(float* vel_ang_ref, float* vel_ang, PID_t* pid, float max_power)
{
    Power_Limit_t power_limit = {0};
    for (int i = 0; i < 4; i++)
    {
        power_limit.Speed_Ref[i] =
            vel_ang_ref[i] * (DEG_TO_RAD * WHEEL_RADIUS / WHEEL_REDUCTION);
        power_limit.Speed_Fdb[i] =
            vel_ang[i] * (DEG_TO_RAD * WHEEL_RADIUS / WHEEL_REDUCTION);
    }
    power_limit.Pid = pid;
    power_limit.Max_Power = max_power;

    PowerLimit_Distribute(&power_limit);
    PowerLimit_GetTotalRate(&power_limit);
    PowerLimit_GetMotorRate(&power_limit);

    for (int i = 0; i < 4; i++)
    {
        vel_ang_ref[i] *= power_limit.Rate_Vref;
        Rate_PidOut_iq[i] = power_limit.Rate_PidOut_iq[i];
    }
}
