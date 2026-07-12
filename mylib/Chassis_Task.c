#include "Chassis_Task.h"
#include <math.h>
#include "Can_Motor.h"
#include "Remote_Control.h"

extern CAN_HandleTypeDef hcan2;

// 假装有一个云台,其响应是即时的
gimbal_t  g_gimbal;
gyro_t    g_gyro;
chassis_t g_Chassis = {0};
wheel_t   g_wheel   = {0};

static const float L                       = 0.23f;
static const float wheel_placement_bias[4] = {45.0f, 135.0f, -135.0f, -45.0f};
static const float ROTATE_SPEED            = -1.0f * RAD_TO_DEG;

PID_t pid_wheel[4];
PID_t pid_chassis_follow_yaw;


void chassis_pid_init(void)
{
    for (int i = 0; i < 4; i++)
    {
        PID_Init(
            &pid_wheel[i], PID_POSITION, 1.0f, 0.0f, 0.002f, 10000.0f, 5000.0f);
    }
    PID_Init(&pid_chassis_follow_yaw,
        PID_POSITION,
        1.2f,
        0.0f,
        0.0f,
        3000.0f,
        1000.0f);
}


void wheel_angle_rel_yaw_set(wheel_t* wheel)
{

}
static void resolve_update_wheel_ref(chassis_t* chassis, wheel_t* wheel)
{
    float Vel_lin_ref[4] = {0};
    for (int i = 0; i < 4; i++)
    {
        float theta           = wheel->angle_rel_yaw[i] * DEG_TO_RAD;
        Vel_lin_ref[i]        = (chassis->V_Y_ref * cosf(theta)
            + chassis->V_X_ref * sinf(theta) + chassis->omega_ref * L);
        wheel->Vel_ang_ref[i] = (Vel_lin_ref[i] / WHEEL_RADIUS) * RAD_TO_DEG;
    }
}


void wheelset_cal_send(wheel_t* wheel)
{
    for (int i = 0; i < 4; i++)
    {
        PID_Calc(&pid_wheel[i], wheel->Vel_ang[i], wheel->Vel_ang_ref[i]);
        // pid_wheel[i].out=0;
    }

    C620_Out_Send(&hcan2,
        pid_wheel[0].out,
        pid_wheel[1].out,
        pid_wheel[2].out,
        pid_wheel[3].out);
}

static void Chassis_Clear(void)
{
    for (int i = 0; i < 4; i++)
    {
        PID_Clear(&pid_wheel[i]);
    }
    PID_Clear(&pid_chassis_follow_yaw);
    g_Chassis.V_X_ref   = 0;
    g_Chassis.V_Y_ref   = 0;
    g_Chassis.omega_ref = 0;
    // g_gyro.angle_base   = g_gyro.angle;
    g_gimbal.yaw_angle_ref=g_gyro.angle;
    g_Chassis.angle_rel_yaw_fdb=0;
}

void wheel_fdb_update(void)
{
    for (int i = 0; i < 4; i++)
        g_wheel.Vel_ang[i] = Motor_Encoder[i].Omega_rpm * RPM_TO_DPS;
}

void gimbal_task() {}
int flag=1;
void chassis_task(chassis_t* chassis, wheel_t* wheel, gyro_t* gyro)
{
    if (flag==1)
    {
        g_gyro.angle_base=g_gyro.angle;
        flag=0;
    }
    //放到最前面,否则容易有量不更新
    if (chassis->last_mode != chassis->mode)
    {
        Chassis_Clear();
    }
    g_gimbal.yaw_angle_ref += yaw_ang_inc;
    // 假定即时响应
    g_gimbal.yaw_angle_fdb = g_gimbal.yaw_angle_ref;
    // g_gimbal.yaw_omega_fdb = g_gimbal.yaw_omega_ref;
    g_Chassis.angle_rel_yaw_fdb
        = g_gyro.angle_rel_base - g_gimbal.yaw_angle_fdb;
    // g_Chassis.angle_rel_yaw_ref
    //     = g_gyro.angle_rel_base - g_gimbal.yaw_angle_ref;

    wheel_fdb_update();
    switch (chassis->mode)
    {
        case CHASSIS_RELAX:
        {
            Chassis_Clear();
            C620_Out_Send(&hcan2, 0, 0, 0, 0);
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL:
        {
    g_gimbal.yaw_angle_ref -= (2*yaw_ang_inc);

            for (int i = 0; i < 4; i++)
            {
                wheel->angle_rel_yaw[i]
                    = wheel_placement_bias[i];// - g_gimbal.yaw_angle_fdb;
            }
            chassis->omega_ref = PID_Calc(
                &pid_chassis_follow_yaw, g_gyro.angle_rel_base - g_gimbal.yaw_angle_fdb, 0);
            resolve_update_wheel_ref(chassis, wheel);
            wheelset_cal_send(wheel);
            break;
        }
        case CHASSIS_INDEPENDENT:
        {

            for (int i = 0; i < 4; i++)
            {
                wheel->angle_rel_yaw[i]
                    = wheel_placement_bias[i] - g_gimbal.yaw_angle_fdb;
            }
            chassis->omega_ref =0;
            // PID_Calc(
            //     &pid_chassis_follow_yaw, g_gyro.angle, g_gyro.angle_base2);
            resolve_update_wheel_ref(chassis, wheel);
            wheelset_cal_send(wheel);
            break;
        }
        case CHASSIS_ROTATE:
        {
            // 小陀螺时,依然是假想云台指向正方向

            for (int i = 0; i < 4; i++)
            {
                wheel->angle_rel_yaw[i]
                    = wheel_placement_bias[i] - g_gyro.angle;
            }
            chassis->omega_ref =
                // PID_Calc(
                //   &pid_chassis_follow_yaw, g_gyro.angle, g_gyro.angle_base)
                +ROTATE_SPEED;
            resolve_update_wheel_ref(chassis, wheel);
            wheelset_cal_send(wheel);
            break;
        }
        case CHASSIS_REV_ROTATE:
        {
            for (int i = 0; i < 4; i++)
            {
                wheel->angle_rel_yaw[i]
                    = wheel_placement_bias[i] - g_gyro.angle ;
            }
            chassis->omega_ref =
                // PID_Calc(
                //   &pid_chassis_follow_yaw, g_gyro.angle, g_gyro.angle_base)
                -ROTATE_SPEED;
            resolve_update_wheel_ref(chassis, wheel);
            wheelset_cal_send(wheel);
            break;
        }
        default:
            break;
    }
    chassis->last_mode = chassis->mode;
}
