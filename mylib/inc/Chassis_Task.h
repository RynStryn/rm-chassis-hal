#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "config.h"
#include "pid.h"
#include "stm32f4xx_hal.h"
typedef struct {
    float angle;
    float angle_base;
    float angle_rel_base;
    float angle_base2;
    float omega;
} gyro_t;

typedef struct {
    float Vel_ang[4];
    float Vel_ang_ref[4];
    float angle_rel_yaw[4];  // 枪口指向的方向是正方向(Y)
} wheel_t;

typedef enum {
    CHASSIS_RELAX = 0,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_INDEPENDENT,  // 纯平移
    CHASSIS_ROTATE,
    CHASSIS_REV_ROTATE,
} chassis_mode_e;

typedef struct {
    float          angle_rel_yaw_fdb;
    float          angle_rel_yaw_ref;
    float          omega_ref;
    float          omega_fdb;
    float          V_X_ref;//往右
    float          V_X_fdb;
    float          V_Y_ref;//往前
    float          V_Y_fdb;
    chassis_mode_e mode;
    chassis_mode_e last_mode;
} chassis_t;
// 这些量省略rel_base
typedef struct {
    float yaw_angle_fdb;  //
    float yaw_angle_ref;
    float yaw_omega_fdb;
    float yaw_omega_ref;
} gimbal_t;


extern chassis_t g_Chassis;
extern wheel_t   g_wheel;
extern gyro_t    g_gyro;
void  chassis_pid_init(void);
void  chassis_task(chassis_t* chassis, wheel_t* wheel, gyro_t* gyro);
void  gimbal_task(void);

#endif
