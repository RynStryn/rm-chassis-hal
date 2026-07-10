#ifndef __PID_H
#define __PID_H

/*********************************限幅*********************************/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
/*********************************限幅*********************************/







/*********************************宏定义*********************************/
//底盘电机速度环PID
#define M2006_MOTOR_SPEED_PID_KP 15000.0f
#define M2006_MOTOR_SPEED_PID_KI 10.0f
#define M2006_MOTOR_SPEED_PID_KD 0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT  10000.0f
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID.
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 2.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        2900.0f
#define PITCH_SPEED_PID_KI        60.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        35.0f
#define YAW_SPEED_PID_KI        1.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ANGLE_PID_KP 15.0f                  //pitch参数未调
#define PITCH_GYRO_ANGLE_PID_KI 0.0f
#define PITCH_GYRO_ANGLE_PID_KD 0.0f
#define PITCH_GYRO_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ANGLE_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ANGLE_PID_KP        10.0f
#define YAW_GYRO_ANGLE_PID_KI        0.0f
#define YAW_GYRO_ANGLE_PID_KD        0.2f
#define YAW_GYRO_ANGLE_PID_MAX_OUT   400.0f
#define YAW_GYRO_ANGLE_PID_MAX_IOUT  0.0f
/*********************************宏定义*********************************/


/********************************PID结构体******************************/
typedef enum
{
    PID_POSITION,  //位置式PID
    PID_DELTA,     //增量式PID
}PID_Mode_e;

typedef struct
{
    PID_Mode_e mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PID_t;
/**********************************PID结构体***********************************/




/**********************************函数声明**********************************/
void PID_Init(PID_t *pid, PID_Mode_e mode,float kp,float ki,float kd, float max_out, float max_iout);
float PID_Calc(PID_t* pid, float fdb, float ref);
void PID_Clear(PID_t *pid);
/**********************************函数声明**********************************/



#endif
