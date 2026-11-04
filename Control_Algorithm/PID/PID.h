#ifndef __PID_H
#define __PID_H
#include <stdint.h>
#include <stdbool.h>

/*********************************�޷�*********************************/
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
/*********************************�޷�*********************************/
    
    
    
    
    
    
    
/*********************************�궨��*********************************/
//���̵���ٶȻ�PID
#define M2006_MOTOR_SPEED_PID_KP 15000.0f
#define M2006_MOTOR_SPEED_PID_KI 10.0f
#define M2006_MOTOR_SPEED_PID_KD 0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT  10000.0f
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
    
//������ת����PID.
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 2.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP        2900.0f
#define PITCH_SPEED_PID_KI        60.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP        35.0f
#define YAW_SPEED_PID_KI        1.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID��������������� 
#define PITCH_GYRO_ANGLE_PID_KP 15.0f                  //pitch����δ��
#define PITCH_GYRO_ANGLE_PID_KI 0.0f
#define PITCH_GYRO_ANGLE_PID_KD 0.0f
#define PITCH_GYRO_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ANGLE_PID_MAX_IOUT 0.0f

//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ANGLE_PID_KP        10.0f
#define YAW_GYRO_ANGLE_PID_KI        0.0f
#define YAW_GYRO_ANGLE_PID_KD        0.2f
#define YAW_GYRO_ANGLE_PID_MAX_OUT   400.0f
#define YAW_GYRO_ANGLE_PID_MAX_IOUT  0.0f
/*********************************�궨��*********************************/

    
/********************************PID�ṹ��******************************/
typedef enum 
{
    PID_POSITION,  //λ��ʽPID
    PID_DELTA,     //����ʽPID
}PID_Mode_e;

typedef struct
{
    PID_Mode_e mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

} PID_t;
/**********************************PID�ṹ��***********************************/




/**********************************��������**********************************/
void PID_Init(PID_t *pid, PID_Mode_e mode,float kp,float ki,float kd, float max_out, float max_iout);
float PID_Calc(PID_t* pid, float get, float set);
void PID_Clear(PID_t *pid);
/**********************************��������**********************************/



#endif
