#ifndef __CAN_SEND_H
#define __CAN_SEND_H
#include <stdint.h>

#define CHASSIS_M2006_CONTROL_ID             0x200
#define M2006_M0_FEEDBACK_ID                 0x201
#define M2006_M1_FEEDBACK_ID                 0x202
#define M2006_M2_FEEDBACK_ID                 0x203
#define M2006_M3_FEEDBACK_ID                 0x204

#define M2006_POKE_FEEDBACK_ID               0x207
#define POKE_M2006_CONTROL_ID                0x1FF

#define GM6020_YAW_FEEDBACK_ID               0x205
#define GM6020_PITCH_FEEDBACK_ID             0x206
#define GM6020_CONTROL_ID                    0x1FF

#define M3508_FRICTION_CONTROL_ID            0x200
#define M3508_FRICTION1_FEEDBACK_ID          0x201
#define M3508_FRICTION2_FEEDBACK_ID          0x202

typedef struct
{
    int32_t raw_value;
    int32_t last_raw_value;
    int32_t ecd_value;
    int32_t diff;
    int32_t round_cnt;
    int Omega;
    int8_t temperature;
    int16_t current;
} Encoder;

extern uint8_t Motor_Online_Flag[4];

void CAN_Motor_Data_Receive_Ex(uint32_t StdId, uint8_t* data, Encoder* Yaw_Motor_Encoder, Encoder* Pitch_Motor_Encoder, Encoder* Chassis_M3508_Encoder);
void Encoder_Process(Encoder* Motor_Encoder, uint8_t* data);
void CAN_Send_Task(int16_t* Gimbal_Current_Set,int16_t* Chassis_Current_Set);

#endif
