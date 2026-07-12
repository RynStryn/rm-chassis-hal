#ifndef __CAN_MOTOR_H__
#define __CAN_MOTOR_H__

#include "stm32f4xx_hal.h"

#define C620_FEEDBACK_ID_BASE       0x200
#define C620_CONTROL_ID             0x200

#define C620_CURRENT_MAX            16384
#define C620_CURRENT_LIMIT          C620_CURRENT_MAX

#define M3508_ENC_RESOLUTION        8192
#define M3508_REDUCTION_RATIO       (3591.0f / 187.0f)
#define RPM_TO_DPS                  6
#define PI                          3.1415926535f

typedef struct
{
    int32_t raw_value;
    int32_t last_raw_value;
    int32_t ecd_value;
    int32_t diff;
    int32_t round_cnt;
    int Omega_rpm;
    int8_t temperature;
    int16_t current;
} Encoder_t;

extern Encoder_t Motor_Encoder[4];
extern uint8_t Motor_Online_Flag[4];
extern uint8_t wheel_online[4];

void CAN_Start(void);
void Encoder_Process(Encoder_t* Motor_Encoder, uint8_t* data);
void Can2ReceiveMsgProcess(uint32_t StdId, uint8_t* data, Encoder_t* Chassis_M3508_Encoder);
void C620_Out_Send(CAN_HandleTypeDef* hcan, int16_t iq0, int16_t iq1, int16_t iq2, int16_t iq3);

#endif
