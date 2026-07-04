#ifndef __CAN_MOTOR_H__
#define __CAN_MOTOR_H__

#include "stm32f4xx_hal.h"

#define C620_FEEDBACK_ID_BASE       0x200
#define C620_CONTROL_ID             0x200

#define C620_CURRENT_MAX            16384
#define C620_CURRENT_LIMIT          C620_CURRENT_MAX

#define M3508_ENC_RESOLUTION        8192
#define M3508_ENC_HALF_RESOLUTION   (M3508_ENC_RESOLUTION / 2)
#define M3508_ANGLE_DEG_PER_TICK    (360.0f / M3508_ENC_RESOLUTION)
#define M3508_DEG_PER_REV           360.0f
#define M3508_REDUCTION_RATIO       (3591.0f / 187.0f)

#define RPM_TO_DPS                  6

#define M3508_SPEED_FILTER_BUF_SIZE 6

#define PI                          3.1415926535f

typedef struct {
    int32_t raw_value;
    int32_t last_raw_value;
    int32_t ecd_value;
    int32_t diff;
    int32_t temp_count;
    uint8_t buf_count;
    int32_t ecd_bias;
    int32_t ecd_raw_rate;
    int32_t rate_buf[M3508_SPEED_FILTER_BUF_SIZE];
    int32_t round_cnt;
    int16_t filter_rate_rpm;
    int     angVel_dps;
    float   ecd_angle_deg;
    uint32_t temperature;
    int16_t current;
} Motor_Encoder_t;

extern volatile Motor_Encoder_t Motor_Encoder[8];
extern int wheelIsOK[4];

void GetEncoderBias(volatile Motor_Encoder_t *v, uint8_t *data);
void M3508Encoder_Process(volatile Motor_Encoder_t *v, uint8_t *data);

void Can2ReceiveMsgProcess(uint32_t StdId, uint8_t *data);
void C620_Out_Send(CAN_HandleTypeDef *hcan, float iq1, float iq2, float iq3, float iq4);

#endif
