#ifndef __CAN_MOTOR_H__
#define __CAN_MOTOR_H__

#include "stm32f4xx_hal.h"

typedef struct {
    int32_t raw_value;
    int32_t last_raw_value;
    int32_t ecd_value;
    int32_t diff;
    int32_t temp_count;
    uint8_t buf_count;
    int32_t ecd_bias;
    int32_t ecd_raw_rate;
    int32_t rate_buf[6];
    int32_t round_cnt;
    int16_t filter_rate_rpm;
    int Speed_dps;
    float ecd_angle_deg;
    uint32_t temperature;
    int16_t current;
} Encoder;

extern volatile Encoder Motor_c610c620_Encoder[8];
extern int wheelIsOK[4];

void GetEncoderBias(volatile Encoder *v, uint8_t *data);
void EncoderProcess(volatile Encoder *v, uint8_t *data);

void Can2ReceiveMsgProcess(uint32_t StdId, uint8_t *data);
void Motor_c610andc620_out1(CAN_HandleTypeDef *hcan, float out0, float out1, float out2, float out3);

#endif
