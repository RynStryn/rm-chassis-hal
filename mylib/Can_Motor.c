#include "inc/Can_Motor.h"

#include "Chassis_Task.h"

int wheelIsOK[4] = {0, 0, 0, 0};
volatile Motor_Encoder_t Motor_Encoder[8] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

int can2_count = 0;

static float Float_Limit(float x, float max);

void Can2ReceiveMsgProcess(uint32_t StdId, uint8_t* data)
{
    can2_count++;

    switch (StdId)
    {
        case C620_FEEDBACK_ID_BASE + 1:
            wheelIsOK[0] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_Encoder[0], data)
                : M3508Encoder_Process(&Motor_Encoder[0], data);
            break;
        case C620_FEEDBACK_ID_BASE + 2:
            wheelIsOK[1] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_Encoder[1], data)
                : M3508Encoder_Process(&Motor_Encoder[1], data);
            break;
        case C620_FEEDBACK_ID_BASE + 3:
            wheelIsOK[2] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_Encoder[2], data)
                : M3508Encoder_Process(&Motor_Encoder[2], data);
            break;
        case C620_FEEDBACK_ID_BASE + 4:
            wheelIsOK[3] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_Encoder[3], data)
                : M3508Encoder_Process(&Motor_Encoder[3], data);
            break;
        default:
            break;
    }
}

void C620_Out_Send(
    CAN_HandleTypeDef* hcan,
    float iq1,
    float iq2,
    float iq3,
    float iq4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    int16_t out1, out2, out3, out4;

    out1 = (int16_t)Float_Limit(iq1, C620_CURRENT_LIMIT);
    out2 = (int16_t)Float_Limit(iq2, C620_CURRENT_LIMIT);
    out3 = (int16_t)Float_Limit(iq3, C620_CURRENT_LIMIT);
    out4 = (int16_t)Float_Limit(iq4, C620_CURRENT_LIMIT);

    tx_header.StdId = C620_CONTROL_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = (uint8_t)(out1 >> 8);
    tx_data[1] = (uint8_t)out1;
    tx_data[2] = (uint8_t)(out2 >> 8);
    tx_data[3] = (uint8_t)out2;
    tx_data[4] = (uint8_t)(out3 >> 8);
    tx_data[5] = (uint8_t)out3;
    tx_data[6] = (uint8_t)(out4 >> 8);
    tx_data[7] = (uint8_t)out4;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

void GetEncoderBias(volatile Motor_Encoder_t* v, uint8_t* data)
{
    v->ecd_bias = (data[0] << 8) | data[1];
    v->ecd_value = v->ecd_bias;
    v->last_raw_value = v->ecd_bias;
    v->temp_count++;
}

void M3508Encoder_Process(volatile Motor_Encoder_t* v, uint8_t* data)
{
    v->current = (int16_t)((data[4] << 8) | data[5]);
    int i = 0;
    int32_t temp_sum = 0;

    v->last_raw_value = v->raw_value;
    v->raw_value = (data[0] << 8) | data[1];
    v->diff = v->raw_value - v->last_raw_value;
    if (v->diff < -M3508_ENC_HALF_RESOLUTION)
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + M3508_ENC_RESOLUTION;
    }
    else if (v->diff > M3508_ENC_HALF_RESOLUTION)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - M3508_ENC_RESOLUTION;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    v->ecd_value = v->raw_value + v->round_cnt * M3508_ENC_RESOLUTION;
    v->ecd_angle_deg = (float)(v->raw_value - v->ecd_bias) * M3508_ANGLE_DEG_PER_TICK
                       + v->round_cnt * M3508_DEG_PER_REV;

    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
    if (v->buf_count == M3508_SPEED_FILTER_BUF_SIZE)
    {
        v->buf_count = 0;
    }

    for (i = 0; i < M3508_SPEED_FILTER_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }

    v->filter_rate_rpm = (int16_t)((data[2] << 8) | data[3]);
    v->angVel_dps = (int)((data[2] << 8) | data[3]) * RPM_TO_DPS;
}

static float Float_Limit(float x, float max)
{
    if (x > max) return max;
    if (x < -max) return -max;
    return x;
}

