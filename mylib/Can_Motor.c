#include "inc/Can_Motor.h"

int wheelIsOK[4] = {0, 0, 0, 0};

volatile Encoder Motor_c610c620_Encoder[8] = {
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

float Float_Limit(float x, float max);

void Can2ReceiveMsgProcess(uint32_t StdId, uint8_t* data)
{
    can2_count++;

    switch (StdId)
    {
        case 0x201:
            wheelIsOK[0] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_c610c620_Encoder[0], data)
                : EncoderProcess(&Motor_c610c620_Encoder[0], data);
            break;
        case 0x202:
            wheelIsOK[1] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_c610c620_Encoder[1], data)
                : EncoderProcess(&Motor_c610c620_Encoder[1], data);
            break;
        case 0x203:
            wheelIsOK[2] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_c610c620_Encoder[2], data)
                : EncoderProcess(&Motor_c610c620_Encoder[2], data);
            break;
        case 0x204:
            wheelIsOK[3] = 1;
            (can2_count <= 50)
                ? GetEncoderBias(&Motor_c610c620_Encoder[3], data)
                : EncoderProcess(&Motor_c610c620_Encoder[3], data);
            break;
        default:
            break;
    }
}

void Motor_c610andc620_out1(
    CAN_HandleTypeDef* hcan,
    float out0,
    float out1,
    float out2,
    float out3)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    int16_t Out0, Out1, Out2, Out3;

    Out0 = (int16_t)Float_Limit(out0, 30000);
    Out1 = (int16_t)Float_Limit(out1, 30000);
    Out2 = (int16_t)Float_Limit(out2, 30000);
    Out3 = (int16_t)Float_Limit(out3, 30000);

    tx_header.StdId = 0x200;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = (uint8_t)(Out0 >> 8);
    tx_data[1] = (uint8_t)Out0;
    tx_data[2] = (uint8_t)(Out1 >> 8);
    tx_data[3] = (uint8_t)Out1;
    tx_data[4] = (uint8_t)(Out2 >> 8);
    tx_data[5] = (uint8_t)Out2;
    tx_data[6] = (uint8_t)(Out3 >> 8);
    tx_data[7] = (uint8_t)Out3;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

void GetEncoderBias(volatile Encoder* v, uint8_t* data)
{
    v->ecd_bias = (data[0] << 8) | data[1];
    v->ecd_value = v->ecd_bias;
    v->last_raw_value = v->ecd_bias;
    v->temp_count++;
}

void EncoderProcess(volatile Encoder* v, uint8_t* data)
{
    v->current = (data[4] << 8) | data[5];
    int i = 0;
    int32_t temp_sum = 0;

    v->last_raw_value = v->raw_value;
    v->raw_value = (data[0] << 8) | data[1];
    v->diff = v->raw_value - v->last_raw_value;
    if (v->diff < -4096)
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if (v->diff > 4096)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - 8192;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    v->ecd_value = v->raw_value + v->round_cnt * 8192;
    v->ecd_angle_deg = (float)(v->raw_value - v->ecd_bias) * 0.04394531f + v->round_cnt * 360.0f;

    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
    if (v->buf_count == 6)
    {
        v->buf_count = 0;
    }

    for (i = 0; i < 6; i++)
    {
        temp_sum += v->rate_buf[i];
    }

    v->filter_rate_rpm = (int16_t)((data[2] << 8) | data[3]);
    v->Speed_dps = (int)((data[2] << 8) | data[3]) * 6;
}

float Float_Limit(float x, float max)
{
    if (x > max) return max;
    if (x < -max) return -max;
    return x;
}
