#include "../inc/Can_Motor.h"
#include "Chassis_Task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t wheel_online[4] = {0};

Encoder_t Motor_Encoder[4];

void Encoder_Process(Encoder_t* Motor_Encoder, uint8_t* data)
{
    Motor_Encoder->last_raw_value = Motor_Encoder->raw_value;
    Motor_Encoder->current = (data[4] << 8) | data[5];
    Motor_Encoder->raw_value = (data[0] << 8) | data[1];
    Motor_Encoder->diff = Motor_Encoder->raw_value - Motor_Encoder->last_raw_value;

    // M3508 电机过零点处理，对应 8192 线分辨率的一半
    if (Motor_Encoder->diff < -4096)
    {
        Motor_Encoder->round_cnt++;
    }
    else if (Motor_Encoder->diff > 4096)
    {
        Motor_Encoder->round_cnt--;
    }

    Motor_Encoder->ecd_value = Motor_Encoder->raw_value + Motor_Encoder->round_cnt * 8192;
    Motor_Encoder->Omega_rpm = (int)(int16_t)((data[2] << 8) | data[3]);
    Motor_Encoder->temperature = data[6];
}

void Can2ReceiveMsgProcess(uint32_t StdId, uint8_t* data, Encoder_t* Chassis_M3508_Encoder)
{
    switch (StdId)
    {
        case C620_FEEDBACK_ID_BASE + 1:
            wheel_online[0] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[0], data);
            break;
        case C620_FEEDBACK_ID_BASE + 2:
            wheel_online[1] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[1], data);
            break;
        case C620_FEEDBACK_ID_BASE + 3:
            wheel_online[2] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[2], data);
            break;
        case C620_FEEDBACK_ID_BASE + 4:
            wheel_online[3] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[3], data);
            break;
        default:
            break;
    }
}

void CAN_Start(void)
{
    CAN_FilterTypeDef can_filter_config = {0};
    can_filter_config.FilterBank = 0;
    can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_config.FilterIdHigh = 0x0000;
    can_filter_config.FilterIdLow = 0x0000;
    can_filter_config.FilterMaskIdHigh = 0x0000;
    can_filter_config.FilterMaskIdLow = 0x0000;
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter_config.FilterActivation = ENABLE;
    can_filter_config.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_config);

    can_filter_config.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_config);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void C620_Out_Send(CAN_HandleTypeDef* hcan, int16_t iq0, int16_t iq1, int16_t iq2, int16_t iq3)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.StdId = C620_CONTROL_ID;
    tx_header.ExtId = 0;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = (iq0 >> 8);
    tx_data[1] = iq0;
    tx_data[2] = (iq1 >> 8);
    tx_data[3] = iq1;
    tx_data[4] = (iq2 >> 8);
    tx_data[5] = iq2;
    tx_data[6] = (iq3 >> 8);
    tx_data[7] = iq3;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan->Instance == CAN2)
    {
        Can2ReceiveMsgProcess(rx_header.StdId, rx_data, Motor_Encoder);
    }
}
