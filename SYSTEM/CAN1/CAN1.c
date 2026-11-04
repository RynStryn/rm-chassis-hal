#include "main.h"

static CAN_RxHeaderTypeDef rx_header_1;
static uint8_t rx_data_1[8];

void CAN1_Init(void)
{
    CAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        CAN_Motor_Data_Receive_Ex(rx_header.StdId, rx_data, &Gimbal.Yaw_Motor_Encoder, &Gimbal.Pitch_Motor_Encoder, Chassis.Chassis_M2006_Encoder);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
        CAN_Motor_Data_Receive_Ex(rx_header.StdId, rx_data, &Gimbal.Yaw_Motor_Encoder, &Gimbal.Pitch_Motor_Encoder, Chassis.Chassis_M2006_Encoder);
    }
}
