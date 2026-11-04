#include "main.h"

uint8_t Motor_Online_Flag[4]={0};

void Encoder_Process(Encoder* Motor_Encoder, uint8_t* data)
{
    Motor_Encoder->last_raw_value = Motor_Encoder->raw_value;
    Motor_Encoder->current = (data[4]<<8)|data[5];
    Motor_Encoder->raw_value = (data[0]<<8)|data[1];
    Motor_Encoder->diff = Motor_Encoder->raw_value - Motor_Encoder->last_raw_value;
    if(Motor_Encoder->diff < -4096)
    {
        Motor_Encoder->round_cnt++;
    }
    else if(Motor_Encoder->diff > 4096)
    {
        Motor_Encoder->round_cnt--;
    }
    Motor_Encoder->ecd_value = Motor_Encoder->raw_value + Motor_Encoder->round_cnt * 8192;
    Motor_Encoder->Omega = (int)(int16_t)((data[2]<<8)|data[3]);
    Motor_Encoder->temperature = data[6];
}

void CAN_Motor_Data_Receive_Ex(uint32_t StdId, uint8_t* data, Encoder* Yaw_Motor_Encoder, Encoder* Pitch_Motor_Encoder, Encoder* Chassis_M3508_Encoder)
{
    switch(StdId)
    {
        case M2006_M0_FEEDBACK_ID:
            Motor_Online_Flag[0] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[0],data);
            break;
        case M2006_M1_FEEDBACK_ID:
            Motor_Online_Flag[1] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[1],data);
            break;
        case M2006_M2_FEEDBACK_ID:
            Motor_Online_Flag[2] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[2],data);
            break;
        case M2006_M3_FEEDBACK_ID:
            Motor_Online_Flag[3] = 1;
            Encoder_Process(&Chassis_M3508_Encoder[3],data);
            break;
        case GM6020_YAW_FEEDBACK_ID:
            Encoder_Process(Yaw_Motor_Encoder,data);
            break;
        case GM6020_PITCH_FEEDBACK_ID:
            Encoder_Process(Pitch_Motor_Encoder,data);
            break;
        case M2006_POKE_FEEDBACK_ID:
        default :
            break;
    }
}

void CAN_Send_Task(int16_t* Gimbal_Current_Set,int16_t* Chassis_Current_Set)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.StdId = GM6020_CONTROL_ID;
    tx_header.ExtId = 0;
    tx_header.TransmitGlobalTime = DISABLE;
    tx_data[0] = (Gimbal_Current_Set[0] >> 8);
    tx_data[1] = Gimbal_Current_Set[0];
    tx_data[2] = (Gimbal_Current_Set[1] >> 8);
    tx_data[3] = Gimbal_Current_Set[1];
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

    tx_header.StdId = CHASSIS_M2006_CONTROL_ID;
    tx_data[0] = (Chassis_Current_Set[0]>>8);
    tx_data[1] = Chassis_Current_Set[0];
    tx_data[2] = (Chassis_Current_Set[1]>>8);
    tx_data[3] = Chassis_Current_Set[1];
    tx_data[4] = (Chassis_Current_Set[2]>>8);
    tx_data[5] = Chassis_Current_Set[2];
    tx_data[6] = (Chassis_Current_Set[3]>>8);
    tx_data[7] = Chassis_Current_Set[3];
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}
