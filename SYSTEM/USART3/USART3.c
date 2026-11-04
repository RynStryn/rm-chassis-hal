#include "main.h"

uint8_t CH040_Rx_Buffer[CH040_RX_BUFF_SIZE];
imu_data_t imu_data;
CH040DATA_t CH040DATA;

void USART3_Init(uint32_t baud_rate)
{
    huart3.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }

    DMA1_Stream1->NDTR = CH040_RX_BUFF_SIZE;
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

float Last_Yaw_Angle = 0, Diff = 0;
int16_t Yaw_Circle_Count=0;

void CH040_Data_Get(imu_data_t* imu_data , CH040DATA_t* CH040DATA)
{
    CH040DATA->Pitch_Angle = imu_data->eul[0];
    CH040DATA->Roll_Angle = imu_data->eul[1];
    CH040DATA->Yaw_Angle = imu_data->eul[2];

    CH040DATA->X_Acc = imu_data->acc[0];
    CH040DATA->Y_Acc = imu_data->acc[1];
    CH040DATA->Z_Acc = imu_data->acc[2];

    CH040DATA->Pitch_Gyro_Omega = imu_data->gyr[0];
    CH040DATA->Roll_Gyro_Omega = imu_data->gyr[1];
    CH040DATA->Yaw_Gyro_Omega = imu_data->gyr[2];

    Diff=CH040DATA->Yaw_Angle - Last_Yaw_Angle;
    if(Diff<-180.0f) Yaw_Circle_Count++;
    else if(Diff>180.0f) Yaw_Circle_Count--;

    CH040DATA->Yaw_Multi_Angle = CH040DATA->Yaw_Angle + Yaw_Circle_Count * 360.0f;
    Last_Yaw_Angle = CH040DATA->Yaw_Angle;
}
