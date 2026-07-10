#include "CH040_USART3.h"
#include <string.h>
#include "Chassis_Task.h"
#include "main.h"

extern UART_HandleTypeDef huart3;

uint8_t      CH040_Rx_Buffer[CH040_RX_BUFF_SIZE];
CH040_data_t imu_data;
CH040DATA_t  CH040DATA;

static uint8_t frame_buf[CH040_DATA_FRAME_LENGTH];
static uint8_t frame_rdy;

void USART3_Init(uint32_t baud_rate)
{
    huart3.Init.BaudRate = baud_rate;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, CH040_Rx_Buffer, CH040_RX_BUFF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void CH040_UART_RxCallback(uint16_t Size)
{
    if (Size == CH040_DATA_FRAME_LENGTH)
    {
        memcpy(frame_buf, CH040_Rx_Buffer, CH040_DATA_FRAME_LENGTH);
        frame_rdy = 1;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, CH040_Rx_Buffer, CH040_RX_BUFF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

static void CH040_Frame_Parse(uint8_t* buf, CH040_data_t* data)
{
    uint8_t* p        = buf + CH040_FRAME_HEADER_LENGTH;
    data->tag         = p[0];
    data->main_status = (uint16_t) p[1] | ((uint16_t) p[2] << 8);
    data->temperature = (int8_t) p[3];
    memcpy(&data->pressure, p + 4, 4);
    memcpy(&data->timestamp, p + 8, 4);
    memcpy(data->acc, p + 12, 12);
    memcpy(data->gyr, p + 24, 12);
    memcpy(data->mag, p + 36, 12);
    memcpy(data->eul, p + 48, 12);
    memcpy(data->quat, p + 60, 16);
}

float   Last_Yaw_Angle = 0, Diff = 0;
int16_t Yaw_Circle_Count = 0;

void CH040_Data_Get(CH040_data_t* imu_data, CH040DATA_t* CH040DATA)
{
    if (frame_rdy)
    {
        CH040_Frame_Parse(frame_buf, imu_data);
        frame_rdy = 0;
    }

    CH040DATA->Pitch_Angle = imu_data->eul[0];
    CH040DATA->Roll_Angle  = imu_data->eul[1];
    CH040DATA->Yaw_Angle   = imu_data->eul[2];

    CH040DATA->X_Acc = imu_data->acc[0];
    CH040DATA->Y_Acc = imu_data->acc[1];
    CH040DATA->Z_Acc = imu_data->acc[2];

    CH040DATA->Pitch_Gyro_Omega = imu_data->gyr[0];
    CH040DATA->Roll_Gyro_Omega  = imu_data->gyr[1];
    CH040DATA->Yaw_Gyro_Omega   = imu_data->gyr[2];

    Diff = CH040DATA->Yaw_Angle - Last_Yaw_Angle;
    if (Diff < -180.0f)
        Yaw_Circle_Count++;
    else if (Diff > 180.0f)
        Yaw_Circle_Count--;

    CH040DATA->Yaw_Multi_Angle
        = CH040DATA->Yaw_Angle + Yaw_Circle_Count * 360.0f;
    Last_Yaw_Angle = CH040DATA->Yaw_Angle;

    g_gyro.angle          = CH040DATA->Yaw_Multi_Angle;
    g_gyro.omega          = CH040DATA->Yaw_Gyro_Omega;
    g_gyro.angle_rel_base = g_gyro.angle - g_gyro.angle_base;
}
