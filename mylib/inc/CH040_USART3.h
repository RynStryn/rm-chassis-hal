#ifndef __CH040_USART_H__
#define __CH040_USART_H__

#include "stm32f4xx_hal.h"
#include "config.h"

#define CH040_RX_BUFF_SIZE              82
#define CH040_DATA_FRAME_LENGTH         82
#define CH040_FRAME_HEADER_LENGTH       6

//__packed typedef struct
typedef struct
{
    uint8_t     tag;
    uint16_t    main_status;
    int8_t      temperature;
    float       pressure;
    uint32_t    timestamp;
    float       acc[3];
    float       gyr[3];
    float       mag[3];
    float       eul[3];
    float       quat[4];
} CH040_data_t;

typedef struct
{
    float Pitch_Angle;
    float Yaw_Angle;
    float Roll_Angle;
    float Yaw_Multi_Angle;
    float Pitch_Gyro_Omega;
    float Yaw_Gyro_Omega;
    float Roll_Gyro_Omega;
    float X_Acc;
    float Y_Acc;
    float Z_Acc;
} CH040DATA_t;

extern uint8_t CH040_Rx_Buffer[82];
extern CH040_data_t imu_data;
extern CH040DATA_t CH040DATA;
extern volatile uint16_t debug_size;

void USART6_Init(uint32_t baud_rate);
void CH040_Data_Get(CH040_data_t* imu_data, CH040DATA_t* CH040DATA);
void CH040_UART_RxCallback(uint16_t Size);

#endif
