#include "RC_USART1.h"
#include "FS_i6.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];

void USART1_Init(uint32_t baud_rate)
{
    huart1.Init.BaudRate = baud_rate;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}
uint16_t rc_cnt;
void RC_UART_RxCallback(uint16_t Size)
{
    rc_cnt++;
    if (Size == RC_DATA_FRAME_LENGTH)
    {
        FS_Data_Get(USART1_DMA_RX_BUF);
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}
