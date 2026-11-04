#include "main.h"

uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];

void USART1_Init(uint32_t baud_rate)
{
    huart1.Init.BaudRate = baud_rate;
    huart1.Init.Parity = UART_PARITY_EVEN;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

    hdma_usart1_rx.Instance->M1AR = (uint32_t)&_USART1_DMA_RX_BUF[1][0];
    hdma_usart1_rx.Instance->CR |= DMA_SxCR_DBM;
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
