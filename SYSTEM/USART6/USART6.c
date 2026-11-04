#include "main.h"

uint8_t _USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
uint8_t USART6_Tx_Buf[USART6_TX_BUF_LENGTH];

void USART6_Init(uint32_t baud_rate)
{
    huart6.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }

    DMA2_Stream1->NDTR = BSP_USART6_DMA_RX_BUF_LEN;
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

void USART6_DMA_Tx_Enable(uint16_t data_len)
{
    if(data_len > USART6_TX_BUF_LENGTH)
    {
        data_len = USART6_TX_BUF_LENGTH;
    }

    while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }
    hdma_usart6_tx.Instance->NDTR = data_len;
    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

void USART6_Send_Data(uint8_t *data, uint16_t len)
{
    if(len > USART6_TX_BUF_LENGTH)
    {
        len = USART6_TX_BUF_LENGTH;
    }

    memcpy(USART6_Tx_Buf, data, len);
    USART6_DMA_Tx_Enable(len);
}
