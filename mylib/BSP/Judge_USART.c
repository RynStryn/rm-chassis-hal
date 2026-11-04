#include "Judge_USART.h"
#include "Fault_Diagnosis_Task.h"
#include "JudgeSys.h"
#include "main.h"

extern UART_HandleTypeDef huart5;

uint8_t Judge_Rx_Buffer[JUDGE_RX_BUFFER_SIZE];

void Judge_USART_Init(uint32_t baud_rate)
{
    huart5.Init.BaudRate = baud_rate;

    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Judge_Rx_Buffer, JUDGE_RX_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void Judge_UART_RxCallback(uint16_t Size)
{
    judgement_data_handle(Judge_Rx_Buffer, Size);

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Judge_Rx_Buffer, JUDGE_RX_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}
