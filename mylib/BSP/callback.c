#include "main.h"
#include "CH040_USART3.h"
#include "RC_USART1.h"
#include "Judge_USART.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        RC_UART_RxCallback(Size);
    }
    else if (huart == &huart5)
    {
        Judge_UART_RxCallback(Size);
    }
    else if (huart == &huart6)
    {
        CH040_UART_RxCallback(Size);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    }
    if (htim->Instance == TIM6)
    {
        Control_Task();
    }
}