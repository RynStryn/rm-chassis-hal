#ifndef __JUDGE_USART_H
#define __JUDGE_USART_H
#include "stm32f4xx_hal.h"

#define JUDGE_RX_BUFFER_SIZE 256

extern uint8_t Judge_Rx_Buffer[JUDGE_RX_BUFFER_SIZE];

void Judge_USART_Init(uint32_t baud_rate);
void Judge_UART_RxCallback(uint16_t Size);

#endif
