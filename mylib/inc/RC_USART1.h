#ifndef __RC_USART1_H
#define __RC_USART1_H
#include "stm32f4xx_hal.h"

#define  BSP_USART1_DMA_RX_BUF_LEN             100
#define  RC_DATA_FRAME_LENGTH                  25//18-25

extern uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];

void USART1_Init(uint32_t baud_rate);
void RC_UART_RxCallback(uint16_t Size);

#endif
