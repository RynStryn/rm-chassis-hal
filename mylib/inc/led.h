#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx_hal.h"

#define HARD_WDG_ON   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define HARD_WDG_OFF  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)

void IWDG_Init(void);
void led_Config(void);
void led_on(void);
void led_off(void);
void Lazer_Config(void);

#endif
