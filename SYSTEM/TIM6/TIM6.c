#include "main.h"

int Timer_6;

void TIM6_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}
