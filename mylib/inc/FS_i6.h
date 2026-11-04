#ifndef __FS_I6_H
#define __FS_I6_H
#include "stm32f4xx_hal.h"

#define FS_FRAME_LENGTH 25u

typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t swb;
    int16_t swa;
    int16_t swc;
    int16_t swd;
    int16_t vra;
    int16_t vrb;
} Remote_t;

extern Remote_t Remote_Data;

void FS_Data_Get(uint8_t *buf);

#endif
