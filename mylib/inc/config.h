#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f4xx_hal.h"

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define RAD_TO_ANGLE        ((float)(180.0f / 3.1415926f))
#define ANGLE_TO_RAD        ((float)(3.1415926f / 180.0f))

#define KEY_LOW_LIMIT              25
#define KEY_SHORT_LONG_BOUNDARY    500


#define CHASSIS_DIR_BIAS            45
#define CHASSIS_AUTO_ESCAPE_SWITCH  0

#define CHASSIS_RAMP_SWITCH         1
#define CHASSIS_RAMP_X_STEP_SIZE    0.07f
#define CHASSIS_RAMP_Y_STEP_SIZE    0.07f
#define CHASSIS_RAMP_W_STEP_SIZE    0.3f

#define CHASSIS_VX_LIMIT            3.0f
#define CHASSIS_VY_LIMIT            10.0f
#define CHASSIS_W_LIMIT             3.0f

#define CHASSIS_REMOTE_SENSITIVITY      0.020f
#define CHASSIS_FOLLOW_GIMBLE_SENSITIVITY 0.05f

#define CHASSIS_KW_X_SPEED              10.0f
#define CHASSIS_KW_Y_SPEED              10.0f
#define CHASSIS_KW_X_ROTATE_SPEED_PATROL  6.0f
#define CHASSIS_KW_Y_ROTATE_SPEED_PATROL  6.0f

#define CHASSIS_MOUSE_W_SENSITIVITY     0.025f
#define CHASSIS_FAST_K                  5.0f
#define CHAS_RUN_THRESHOLD              1.1f

#define WHEEL_GET_TRAP_V_THRESHOLD      0.2f
#define WHEEL_GET_TRAP_TIME_THRESHOLD   2.0f
#define CHASSIS_SPEED_UP_COEFFICIENT    1.2f

#define ROTATE_SPEED_PATROL             2.5f
#define ROTATE_SPEED_AUTO_AIM           2.0f
#define ROTATE_DIFFERENTIAL_AMPLITUDE   0.5f

#define CHASSIS_WHEEL_RADIUS        0.16219f

#endif
