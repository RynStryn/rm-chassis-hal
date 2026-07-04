#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "Chassis_Task.h"
#include "stm32f4xx_hal.h"

#define KEY_PRESSED_OFFSET_W     ((uint32_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S     ((uint32_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A     ((uint32_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D     ((uint32_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint32_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint32_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint32_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_E     ((uint32_t)0x01 << 7)
#define KEY_PRESSED_OFFSET_R     ((uint32_t)0x01 << 8)
#define KEY_PRESSED_OFFSET_F     ((uint32_t)0x01 << 9)
#define KEY_PRESSED_OFFSET_G     ((uint32_t)0x01 << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint32_t)0x01 << 11)
#define KEY_PRESSED_OFFSET_X     ((uint32_t)0x01 << 12)
#define KEY_PRESSED_OFFSET_C     ((uint32_t)0x01 << 13)
#define KEY_PRESSED_OFFSET_V     ((uint32_t)0x01 << 14)
#define KEY_PRESSED_OFFSET_B     ((uint32_t)0x01 << 15)

#define RC_FRAME_LENGTH         18u
#define DBUS_RX_BUF_SIZE        (RC_FRAME_LENGTH * 2)

typedef enum {
    Keep = 0,
    L_2_M,
    M_2_L,
    M_2_H,
    H_2_M,
} Clicker_Action_e;

typedef enum {
    ZERO_INPUT,
    CLICK_FAST,
    CLICK_LONG_TIME,
    TAP_AND_HOLD,
} Key_Mouse_Action_e;

typedef struct {
    int                Cnt;
    int                Hold_Cnt;
    uint8_t            Flag;
    uint8_t            Flag_Last;
    Key_Mouse_Action_e Key_Mouse_Action;
} Key_Mouse_Action_t;

typedef struct {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int8_t  s1;
    int8_t  s2;
    uint8_t s1_last;
    uint8_t s2_last;
    Clicker_Action_e s1_Action;
    Clicker_Action_e s2_Action;
    uint8_t Trigger_H;
    uint8_t Trigger_L;
} Clicker_t;

typedef struct {
    int16_t            x;
    int16_t            y;
    int16_t            z;
    uint8_t            last_press_l;
    uint8_t            last_press_r;
    uint8_t            press_l;
    uint8_t            press_r;
    float              Mouse_W;
    Key_Mouse_Action_t Press_L_Action;
    Key_Mouse_Action_t Press_R_Action;
} Mouse_t;

typedef struct {
    uint16_t           v;
    uint16_t           last_v;
    float              Key_V_Y;
    float              Key_V_X;
    Key_Mouse_Action_t Key_W_Action;
    Key_Mouse_Action_t Key_S_Action;
    Key_Mouse_Action_t Key_A_Action;
    Key_Mouse_Action_t Key_D_Action;
    Key_Mouse_Action_t Key_Q_Action;
    Key_Mouse_Action_t Key_E_Action;
    Key_Mouse_Action_t Key_R_Action;
    Key_Mouse_Action_t Key_F_Action;
    Key_Mouse_Action_t Key_G_Action;
    Key_Mouse_Action_t Key_Z_Action;
    Key_Mouse_Action_t Key_X_Action;
    Key_Mouse_Action_t Key_C_Action;
    Key_Mouse_Action_t Key_V_Action;
    Key_Mouse_Action_t Key_B_Action;
    Key_Mouse_Action_t Key_SHIFT_Action;
    Key_Mouse_Action_t Key_CTRL_Action;
} Key_t;

typedef struct {
    Clicker_t Remote_clicker;
    Mouse_t   Remote_mouse;
    Key_t     key;
} Remote;

typedef enum {
    NUll = 0,
    REMOTE,
    KEY_MOUSE,
} Control_Mode_e;

extern Remote         Remote_data;
extern Control_Mode_e RemoteCtrl_Mode;
extern uint8_t        usart1_rx_buf[DBUS_RX_BUF_SIZE];

void RemoteDataDispose(uint8_t* pData);
void Remote_Switch_Action_Get(Clicker_t* pRemote);
void Key_Mouse_Action_Get(Mouse_t* pMouse, Key_t* pKey);
void Remote_Ctrl_Chassis(void);
void Remote_Init(void);

#endif
