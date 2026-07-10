#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H
#include <stm32f4xx.h>


/**********************************宏定义*************************************/
#define         MIDDLE                          3
#define         UP                              1
#define         DOWN                            2
#define         RC_CH_VALUE_OFFSET              1024
#define         LONG_PRESS_THRESHOLD            500
#define         RC_DEAD_ZONE                    10
/**********************************宏定义*************************************/



/*********************************DT7接收机************************************/
typedef enum
{
	KEEP =0            ,
	UP_TO_MIDDLE       ,
	MIDDLE_TO_UP       ,
	MIDDLE_TO_DOWN     ,
	DOWN_TO_MIDDLE     ,
}Clicker_Action_e;//拨杆的动作


typedef struct __attribute__((packed))
{
	int tick;//计数，用来判断长短按
	uint8_t  Raw_Short_Press;//原始短按标志位,只用来判断按键状态
    uint8_t  Short_Press;//短按标志位
    uint8_t  Long_Press;//长按标志位
    uint8_t  last_Raw_Short_Press;//上一次的原始短按标志位，来检测上升下降沿去判断有没有重新按下，用来判断是否反转按键状态
    uint8_t  Toggle_Press;//按键状态反转标志位
}Press_Action_t;//鼠标键盘长按短按动作

typedef struct __attribute__((packed))
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
	}Stick_t;
typedef struct __attribute__((packed))
{
	int16_t ch4;//拨轮
	int8_t s1;
	int8_t s2;
	/**********Mine**********/
	uint8_t s1_last;
	uint8_t s2_last;
	Clicker_Action_e s1_Action;
	Clicker_Action_e s2_Action;
    uint8_t Trigger_Up;
    uint8_t Trigger_Down;
    Press_Action_t Trigger_Up_Action;
    Press_Action_t Trigger_Down_Action;

}toggle_t;//遥控器相关


typedef struct __attribute__((packed))
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
	Press_Action_t Press_L_Action;
	Press_Action_t Press_R_Action;
}Mouse_t;//鼠标相关

typedef struct __attribute__((packed))
{
	uint16_t v;
	uint16_t last_v;
	Press_Action_t Key_W_Action;
	Press_Action_t Key_S_Action;
	Press_Action_t Key_A_Action;
	Press_Action_t Key_D_Action;
	Press_Action_t Key_Q_Action;
	Press_Action_t Key_E_Action;
	Press_Action_t Key_R_Action;
	Press_Action_t Key_F_Action;
	Press_Action_t Key_G_Action;
	Press_Action_t Key_Z_Action;
	Press_Action_t Key_X_Action;
	Press_Action_t Key_C_Action;
	Press_Action_t Key_V_Action;
	Press_Action_t Key_B_Action;
	Press_Action_t Key_SHIFT_Action;
	Press_Action_t Key_CTRL_Action;
}Key_t;//键盘相关

typedef struct __attribute__((packed))
{
	Stick_t stick;
	toggle_t toggle_SW;
	Mouse_t Remote_mouse;
	Key_t key;
}RC_DT7_Data_t;//总

typedef enum
{
	RELAX = 0,
	CONTORLLER,
	KEY_MOUSE,
	OFFLINE
}RC_Mode_e;//控制模式切换

/*********************************DT7接收机************************************/



/********************************外部声明*************************************/
extern RC_DT7_Data_t g_RC_data;
extern RC_Mode_e RC_Mode;
extern float yaw_ang_inc;
/********************************外部声明*************************************/



/********************************函数声明*************************************/
void DT7_RC_Data_Dispose(uint8_t* raw_Data,RC_DT7_Data_t* RC_data);
void Key_Mouse_Action_Judge(Press_Action_t* Key_Mouse_Action);
void Key_Mouse_State_Update(RC_DT7_Data_t* Remote_data);
void RC_Mode_Select(RC_Mode_e* RC_Mode);
void RC_Toggle_Action_Detect(RC_DT7_Data_t* RC_data);
void RC_Cmd_Interpreter(void);
/********************************函数声明*************************************/



typedef struct {

        float left_x;
        float left_y;
        float right_x;
        float right_y;


        uint8_t s1;

} cmd_t;

extern cmd_t g_cmd;
#endif


