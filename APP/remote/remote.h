#ifndef __REMOTE_H_
#define __REMOTE_H_

#include "system.h"
#include "main.h" 
//取值0或1  用于标志位
//typedef   signed          char    bool_t;


/* 遥控器通信协议 
   当接收机和发射机建立连接后，接收机会每隔 7ms 通过DBUS发送一帧
   数据（18 字节） */
#define    REMOTE_DBUS_FRAME_LEN    18



/* ****************************(C) COPYRIGHT 2016 DJI*****************************/


#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
#define RC_CH_VALUE_MIN ((int16_t)(-660))
#define RC_CH_VALUE_OFFSET ((int16_t)1024)
#define RC_CH_VALUE_MAX ((int16_t)660)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
					      uint8_t press_r_last;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

typedef __packed struct
{
	uint16_t rc_keyQ_time;  //按键Q的时间
	uint16_t rc_keyE_time;  //按键E的时间
	uint16_t rc_keyR_time;  //按键R的时间
	uint16_t rc_keyF_time;  //按键F的时间
	uint16_t rc_keyG_time;  //按键G的时间
	uint16_t rc_keyZ_time;  //按键Z的时间
	uint16_t rc_keyX_time;  //按键X的时间
	uint16_t rc_keyC_time;  //按键C的时间
	uint16_t rc_keyV_time;  //按键V的时间
	uint16_t rc_keyB_time;  //按键B的时间
	uint16_t rc_keyShift_time; //按键Shift的时间
	uint16_t rc_keyCtrl_time;  //按键Ctrl的时间
	uint16_t mouse_left_time;
	uint16_t mouse_right_time;
	
	uint16_t rc_keyQ_E_time;      //同时按QE的时间
	uint16_t rc_keyShift_E_time;  //同时按Shift和E的时间
}RC_ctrl_time_t;
/* ----------------------- Internal Data ----------------------------------- */





/* 获取遥控器摇杆偏移量
   根据遥控器文档：


左摇杆：    右摇杆：
左右为ch2   左右为ch0
上下为ch3   上下为ch1

                         上 1684           
左    中     右       
364   1024   1684        中 1024

                         下  364  */


/* 获取遥控器摇杆偏移值 
   RLR：右摇杆左右移动  LUD：左摇杆上下移动	*/
#define    RC_CH0_RLR_OFFSET    (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET)
#define    RC_CH1_RUD_OFFSET  	(RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET)
#define    RC_CH2_LLR_OFFSET  	(RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET)
#define    RC_CH3_LUD_OFFSET  	(RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET)


/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (RC_Ctl.rc.s1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (RC_Ctl.rc.s1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (RC_Ctl.rc.s1 == RC_SW_DOWN)

#define    IF_RC_SW2_UP      (RC_Ctl.rc.s2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (RC_Ctl.rc.s2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (RC_Ctl.rc.s2 == RC_SW_DOWN)


/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (RC_Ctl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (RC_Ctl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (RC_Ctl.mouse.z)


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (RC_Ctl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (RC_Ctl.mouse.press_r == 1)


/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  RC_Ctl.key.v  )
#define    IF_KEY_PRESSED_W       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )




void REMOTE_vResetData( void );
void REMOTE_vUpdateLostTime( void );
void REMOTE_vReadData(char *pucRxBuffer );
bool_t REMOTE_IfDataError( void );
bool_t REMOTE_IfKeyReset( void );

uint32_t REMOTE_ulGetLostTime( void );
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctl);
void remote_control_init(void);
const RC_ctrl_t *get_remote_control_point(void);

extern RC_ctrl_t RC_Ctl;

#endif
