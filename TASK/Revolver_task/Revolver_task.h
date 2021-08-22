#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H

#include "system.h"
#include "remote.h"
#include "can_receive.h"
#include "user_lib.h"
#include "pid.h"
#define REVOLVER_TASK_INIT_TIME 200
#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define PRESS_LONG_TIME 400
#define RevChannel 1

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

#define 	REVOL_SPEED_GRID      12			//拨盘格数
#define  	AN_BULLET         (24576.0f)		//单个子弹电机位置增加值
#define   REVOL_CAN_OPEN    350  //摩擦轮实际速度超过这个值才允许拨盘转动,根据摩擦轮最小目标速度来改变


#define REVOL_STEP0    0		//失能标志
#define REVOL_STEP1    1		//SW1复位标志
#define REVOL_STEP2    2		//弹仓开关标志


#define REVOLVER_SPEED_PID_KP 900.0f
#define REVOLVER_SPEED_PID_KI 0
#define REVOLVER_SPEED_PID_KD 0
#define REVOLVER_SPEED_PID_MAX_OUT 8000.0f
#define REVOLVER_SPEED_PID_MAX_IOUT 5000.0f

#define REVOLVER_POSITION_PID_KP 0.0008f
#define REVOLVER_POSITION_PID_KI 0
#define REVOLVER_POSITION_PID_KD 0
#define REVOLVER_POSITION_PID_MAX_OUT 10.0f
#define REVOLVER_POSITION_PID_MAX_IOUT 10.0f

//电机rmp 变化成 旋转速度的比例


typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
	REVOL_STOP_MODE =2,
}eRevolverCtrlMode;


typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;



typedef struct
{
	  const RC_ctrl_t *revolver_rc_ctrl;
	  RC_ctrl_time_t RC_revolver_ctrl_time;
    const motor_measure_t *revolver_motor_measure;
	  PidTypeDef revolver_motor_speed_pid;         //电机速度PID
	  PidTypeDef revolver_motor_position_pid;         //电机位置PID
	
	  //摩擦轮
	  ramp_function_source_t fric1_ramp;
    ramp_function_source_t fric2_ramp;
	
  	eShootAction actShoot;
	  eRevolverCtrlMode Revolver_mode;
	  eRevolverCtrlMode last_Revolver_mode;
	
	  bool_t press_l;
    bool_t last_press_l;
    uint16_t press_l_time;
	   
	  uint16_t key_ShootNum;
	
	  fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
	  fp32 set_ramp_angle;
		int64_t sum_ecd;
		
		fp32 revolver_buff_ramp;
    int16_t given_current;
   
} Revolver_Control_t;





void Revolver_task(void *pvParameters);
portTickType REVOL_uiGetRevolTime(void);








#endif
