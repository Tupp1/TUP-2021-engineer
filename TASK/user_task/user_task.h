#ifndef USER_TASK_H
#define USER_TASK_H

#include "remote.h"
#include "CAN_receive.h"
#include "pid.h"

typedef enum
{
	OPEN  = 0,
	CLOSE = 1,
	READY_CLOSE = 2,
	INIT = 3,
}eCoverState;



typedef struct
{
	const motor_measure_t *supply_motor_measure;
	PidTypeDef supply_motor_speed_pid;         //电机速度PID
	PidTypeDef supply_motor_position_pid;         //电机位置PID
	
	eCoverState cover_state;
	
	fp32 speed;
	fp32 speed_set;
	fp32 angle;
	fp32 set_angle;
	fp32 set_ramp_angle;
	fp32 stir_buff_ramp;
	int64_t sum_ecd;	
	fp32 revolver_buff_ramp;
  int16_t given_current;
	uint16_t init_time;
	uint16_t key_time;
	int32_t open_angle;
	int32_t close_angle;
	int32_t init_angle_add;
} Supply_Control_t;


#define SUPPLY 0
#define RESCUE 1



typedef struct
{
	  const RC_ctrl_t *user_rc_ctrl;
	  RC_ctrl_time_t RC_user_ctrl_time;
	  Supply_Control_t supply_control;
	  Supply_Control_t rescue_r_control;
	  Supply_Control_t rescue_l_control;
} User_Control_t;




#define USER_TASK_INIT_TIME 360 //任务开始空闲一段时间
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define USER_X_CHANNEL 1 //前后的遥控器通道号码
#define USER_Y_CHANNEL 0 //左右的遥控器通道号码
#define USER_W_CHANNEL 3 //前后的遥控器通道号码
#define USER_RC_DEADLINE 20

#define USER_SPEED_PID_KP 800.0f
#define USER_SPEED_PID_KI 0
#define USER_SPEED_PID_KD 0
#define USER_SPEED_PID_MAX_OUT 8000.0f
#define USER_SPEED_PID_MAX_IOUT 5000.0f

#define USER_POSITION_PID_KP -0.0002f
#define USER_POSITION_PID_KI 0
#define USER_POSITION_PID_KD 0
#define USER_POSITION_PID_MAX_OUT 10.0f
#define USER_POSITION_PID_MAX_IOUT 10.0f


#define RESCUE_SPEED_PID_KP 500.0f
#define RESCUE_SPEED_PID_KI 0
#define RESCUE_SPEED_PID_KD 0
#define RESCUE_SPEED_PID_MAX_OUT 8000.0f
#define RESCUE_SPEED_PID_MAX_IOUT 5000.0f

#define RESCUE_POSITION_PID_KP -0.0002f
#define RESCUE_POSITION_PID_KI 0
#define RESCUE_POSITION_PID_KD 0
#define RESCUE_POSITION_PID_MAX_OUT 10.0f
#define RESCUE_POSITION_PID_MAX_IOUT 10.0f



#define SUPPLY_MOTOR_INIT_TIME  3000
#define RESCUE_MOTOR_INIT_TIME  3000

#define SUPPLY_MOTOR_INIT_RAMP  66.0f

extern void UserTask(void *pvParameters);

#endif
