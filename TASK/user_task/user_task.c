
#include "io.h"
#include "User_Task.h"
#include "main.h"
#include "ramp.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "UDTrans_task.h"
#include "led.h"
#include "calibrate_task.h"
#include "gimbal_behaviour.h"
#include "INS_Task.h"
#include "chassis_behaviour.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


static void SUPPLY_UpdateMotorAngleSum(Supply_Control_t *Supply_sum);
static void User_Init(User_Control_t *User_init);
static void User_Feedback_Update(User_Control_t *user_feedback);
//static void SUPPLY_control_loop(User_Control_t *user_control);
//static void RESCUE_control_loop(Supply_Control_t *user_control);
//static void RESCUEL_control_loop(Supply_Control_t *user_control);
static void User_Set_Mode(User_Control_t *user_set_mode);
static void User_control_loop(User_Control_t *user_control);
static void User_open_control(Supply_Control_t *user_control);
static void User_init_control(Supply_Control_t *user_control,u8 choose);

	
		static void User_close_control(Supply_Control_t *user_control);
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif
		
		
User_Control_t User_Control;
extern UDTrans_Control_t UDTrans_Control;


void UserTask(void *pvParameters)
{
	  portTickType currentTime;	 
    vTaskDelay(USER_TASK_INIT_TIME);
    User_Init(&User_Control);
    while (1) 
    {
			currentTime = xTaskGetTickCount();//当前系统时间
			User_Set_Mode(&User_Control);
			User_Feedback_Update(&User_Control);
			User_control_loop(&User_Control);
//			CAN2_CMD_200(User_Control.supply_control.given_current,UDTrans_Control.upmotor_b.given_current,
//								 UDTrans_Control.upmotor_f.given_current,UDTrans_Control.transmotor.given_current);
//			CAN1_CMD_1FF(User_Control.rescue_l_control.given_current,User_Control.rescue_r_control.given_current,0,0);
			vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//绝对延时
			
#if INCLUDE_uxTaskGetStackHighWaterMark
      UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


static void User_Set_Mode(User_Control_t *user_set_mode)
{
	 static uint16_t cover_mode_change_time;
	 if(user_set_mode->user_rc_ctrl->rc.ch[0] < -RC_CALI_VALUE_HOLE &&  \
			user_set_mode->user_rc_ctrl->rc.ch[1] > RC_CALI_VALUE_HOLE &&  \
			user_set_mode->user_rc_ctrl->rc.ch[2] > RC_CALI_VALUE_HOLE && \
			user_set_mode->user_rc_ctrl->rc.ch[3] > RC_CALI_VALUE_HOLE &&  \
			switch_is_down(user_set_mode->user_rc_ctrl->rc.s[0]) &&        \
			switch_is_down(user_set_mode->user_rc_ctrl->rc.s[1])  )
		 {
				//上内八
				cover_mode_change_time++;
		 }
		if(cover_mode_change_time==2000)
		{
				led_red_toggle();
				cover_mode_change_time = 0;
				user_set_mode->supply_control.cover_state = INIT;
			  user_set_mode->rescue_r_control.cover_state = INIT;
			  user_set_mode->rescue_l_control.cover_state = INIT;
		}
		
		//键盘设置行为模式
		//B键补给
		if (user_set_mode->user_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
    {
			if (user_set_mode->RC_user_ctrl_time.rc_keyB_time < RC_KEY_LONG_TIME)
			{
				 user_set_mode->RC_user_ctrl_time.rc_keyB_time++;
				 if(user_set_mode->RC_user_ctrl_time.rc_keyB_time==RC_KEY_LONG_TIME)
				 {
						user_set_mode->supply_control.cover_state = OPEN;
					  user_set_mode->RC_user_ctrl_time.rc_keyB_time = 0;
				 }
			}
		}
		else
		{
			  user_set_mode->RC_user_ctrl_time.rc_keyB_time = 0;
		}
		
		//C键救援
		 if (user_set_mode->user_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C)
    {
			if (user_set_mode->RC_user_ctrl_time.rc_keyC_time < RC_KEY_LONG_TIME)
			{
				user_set_mode->RC_user_ctrl_time.rc_keyC_time++;
				 if(user_set_mode->RC_user_ctrl_time.rc_keyC_time==RC_KEY_LONG_TIME)
				 {
				  	user_set_mode->rescue_r_control.cover_state = CLOSE;
			      user_set_mode->rescue_l_control.cover_state = CLOSE;
					  user_set_mode->RC_user_ctrl_time.rc_keyC_time = 0;
				 }
			}
		}
		else
		{
			user_set_mode->RC_user_ctrl_time.rc_keyC_time=0;
		}
		
		//Ctrl键，子模式恢复正常
		if(user_set_mode->user_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)
		{
			if (user_set_mode->RC_user_ctrl_time.rc_keyCtrl_time < RC_KEY_LONG_TIME)
			{
				 user_set_mode->RC_user_ctrl_time.rc_keyCtrl_time++;
				 if(user_set_mode->RC_user_ctrl_time.rc_keyCtrl_time==RC_KEY_LONG_TIME)
				 {
				   	user_set_mode->supply_control.cover_state = CLOSE;
			      user_set_mode->rescue_r_control.cover_state = OPEN;
			      user_set_mode->rescue_l_control.cover_state = OPEN;
					 	user_set_mode->RC_user_ctrl_time.rc_keyCtrl_time = 0;
				 }
			}
		}
		else
		{
			user_set_mode->RC_user_ctrl_time.rc_keyCtrl_time=0;
		}
		
}


static void User_Init(User_Control_t *User_init)
{
	 //播轮电机的pid没有必要使用上位机调参
	 const fp32 User_speed_pid[3] = {USER_SPEED_PID_KP, USER_SPEED_PID_KI, USER_SPEED_PID_KD};
	 const fp32 User_position_pid[3] = {USER_POSITION_PID_KP, USER_POSITION_PID_KI, USER_POSITION_PID_KD};
	 
	 //播轮电机的pid没有必要使用上位机调参
	 const fp32 Rescue_speed_pid[3] = {RESCUE_SPEED_PID_KP, RESCUE_SPEED_PID_KI, RESCUE_SPEED_PID_KD};
	 const fp32 Rescue_position_pid[3] = {RESCUE_POSITION_PID_KP, RESCUE_POSITION_PID_KI, RESCUE_POSITION_PID_KD};
	 
	 //初始化PID
	 PID_Init(&(User_init->supply_control.supply_motor_speed_pid), PID_POSITION, User_speed_pid, 
	            USER_SPEED_PID_MAX_OUT, USER_SPEED_PID_MAX_IOUT);
	 PID_Init(&(User_init->supply_control.supply_motor_position_pid), PID_POSITION, User_position_pid, 
	            USER_POSITION_PID_MAX_OUT, USER_POSITION_PID_MAX_IOUT);
	 
	 //初始化PID 救援2006电机
	 PID_Init(&(User_init->rescue_l_control.supply_motor_speed_pid), PID_POSITION, Rescue_speed_pid, 
	            RESCUE_SPEED_PID_MAX_OUT, RESCUE_SPEED_PID_MAX_IOUT);
	 PID_Init(&(User_init->rescue_l_control.supply_motor_position_pid), PID_POSITION, Rescue_position_pid, 
	            RESCUE_POSITION_PID_MAX_OUT,RESCUE_POSITION_PID_MAX_IOUT);
   PID_Init(&(User_init->rescue_r_control.supply_motor_speed_pid), PID_POSITION, Rescue_speed_pid, 
	            RESCUE_SPEED_PID_MAX_OUT, RESCUE_SPEED_PID_MAX_IOUT);
	 PID_Init(&(User_init->rescue_r_control.supply_motor_position_pid), PID_POSITION, Rescue_position_pid, 
	            RESCUE_POSITION_PID_MAX_OUT,RESCUE_POSITION_PID_MAX_IOUT);	
	 
   //遥控器指针
   User_init->user_rc_ctrl=get_remote_control_point();
	 
	 //弹仓盖电机指针
   User_init->supply_control.cover_state=INIT;
	 User_init->supply_control.open_angle = -123535;
	 User_init->supply_control.close_angle = 0;
	 User_init->supply_control.init_angle_add = 300;
	 User_init->supply_control.set_angle = 0;
	 User_init->supply_control.angle =0;
	 User_init->supply_control.stir_buff_ramp = 300;
	 User_init->supply_control.init_time = 0;
	 //救援电机指针
   User_init->rescue_r_control.cover_state=INIT;
	 User_init->rescue_r_control.open_angle = 0;
	 User_init->rescue_r_control.close_angle = -92571;
	 User_init->rescue_r_control.set_angle = 0;
	 User_init->rescue_r_control.init_angle_add = 300;
	 User_init->rescue_r_control.angle =0;
	 User_init->rescue_r_control.stir_buff_ramp = 300;
	 User_init->rescue_r_control.init_time = 0;
	 
   User_init->rescue_l_control.cover_state=INIT;
	 User_init->rescue_l_control.open_angle = 0;
	 User_init->rescue_l_control.close_angle = 92571;
	 User_init->rescue_l_control.init_angle_add = -300;
	 User_init->rescue_l_control.set_angle = 0;
	 User_init->rescue_l_control.angle =0;
	 User_init->rescue_l_control.stir_buff_ramp = 300;
	 User_init->rescue_l_control.init_time = 0;
	 
	 User_Feedback_Update(User_init);
}

static void User_control_loop(User_Control_t *user_control)
{
	if(user_control->rescue_r_control.cover_state == INIT) User_init_control(&user_control->rescue_r_control,RESCUE);
	if(user_control->rescue_l_control.cover_state == INIT) User_init_control(&user_control->rescue_l_control,RESCUE);
	if(user_control->supply_control.cover_state == INIT) User_init_control(&user_control->supply_control,SUPPLY);
	
	if(user_control->rescue_r_control.cover_state == OPEN) User_open_control(&user_control->rescue_r_control);
	if(user_control->rescue_l_control.cover_state == OPEN) User_open_control(&user_control->rescue_l_control);
	if(user_control->supply_control.cover_state == OPEN) User_open_control(&user_control->supply_control);
	
	if(user_control->rescue_r_control.cover_state == 	CLOSE) User_close_control(&user_control->rescue_r_control);
	if(user_control->rescue_l_control.cover_state == CLOSE) User_close_control(&user_control->rescue_l_control);
	if(user_control->supply_control.cover_state == CLOSE) User_close_control(&user_control->supply_control);

}


static void User_init_control(Supply_Control_t *user_control,u8 choose)
{
	fp32 angle_out;
	if(fabs(user_control->set_angle-user_control->angle)>20000) user_control->init_time++;
	else 
	{
		user_control->set_angle += user_control->init_angle_add;
	}
	if(user_control->init_time == 1000)
	{
		user_control->set_angle=0.0f;
		user_control->set_ramp_angle = 0.0f;
		user_control->sum_ecd=0.0f;
		if(choose==SUPPLY) 	user_control->cover_state = CLOSE;
		else if(choose == RESCUE) user_control->cover_state = OPEN;
		user_control->init_time=0;
	}
	angle_out = PID_Calc(&user_control->supply_motor_position_pid, user_control->set_angle, user_control->angle);
	user_control->given_current = PID_Calc(&user_control->supply_motor_speed_pid, user_control->speed, angle_out);
}

static void User_open_control(Supply_Control_t *user_control)
{
	static fp32 angle_out;
	user_control->set_angle = user_control->open_angle;
	if(fabs(user_control->set_ramp_angle - user_control->set_angle)>1000)//缓慢转过去
	{
		user_control->set_ramp_angle = RAMP_float(user_control->set_angle, user_control->set_ramp_angle, user_control->stir_buff_ramp);
	}
	angle_out = PID_Calc(&user_control->supply_motor_position_pid, user_control->set_ramp_angle, user_control->angle);
	user_control->given_current = PID_Calc(&user_control->supply_motor_speed_pid, user_control->speed, angle_out);
	
}



static void User_close_control(Supply_Control_t *user_control)
{
	static fp32 angle_out;
	user_control->set_angle = user_control->close_angle;
	if(fabs(user_control->set_ramp_angle - user_control->set_angle)>1000)//缓慢转过去
	{
		user_control->set_ramp_angle = RAMP_float(user_control->set_angle, user_control->set_ramp_angle, user_control->stir_buff_ramp);
	}
	angle_out = PID_Calc(&user_control->supply_motor_position_pid, user_control->set_ramp_angle, user_control->angle);
	user_control->given_current = PID_Calc(&user_control->supply_motor_speed_pid, user_control->speed, angle_out);
}


static void SUPPLY_UpdateMotorAngleSum(Supply_Control_t *Supply_sum)
{		 
	//临界值判断法
	if (abs(Supply_sum->supply_motor_measure->ecd - Supply_sum->supply_motor_measure->last_ecd) > 4095)//转过半圈
	{		
		//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
		if (Supply_sum->supply_motor_measure->ecd < Supply_sum->supply_motor_measure->last_ecd)//过半圈且过零点
		{
			//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
			Supply_sum->sum_ecd += 8191 -  Supply_sum->supply_motor_measure->last_ecd + Supply_sum->supply_motor_measure->ecd;
		}
		else
		{
			//超过了一圈
				Supply_sum->sum_ecd -= 8191 - Supply_sum->supply_motor_measure->ecd + Supply_sum->supply_motor_measure->last_ecd;
		}
	}
	else      
	{
		//未过临界值,累加上转过的角度差
			 Supply_sum->sum_ecd += Supply_sum->supply_motor_measure->ecd  -  Supply_sum->supply_motor_measure->last_ecd;
	}
	
}


/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void User_Feedback_Update(User_Control_t *user_feedback)
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + ( user_feedback->supply_control.supply_motor_measure->speed_rpm* Motor_RMP_TO_SPEED) * fliter_num[2];
    user_feedback->supply_control.speed = speed_fliter_3;
		
		speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + ( user_feedback->rescue_r_control.supply_motor_measure->speed_rpm* Motor_RMP_TO_SPEED) * fliter_num[2];
    user_feedback->rescue_r_control.speed = speed_fliter_3;
		
		speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + ( user_feedback->rescue_l_control.supply_motor_measure->speed_rpm* Motor_RMP_TO_SPEED) * fliter_num[2];
    user_feedback->rescue_l_control.speed = speed_fliter_3;

    SUPPLY_UpdateMotorAngleSum(&(user_feedback->supply_control));
		SUPPLY_UpdateMotorAngleSum(&(user_feedback->rescue_l_control));
		SUPPLY_UpdateMotorAngleSum(&(user_feedback->rescue_r_control));

    //计算输出轴角度
    user_feedback->supply_control.angle = user_feedback->supply_control.sum_ecd;
		user_feedback->rescue_l_control.angle = user_feedback->rescue_l_control.sum_ecd;
		user_feedback->rescue_r_control.angle = user_feedback->rescue_r_control.sum_ecd;
		
//		//救援2006电机
//		if(rescue_l_monitor()==TRUE) user_feedback->rescue_l_control.key_time++;
//		else user_feedback->rescue_l_control.key_time=0;
//		if(user_feedback->rescue_l_control.key_time==200) user_feedback->rescue_l_control.cover_state = OPEN;

//		if(rescue_r_monitor()==TRUE) user_feedback->rescue_r_control.key_time++;
//		else user_feedback->rescue_r_control.key_time=0;
//		if(user_feedback->rescue_r_control.key_time==200) user_feedback->rescue_r_control.cover_state = OPEN;

}
