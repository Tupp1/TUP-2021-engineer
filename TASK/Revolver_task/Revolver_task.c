#include "Revolver_task.h"
#include "calibrate_task.h"
#include "control.h"
#include "remote.h"
#include "Gimbal_Task.h"
#include "ramp.h"
#include "led.h"
#include "can.h"
#include "pwm.h"

static void Revolver_control_loop(Revolver_Control_t *revolver_control);
static void Revolver_Feedback_Update(Revolver_Control_t *revolver_feedback);
static void REVOLVER_Init(Revolver_Control_t *revolver_init);
bool REVOLVER_Rc_Switch(Revolver_Control_t *revolver_control);
static void REVOL_UpdateMotorAngleSum(Revolver_Control_t *revolver_sum);
static void Revolver_Set_Mode(Revolver_Control_t *revolver_set_mode);

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

Revolver_Control_t Revolver_control;

portTickType posishoot_time;//射击延时测试

extern int16_t  Shoot_Can_Set_Current;

void Revolver_task(void *pvParameters)
{
	portTickType currentTime;	
	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	REVOLVER_Init(&Revolver_control);
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//当前系统时间
		Revolver_Set_Mode(&Revolver_control);
		Revolver_Feedback_Update(&Revolver_control);
		Revolver_control_loop(&Revolver_control);
//		CAN2_CMD_1FF(0,Shoot_Can_Set_Current,0,0);
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//绝对延时
	}
}

static void Revolver_control_loop(Revolver_Control_t *revolver_control)
{
	 static int16_t angle_out;
  
	 /*********************************模式切换处理******************************/
	 if(revolver_control->Revolver_mode!=revolver_control->last_Revolver_mode)
	 {
		 revolver_control->set_angle=revolver_control->angle;
		 revolver_control->set_ramp_angle=revolver_control->set_angle;
	 }
	 revolver_control->last_Revolver_mode = revolver_control->Revolver_mode;
	 /*********************************模式切换处理******************************/
	
	 
	 if(REVOLVER_Rc_Switch(revolver_control) == TRUE)
		{
			revolver_control->key_ShootNum++;//打一颗
		}
		if(revolver_control->key_ShootNum != 0)
		{
			revolver_control->key_ShootNum--;
			revolver_control->set_angle += AN_BULLET;
		}
		if(revolver_control->set_ramp_angle != revolver_control->set_angle)//缓慢转过去
		{
			revolver_control->set_ramp_angle = RAMP_float(revolver_control->set_angle, revolver_control->set_ramp_angle, revolver_control->revolver_buff_ramp);
		}
		
		if(revolver_control->Revolver_mode == REVOL_SPEED_MODE)
		{
			if(switch_is_up(revolver_control->revolver_rc_ctrl->rc.s[RevChannel]))
			{
				revolver_control->given_current = PID_Calc(&revolver_control->revolver_motor_speed_pid, revolver_control->speed, 6.0f);
			}
			else if(switch_is_mid(revolver_control->revolver_rc_ctrl->rc.s[RevChannel])||
				     switch_is_down(revolver_control->revolver_rc_ctrl->rc.s[RevChannel]))
			{
				revolver_control->given_current = PID_Calc(&revolver_control->revolver_motor_speed_pid, revolver_control->speed, 0.0f);
			}
		}
			
		if(revolver_control->Revolver_mode == REVOL_POSI_MODE)
		{
			 //角度环，速度环串级pid调试
			angle_out = PID_Calc(&revolver_control->revolver_motor_position_pid, revolver_control->angle, revolver_control->set_ramp_angle);
			revolver_control->given_current = PID_Calc(&revolver_control->revolver_motor_speed_pid, revolver_control->speed, angle_out);
		}	
		
		if(switch_is_down(revolver_control->revolver_rc_ctrl->rc.s[ModeChannel]))
		{
			revolver_control->given_current=0;
		}
				
		Shoot_Can_Set_Current=revolver_control->given_current;
}

static void Revolver_Set_Mode(Revolver_Control_t *revolver_set_mode)
{
    if (revolver_set_mode == NULL)
    {
        return;
    }
		static uint16_t shoot_mode_change_time,shoot_mode_change_time_mouse;
		/************************************摩擦轮控制*************************************/
		static uint16_t fric_pwm1 = Fric_OFF;
    static uint16_t fric_pwm2 = Fric_OFF;
		if(switch_is_up(revolver_set_mode->revolver_rc_ctrl->rc.s[ModeChannel])||
			 switch_is_mid(revolver_set_mode->revolver_rc_ctrl->rc.s[ModeChannel]))
		{
			if(switch_is_mid(revolver_set_mode->revolver_rc_ctrl->rc.s[RevChannel]))
			{
				 revolver_set_mode->fric1_ramp.max_value = Firc_UP;
				 revolver_set_mode->fric2_ramp.max_value = Firc_UP;
				
				 ramp_calc(&revolver_set_mode->fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

				 if( revolver_set_mode->fric1_ramp.out ==  revolver_set_mode->fric1_ramp.max_value)
				 {
						ramp_calc(& revolver_set_mode->fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
				 }
			}
			else if (switch_is_up(revolver_set_mode->revolver_rc_ctrl->rc.s[RevChannel]))
			{
				 revolver_set_mode->fric1_ramp.out = Firc_UP;
				 revolver_set_mode->fric2_ramp.out = Firc_UP;
			}
			else if (switch_is_down(revolver_set_mode->revolver_rc_ctrl->rc.s[RevChannel]))
			{
				 revolver_set_mode->fric1_ramp.out = Fric_OFF;
				 revolver_set_mode->fric2_ramp.out = Fric_OFF;
			} 
			fric_pwm1 = (uint16_t)(revolver_set_mode->fric1_ramp.out);
			fric_pwm2 = (uint16_t)(revolver_set_mode->fric2_ramp.out);
			shoot_fric1_on(fric_pwm1);
			shoot_fric2_on(fric_pwm2);
		}
		else
		{
			shoot_fric_off();
		}
		
		/************************************摩擦轮控制*************************************/
		
		/***********************************射击模式修改************************************/
		if (revolver_set_mode->revolver_rc_ctrl->rc.ch[0] < -RC_CALI_VALUE_HOLE &&  \
			  revolver_set_mode->revolver_rc_ctrl->rc.ch[1] > RC_CALI_VALUE_HOLE &&  \
		    revolver_set_mode->revolver_rc_ctrl->rc.ch[2] > RC_CALI_VALUE_HOLE && \
		    revolver_set_mode->revolver_rc_ctrl->rc.ch[3] > RC_CALI_VALUE_HOLE &&  \
		    switch_is_down(revolver_set_mode->revolver_rc_ctrl->rc.s[0]) &&        \
		    switch_is_down(revolver_set_mode->revolver_rc_ctrl->rc.s[1])  )
		    
    {
        //判断遥控器2s上外八 计时的时间， 云台使能
        shoot_mode_change_time++;
    }
		
		if(shoot_mode_change_time==2000)
		{
			led_red_toggle();
//			revolver_set_mode->Revolver_mode=!revolver_set_mode->Revolver_mode;  //后期需要改为遥控器校准改变模式
			shoot_mode_change_time=0;
		}
	 /***********************************射击模式修改************************************/
		
	 /***********************************mouse射击模式修改************************************/
		if(revolver_set_mode->revolver_rc_ctrl->mouse.press_r==1)
		{
			shoot_mode_change_time_mouse++;
			if(shoot_mode_change_time_mouse>2000)
			{
				revolver_set_mode->Revolver_mode=REVOL_SPEED_MODE;
			}
			else
			{
				revolver_set_mode->Revolver_mode=REVOL_POSI_MODE;
			}
		}
		else
		{
			shoot_mode_change_time_mouse=0;
		}
		/***********************************mouse射击模式修改************************************/
		
		if (revolver_set_mode->revolver_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
    {
			revolver_set_mode->actShoot=SHOOT_HIGHTF_LOWS;
		}
	
}

static void REVOLVER_Init(Revolver_Control_t *revolver_init)
{
	 //播轮电机的pid没有必要使用上位机调参
	 fp32 Revolver_speed_pid[3] = {REVOLVER_SPEED_PID_KP, REVOLVER_SPEED_PID_KI, REVOLVER_SPEED_PID_KD};
	 fp32 Revolver_position_pid[3] = {REVOLVER_POSITION_PID_KP, REVOLVER_POSITION_PID_KI, REVOLVER_POSITION_PID_KD};
	 //初始化PID
	 PID_Init(&(revolver_init->revolver_motor_speed_pid), PID_POSITION, Revolver_speed_pid, 
	            REVOLVER_SPEED_PID_MAX_OUT, REVOLVER_SPEED_PID_MAX_IOUT);
	 PID_Init(&(revolver_init->revolver_motor_position_pid), PID_POSITION, Revolver_position_pid, 
	            REVOLVER_POSITION_PID_MAX_OUT, REVOLVER_POSITION_PID_MAX_IOUT);
	 
	  ramp_init(&revolver_init->fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_L_UP, Fric_OFF);
    ramp_init(&revolver_init->fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_R_UP, Fric_OFF);
 
   //遥控器指针
   revolver_init->revolver_rc_ctrl=get_remote_control_point();
	 //电机指针   
	 revolver_init->actShoot=SHOOT_NORMAL;
	 revolver_init->Revolver_mode=REVOL_POSI_MODE;
	 revolver_init->revolver_buff_ramp = AN_BULLET/200;
	 Revolver_Feedback_Update(revolver_init);
}

static void REVOL_UpdateMotorAngleSum(Revolver_Control_t *revolver_sum)
{		 
	//临界值判断法
	if (abs(revolver_sum->revolver_motor_measure->ecd - revolver_sum->revolver_motor_measure->last_ecd) > 4095)//转过半圈
	{		
		//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
		if (revolver_sum->revolver_motor_measure->ecd < revolver_sum->revolver_motor_measure->last_ecd)//过半圈且过零点
		{
			//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
			revolver_sum->sum_ecd += 8191 -  revolver_sum->revolver_motor_measure->last_ecd + revolver_sum->revolver_motor_measure->ecd;
		}
		else
		{
			//超过了一圈
				revolver_sum->sum_ecd -= 8191 - revolver_sum->revolver_motor_measure->ecd + revolver_sum->revolver_motor_measure->last_ecd;
		}
	}
	else      
	{
		//未过临界值,累加上转过的角度差
			 revolver_sum->sum_ecd += revolver_sum->revolver_motor_measure->ecd  -  revolver_sum->revolver_motor_measure->last_ecd;
	}
	
}

static void Revolver_Feedback_Update(Revolver_Control_t *revolver_feedback)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (revolver_feedback->revolver_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    revolver_feedback->speed = speed_fliter_3;

    REVOL_UpdateMotorAngleSum(revolver_feedback);

    //计算输出轴角度
    revolver_feedback->angle = revolver_feedback->sum_ecd;
    //鼠标按键
    revolver_feedback->last_press_l = revolver_feedback->press_l;
    revolver_feedback->press_l = revolver_feedback->revolver_rc_ctrl->mouse.press_l;
    //长按计时
    if (revolver_feedback->press_l)
    {
        if (revolver_feedback->press_l_time < PRESS_LONG_TIME)
        {
            revolver_feedback->press_l_time++;
        }
    }
    else
    {
        revolver_feedback->press_l_time = 0;
    }

}

uint8_t	Revolver_Switch = 0;//弹仓遥控模式开关标志位转换

bool REVOLVER_Rc_Switch(Revolver_Control_t *revolver_control)
{
		if (switch_is_up(revolver_control->revolver_rc_ctrl->rc.s[RevChannel])||revolver_control->revolver_rc_ctrl->mouse.press_l==1)
		{
			if (Revolver_Switch == REVOL_STEP1)
			{
				Revolver_Switch = REVOL_STEP2;
			}
			else if (Revolver_Switch == REVOL_STEP2)
			{
				Revolver_Switch = REVOL_STEP0;
			}
		}
		else		
		{
			Revolver_Switch = REVOL_STEP1;
		}
	
		if (Revolver_Switch == REVOL_STEP2)
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}
}

/**
  * @brief  点射时间获取
  * @param  void
  * @retval 位置环实时指令时间
  * @attention  用于发射延时测试
  */
portTickType REVOL_uiGetRevolTime(void)
{
	return posishoot_time;
}



