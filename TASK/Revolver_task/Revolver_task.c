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

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off() fric_off()         //�ر�����Ħ����

Revolver_Control_t Revolver_control;

portTickType posishoot_time;//�����ʱ����

extern int16_t  Shoot_Can_Set_Current;

void Revolver_task(void *pvParameters)
{
	portTickType currentTime;	
	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	REVOLVER_Init(&Revolver_control);
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		Revolver_Set_Mode(&Revolver_control);
		Revolver_Feedback_Update(&Revolver_control);
		Revolver_control_loop(&Revolver_control);
//		CAN2_CMD_1FF(0,Shoot_Can_Set_Current,0,0);
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//������ʱ
	}
}

static void Revolver_control_loop(Revolver_Control_t *revolver_control)
{
	 static int16_t angle_out;
  
	 /*********************************ģʽ�л�����******************************/
	 if(revolver_control->Revolver_mode!=revolver_control->last_Revolver_mode)
	 {
		 revolver_control->set_angle=revolver_control->angle;
		 revolver_control->set_ramp_angle=revolver_control->set_angle;
	 }
	 revolver_control->last_Revolver_mode = revolver_control->Revolver_mode;
	 /*********************************ģʽ�л�����******************************/
	
	 
	 if(REVOLVER_Rc_Switch(revolver_control) == TRUE)
		{
			revolver_control->key_ShootNum++;//��һ��
		}
		if(revolver_control->key_ShootNum != 0)
		{
			revolver_control->key_ShootNum--;
			revolver_control->set_angle += AN_BULLET;
		}
		if(revolver_control->set_ramp_angle != revolver_control->set_angle)//����ת��ȥ
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
			 //�ǶȻ����ٶȻ�����pid����
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
		/************************************Ħ���ֿ���*************************************/
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
		
		/************************************Ħ���ֿ���*************************************/
		
		/***********************************���ģʽ�޸�************************************/
		if (revolver_set_mode->revolver_rc_ctrl->rc.ch[0] < -RC_CALI_VALUE_HOLE &&  \
			  revolver_set_mode->revolver_rc_ctrl->rc.ch[1] > RC_CALI_VALUE_HOLE &&  \
		    revolver_set_mode->revolver_rc_ctrl->rc.ch[2] > RC_CALI_VALUE_HOLE && \
		    revolver_set_mode->revolver_rc_ctrl->rc.ch[3] > RC_CALI_VALUE_HOLE &&  \
		    switch_is_down(revolver_set_mode->revolver_rc_ctrl->rc.s[0]) &&        \
		    switch_is_down(revolver_set_mode->revolver_rc_ctrl->rc.s[1])  )
		    
    {
        //�ж�ң����2s����� ��ʱ��ʱ�䣬 ��̨ʹ��
        shoot_mode_change_time++;
    }
		
		if(shoot_mode_change_time==2000)
		{
			led_red_toggle();
//			revolver_set_mode->Revolver_mode=!revolver_set_mode->Revolver_mode;  //������Ҫ��Ϊң����У׼�ı�ģʽ
			shoot_mode_change_time=0;
		}
	 /***********************************���ģʽ�޸�************************************/
		
	 /***********************************mouse���ģʽ�޸�************************************/
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
		/***********************************mouse���ģʽ�޸�************************************/
		
		if (revolver_set_mode->revolver_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
    {
			revolver_set_mode->actShoot=SHOOT_HIGHTF_LOWS;
		}
	
}

static void REVOLVER_Init(Revolver_Control_t *revolver_init)
{
	 //���ֵ����pidû�б�Ҫʹ����λ������
	 fp32 Revolver_speed_pid[3] = {REVOLVER_SPEED_PID_KP, REVOLVER_SPEED_PID_KI, REVOLVER_SPEED_PID_KD};
	 fp32 Revolver_position_pid[3] = {REVOLVER_POSITION_PID_KP, REVOLVER_POSITION_PID_KI, REVOLVER_POSITION_PID_KD};
	 //��ʼ��PID
	 PID_Init(&(revolver_init->revolver_motor_speed_pid), PID_POSITION, Revolver_speed_pid, 
	            REVOLVER_SPEED_PID_MAX_OUT, REVOLVER_SPEED_PID_MAX_IOUT);
	 PID_Init(&(revolver_init->revolver_motor_position_pid), PID_POSITION, Revolver_position_pid, 
	            REVOLVER_POSITION_PID_MAX_OUT, REVOLVER_POSITION_PID_MAX_IOUT);
	 
	  ramp_init(&revolver_init->fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_L_UP, Fric_OFF);
    ramp_init(&revolver_init->fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_R_UP, Fric_OFF);
 
   //ң����ָ��
   revolver_init->revolver_rc_ctrl=get_remote_control_point();
	 //���ָ��   
	 revolver_init->actShoot=SHOOT_NORMAL;
	 revolver_init->Revolver_mode=REVOL_POSI_MODE;
	 revolver_init->revolver_buff_ramp = AN_BULLET/200;
	 Revolver_Feedback_Update(revolver_init);
}

static void REVOL_UpdateMotorAngleSum(Revolver_Control_t *revolver_sum)
{		 
	//�ٽ�ֵ�жϷ�
	if (abs(revolver_sum->revolver_motor_measure->ecd - revolver_sum->revolver_motor_measure->last_ecd) > 4095)//ת����Ȧ
	{		
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (revolver_sum->revolver_motor_measure->ecd < revolver_sum->revolver_motor_measure->last_ecd)//����Ȧ�ҹ����
		{
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			revolver_sum->sum_ecd += 8191 -  revolver_sum->revolver_motor_measure->last_ecd + revolver_sum->revolver_motor_measure->ecd;
		}
		else
		{
			//������һȦ
				revolver_sum->sum_ecd -= 8191 - revolver_sum->revolver_motor_measure->ecd + revolver_sum->revolver_motor_measure->last_ecd;
		}
	}
	else      
	{
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
			 revolver_sum->sum_ecd += revolver_sum->revolver_motor_measure->ecd  -  revolver_sum->revolver_motor_measure->last_ecd;
	}
	
}

static void Revolver_Feedback_Update(Revolver_Control_t *revolver_feedback)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (revolver_feedback->revolver_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    revolver_feedback->speed = speed_fliter_3;

    REVOL_UpdateMotorAngleSum(revolver_feedback);

    //���������Ƕ�
    revolver_feedback->angle = revolver_feedback->sum_ecd;
    //��갴��
    revolver_feedback->last_press_l = revolver_feedback->press_l;
    revolver_feedback->press_l = revolver_feedback->revolver_rc_ctrl->mouse.press_l;
    //������ʱ
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

uint8_t	Revolver_Switch = 0;//����ң��ģʽ���ر�־λת��

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
  * @brief  ����ʱ���ȡ
  * @param  void
  * @retval λ�û�ʵʱָ��ʱ��
  * @attention  ���ڷ�����ʱ����
  */
portTickType REVOL_uiGetRevolTime(void)
{
	return posishoot_time;
}



