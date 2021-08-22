#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "control.h"
#include "gimbal_behaviour.h"
#include "sys_config.h"
#include "UDTrans_task.h"
#include "Gimbal_Task.h"
extern UDTrans_Control_t UDTrans_Control;
extern Gimbal_Control_t gimbal_control;

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_spin_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
	
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

		static uint8_t follow_flag = 0;            //�����ݺ���̸�����̨��־λ
		static fp32 follow_time = 0;               //�����ݺ���̸�����̨��ʱ
		static uint8_t key_chassis_spin_flag = 0;  //���̴����ݿ��Ʊ�־λ
		static fp32 key_chassis_spin_time = 0;     //���̴����ݿ��Ʊ��ʱ
		static fp32 spin_wait_time = 0;	           //���̴����ݵȴ���ʱ
		static uint8_t key_chassis_slow_flag = 0;  //���̽��ٿ��Ʊ�־λ
		
		static int no_move_time = 0;
		
/******************************************    һ    **********************************************/	

		//ϵͳ��ʼ��Ϊ���������ȼ����
    if(SYSTEM_GetSystemState() == SYSTEM_STARTING)  
		{
			chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
			return;
		}
	
/******************************************    һ    **********************************************/	

/******************************************    ��    **********************************************/	
		
		//�Ҳ������£�����ģʽ�����ȼ��ڶ�
    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
			chassis_move_mode->chassis_sub_mode = CHASSIS_SUB_NORMAL;
      chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
			//���б�־λ��ʱ����
			follow_flag = 0;    
		  follow_time = 0;    
		  key_chassis_spin_flag = 0;  
			key_chassis_spin_time = 0; 
      spin_wait_time = 0;
			key_chassis_slow_flag = 0;
			return ;
    }
		
/******************************************    ��    **********************************************/	
		
/******************************************    ��    **********************************************/	
		
		//���̲���һ��ʱ��
		if(chassis_move_mode->chassis_mode == CHASSIS_NO_MOVE)
		{
			no_move_time++;
			//�����̨��ɳ�ʼ�������߲�����ʱ����20000���˳�����ģʽ
			if(gimbal_control.gimbal_motor_mode != GIMBAL_INIT || no_move_time > 4000)  
			{	
				no_move_time = 0;
			}
			else 
			{
				return;
			}
		}
		
/******************************************    ��    **********************************************/	


/******************************************   ����   **********************************************/	
		
		//�Ҳ������м�����
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
			//�ж���һ���Ƿ�Ϊ������ģʽ��
			if(chassis_move_mode->last_chassis_mode == CHASSIS_SPIN)
			{
				follow_flag = 1;				
			}				
			switch(follow_flag)
			{
				case 0:
				{
					chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_YAW;	
					follow_time = 0;
					break;
				}
				case 1:   //�����һ��Ϊ������ģʽ�����õ��̸�����̨����һ��ʱ��
				{
					chassis_move_mode->chassis_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
					if(fabs(gimbal_control.gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR)
					{
						follow_time ++;
						if(follow_time > 350)
						{
							follow_flag = 0;								
						}
					}
					break;
				}
			}		
    }

/******************************************   ����    **********************************************/	

		
/***************************************** �������ж� *********************************************/		
		
		//�Ҳ������ϣ����߱�־λΪ1��������
		if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]) || key_chassis_spin_flag == 1)
    {
			spin_wait_time ++;
			chassis_move_mode->chassis_sub_mode = CHASSIS_SUB_NORMAL;
			if(spin_wait_time > 400)  //�ȴ�һ��ʱ���ٴ�����
			{
				chassis_move_mode->chassis_mode = CHASSIS_SPIN;			
			}
    }
		else   //����ȴ���ʱ���㣬������ʱ����
		{
			spin_wait_time = 0;
			key_chassis_spin_time = 0;		
		}
/***************************************** �������ж� *********************************************/		
		
		
/***************************************** Q�������� *********************************************/
		
		if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q && !IF_KEY_PRESSED_E)  
		{
		  if(key_chassis_spin_flag == 0)
		  {
			  if (chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time < RC_KEY_LONG_TIME)
				 {
					 chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time++;
				   if(chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time == RC_KEY_LONG_TIME)
					 {
					   key_chassis_spin_flag = 1;   //����������
						 chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time = 0;	
					 }
				}
			}
			else if(key_chassis_spin_flag == 1 && key_chassis_spin_time > 1000)  //������ڴ����ݣ��ٰ�һ��Q���ر�
			{
				key_chassis_spin_flag = 0; 
			}		
		}
		else
		{
			chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time = 0;
		}

/***************************************** Q�������� *********************************************/
		
		if(key_chassis_spin_flag == 1)
		{
		  key_chassis_spin_time ++;   //�������ƴ����ݿ�������ʱ����
		}

/*************************************** SHIFT���̽��� *******************************************/		

		if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT && !IF_KEY_PRESSED_E)
		{
			if (chassis_move_mode->RC_chassis_ctrl_time.rc_keyShift_time < RC_KEY_LONG_TIME)
			{
				chassis_move_mode->RC_chassis_ctrl_time.rc_keyShift_time++;
				if(chassis_move_mode->RC_chassis_ctrl_time.rc_keyShift_time == RC_KEY_LONG_TIME)
				{
          switch (key_chassis_slow_flag)
					{
						case 0 :
						{
							key_chassis_slow_flag = 1;
							break ;
						}
						case 1 :
						{
							key_chassis_slow_flag = 0;
							break ;
						}					
					}
					chassis_move_mode->RC_chassis_ctrl_time.rc_keyShift_time = 0;	
				}
			}
		}
		else
		{
			chassis_move_mode->RC_chassis_ctrl_time.rc_keyShift_time = 0;
		}
		
		//���̽��ٿ������������̳���ĳЩģʽ�£����߰������Ƶ��̽��ٱ�־λ��1
		if(UDTrans_Control.uptrans_behaviour == UPTRANS_COLLECT_TINY ||
		   UDTrans_Control.uptrans_behaviour == UPTRANS_COLLECT_HUGE ||
		   UDTrans_Control.uptrans_behaviour == UPTRANS_CONVERSION   ||
	     key_chassis_slow_flag == 1 )
		{
		   chassis_move_mode->chassis_sub_mode = CHASSIS_SUB_SLOW;		
		}
		else 
		{
       chassis_move_mode->chassis_sub_mode = CHASSIS_SUB_NORMAL;
		}		
		
/*************************************** SHIFT���̽��� *******************************************/		
		
		//ֻҪ������ģʽ�л���ȥ�������л�Ϊ����ģʽ
		if (chassis_move_mode->last_chassis_mode == CHASSIS_ZERO_FORCE && chassis_move_mode->chassis_mode != CHASSIS_ZERO_FORCE)
		{
				chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
		}
		  
}
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
		else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_SPIN)
    {
        chassis_spin_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *angle_set = 0;
}

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
		//����ǰ��ƽ���ٶ�
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
		
//		//ʼ�ձ�֤��������̨�Ƕ�һ��
//    fp32 relative_angle_set;
//		//���ÿ��������̨�Ƕ�
//		chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(0);
//		//������תPID���ٶ�
//		relative_angle_set = -PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 
//																	  chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle, 
//																  	chassis_move_rc_to_vector->chassis_relative_angle_set)*12/10;

//    fp32 chassis_angle_keyset;		
//    //�������Ƶ�����ת�ٶ�	
//		if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q)
//		{
//			chassis_angle_keyset = 1.30f;
//		}
//		else if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)
//		{
//			chassis_angle_keyset = -1.30f;			
//		}
//		else
//		{
//			chassis_angle_keyset = 0;		
//		}		

		//������ת��
    *wz_set =( chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * (-CHASSIS_WZ_RC_SEN) 	//ң������ת�ٶ�	
				     - chassis_move_rc_to_vector->chassis_RC->mouse.x * CHASSIS_ANGLE_MOUSE_Sen );              //�����ת�ٶ�				 
//						 + chassis_angle_keyset );	 //������ת�ٶ�
		
}

static void chassis_spin_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = (4-fabs(chassis_move_rc_to_vector->vx_set)-fabs(chassis_move_rc_to_vector->vy_set)) * 1.30f;  //�� 1.3
}

