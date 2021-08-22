#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "buzzer.h"
#include "chassis_behaviour.h"
#include "control.h"
#include "user_lib.h"

extern Gimbal_Control_t gimbal_control;
//�������־λ    0Ϊ�أ�1Ϊ��		
uint8_t lock_flag = 0; 	

void lock_on(void)     //�������
{
	GPIO_SetBits(GPIOI,GPIO_Pin_0);   
  lock_flag = 1; 	
}	

void lock_off(void)    //������ر�
{
	GPIO_ResetBits(GPIOI,GPIO_Pin_0);  
  lock_flag = 0; 	
}			

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
  */
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

/**
  * @brief          ��̨У׼��ͨ���жϽ��ٶ����ж���̨�Ƿ񵽴Ｋ��λ��
  * @author         RM
  * @param[in]      ��Ӧ��Ľ��ٶȣ���λrad/s
  * @param[in]      ��ʱʱ�䣬����GIMBAL_CALI_STEP_TIME��ʱ������
  * @param[in]      ��¼�ĽǶ� rad
  * @param[in]      �����ĽǶ� rad
  * @param[in]      ��¼�ı���ֵ raw
  * @param[in]      �����ı���ֵ raw
  * @param[in]      У׼�Ĳ��� ���һ�� ��һ
  * @retval         ���ؿ�
  */
#define GIMBAL_CALI_GYRO_JUDGE(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }


static void gimbal_zero_force_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);
static void gimbal_lock_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);
static void gimbal_init_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);
static void gimbal_relative_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);
static void gimbal_absolute_spin_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);

/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_motionless_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);




void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
		static uint8_t lock_wait_flag = 0;        //�����ݺ���ģʽ��־λ
		static fp32 lock_wait_time = 0;           //�����ݺ���ģʽ��ʱ
		static uint8_t key_gimbal_spin_flag = 0;  //���̴����ݿ��Ʊ�־λ
		static fp32 key_gimbal_spin_time = 0;     //���̴����ݿ��Ʊ��ʱ
		
		gimbal_mode_set->last_gimbal_motor_mode = gimbal_mode_set->gimbal_motor_mode;

/******************************************    һ    **********************************************/	
		
	  //ϵͳ�ڳ�ʼ����Ϊ����ģʽ�����ȼ����
    if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
		{
      gimbal_mode_set->gimbal_motor_mode = GIMBAL_ZERO_FORCE;			
			return ;
		}
		
/******************************************    һ    **********************************************/	

/******************************************    ��    **********************************************/	
		
    //QE������������������ʼ�������ȼ��ڶ�
		//Q�����ƴ�����
		if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q && IF_KEY_PRESSED_E)
		{
			if (gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time < RC_KEY_LONG_TIME)
			 {
				 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time++;
				 if(gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time == RC_KEY_LONG_TIME)
				 {
			     gimbal_mode_set->lock_motor_mode = LOCK_MOTOR_INIT;  //����������������
					 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time = 0;	
				 }
			}	
		}
		else
		{
			gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time = 0;
		}		
		
/******************************************    ��    **********************************************/	
		
/******************************************    ��    **********************************************/	
		
		//��ͨ�����£�����ģʽ�����ȼ�����		
    if(switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0]))
    {
      gimbal_mode_set->gimbal_motor_mode = GIMBAL_ZERO_FORCE;
			//���б�־λ��ʱ����
			lock_wait_flag = 0;      
			lock_wait_time = 0;          
			key_gimbal_spin_flag = 0;  
			key_gimbal_spin_time = 0;     
			return ; 
    }

/******************************************    ��    **********************************************/	
	
/******************************************    ��    **********************************************/	
		
		//��ʼ��ģʽ�ж��Ƿ񵽴���ֵλ�ã����ȼ�����
    if (gimbal_mode_set->gimbal_motor_mode == GIMBAL_INIT)
    {
			static fp32 gimbal_init_time = 0; 
			static uint16_t gimbal_stop_time = 0;
			gimbal_init_time++;
			//�ж�yaw����Ƿ�ص���ʼλ�ã���ǰ��ԽǶ�С���������
			if (fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR)
			{
				//��������ʼλ�ã�ֹͣʱ���ʱ
				if (gimbal_stop_time < GIMBAL_INIT_STOP_TIME)
				{
						gimbal_stop_time++;
				}
			}
			//�����ʼ��ʱ�䵽��һ��ֵ����δ��ɳ�ʼ������Ϊ��֤������������������ʼ�������к����ģʽѡ��
			if (gimbal_init_time == 8000)
			{
				gimbal_stop_time = 0;
				gimbal_init_time = 0;	
				gimbal_mode_set->gimbal_motor_mode = GIMBAL_LOCK;	
				return; 					
			}				
			//���ֹͣʱ��û�е���һ��ֵ������ң������ͨ�������£���ֱ�ӷ��أ������к����ģʽѡ��ʹģʽһֱͣ�ڳ�ʼ��ģʽ��
			//����˵������ģʽ�ȳ�ʼ��ģʽ���ȼ��ߣ�
			if (gimbal_stop_time < GIMBAL_INIT_STOP_TIME && gimbal_mode_set->gimbal_rc_ctrl->rc.s[0] != 2 )
			{    
				return; 
			}
			//����Ѿ��ﵽ��ʼλ�ã������ﲢ�ȶ�һ��ʱ�䣩������ɳ�ʼ����ʱ����㣬�˳���ʼ��ģʽ�����к����ģʽѡ��
			else 
			{
				gimbal_stop_time = 0;
				gimbal_init_time = 0;
			}
    }

/******************************************    ��    **********************************************/	


/******************************************   ����   **********************************************/	

	
		//��ͨ�����У����ԽǶȿ���ģʽ
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0]))
    {			
			//�ж���һ���Ƿ�Ϊ������ģʽ��
			if (gimbal_mode_set->last_gimbal_motor_mode == GIMBAL_SPIN)
			{
				lock_wait_flag = 1;				
			}
			
			switch (lock_wait_flag)
			{
				case 0:
				{
					gimbal_mode_set->gimbal_motor_mode = GIMBAL_LOCK;
					lock_wait_time = 0;
					break;
				}
				case 1:   //�����һ��Ϊ������ģʽ�����þ��ԽǶȿ��ƣ���ʱ����ģʽΪ������̨������й�λ
				{
					gimbal_mode_set->gimbal_motor_mode = GIMBAL_ABSOLUTE_ANGLE;
					//����������Ƕ�ͣ��һ��ʱ�䣬�������̨����ģʽ
					if(fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR)
					{
						lock_wait_time ++;
						if(lock_wait_time > 400)
						{
							lock_wait_flag = 0;								
						}
					}
					break;
				}
			}						
    }

/******************************************   ����   **********************************************/	

/***************************************** �������ж� *********************************************/		
		
		//�Ҳ������ϣ����߱�־λΪ1��������
		if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0]) || key_gimbal_spin_flag == 1)
    {
      gimbal_mode_set->gimbal_motor_mode = GIMBAL_SPIN;	
    } 
		else   //������ʱ����
		{
      key_gimbal_spin_time = 0;				
		}		

/***************************************** �������ж� *********************************************/		

/***************************************** Q�������� *********************************************/

		//Q�����ƴ�����
		if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q && !IF_KEY_PRESSED_E)
		{
		  if(key_gimbal_spin_flag == 0)
		  {
			  if (gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time < RC_KEY_LONG_TIME)
				 {
					 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time++;
				   if(gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time == RC_KEY_LONG_TIME)
					 {
					   key_gimbal_spin_flag = 1;   //����������
						 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time = 0;	
					 }
				}
			}
			else if(key_gimbal_spin_flag == 1 && key_gimbal_spin_time > 1000)  //������ڴ����ݣ��ٰ�һ��Q���ر�
			{
				key_gimbal_spin_flag = 0; 
			}		
		}
		else
		{
			gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time = 0;
		}
		
/***************************************** Q�������� *********************************************/
		
		if(key_gimbal_spin_flag == 1)
		{
		  key_gimbal_spin_time ++;   //�������ƴ����ݿ�������ʱ����
		}
		
/**********************************  SHIFT+E ��̨��ת90�� ****************************************/

		if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT && IF_KEY_PRESSED_E)
		{
			if (gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyShift_E_time < RC_KEY_LONG_TIME)
			{
				gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyShift_E_time++;
				if(gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyShift_E_time == RC_KEY_LONG_TIME)
				{
          switch (gimbal_mode_set->direction_change)
					{
						case 0 :
						{
							gimbal_mode_set->direction_change = 1;
							break ;
						}
						case 1 :
						{
							gimbal_mode_set->direction_change = 0;
							break ;
						}					
					}
					gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyShift_E_time = 0;	
				}
			}
		}
		else
		{
			gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyShift_E_time = 0;
		}
		
/**********************************  SHIFT+E ��̨��ת90�� ****************************************/
		
		//������ģʽ�г����������ʼ��ģʽ
		if (gimbal_mode_set->last_gimbal_motor_mode == GIMBAL_ZERO_FORCE && gimbal_mode_set->gimbal_motor_mode != GIMBAL_ZERO_FORCE)
		{
				gimbal_mode_set->gimbal_motor_mode = GIMBAL_INIT;
		}	
}

/**
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @author         RM
  * @param[in]      ���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[in]      ���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */

void gimbal_behaviour_control_set(fp32 *add_yaw,  Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw;
    static int16_t yaw_channel = 0;
    
    //��ң���������ݴ������� 
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
	
	  rc_add_yaw = (yaw_channel * Yaw_RC_SEN 
		              - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen );	
		
		gimbal_control_set->chassis_w_offset = rc_add_yaw * 3.00f;      //���

    if (gimbal_control_set->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(&rc_add_yaw, gimbal_control_set);
    }
		else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_LOCK)
    {
        gimbal_lock_control(&rc_add_yaw, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(&rc_add_yaw, gimbal_control_set); 
    }
		else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, gimbal_control_set);
    }
		else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_SPIN)
    {
        gimbal_absolute_spin_angle_control(&rc_add_yaw, gimbal_control_set);
    }
		else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, gimbal_control_set);
    }
    //��������������ֵ
    *add_yaw = rc_add_yaw;
}

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_control.gimbal_motor_mode == GIMBAL_INIT || 
		    gimbal_control.gimbal_motor_mode == GIMBAL_MOTIONLESS || 
		    gimbal_control.gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void gimbal_zero_force_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
		
		//���������Χ�ڣ�������򿪣�����̨�͵�����ס
		if (fabs(gimbal_control_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_LOCK_ANGLE_ERROR )
		{
		  lock_on();     //�������
		}
		else
		{
		  lock_off();    //������ر�
		}
}

static void gimbal_lock_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
		if (yaw == NULL || gimbal_control_set == NULL)
		{
				return;
		}
		
		//������̨������λ
		fp32 relative_angle_lock_error;
		relative_angle_lock_error = gimbal_control_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET;		

		//���������Χ�ڣ�������򿪣�����̨�͵�����ס
		if (fabs(gimbal_control_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_LOCK_ANGLE_ERROR )
		{
			lock_on();     //�������
			gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.min_angle_limit;
		}
		else
		{
			lock_off();    //������ر�
			gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
		}	
				
		//����E����ԽǶȹ��������رգ��Ƕ�����
		if(gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E && !IF_KEY_PRESSED_Q && !IF_KEY_PRESSED_SHIFT)
		{
			gimbal_control_set->gimbal_lock_loop = 1;  //��־λ��1
			lock_off();    //������ر�	
			gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
			
//      *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
			
			if ( relative_angle_lock_error > 2.0f )
			{
				*yaw = -0.0030f;	
			}
			if ( relative_angle_lock_error > 0.80f && relative_angle_lock_error < 2.0f )
			{
				*yaw = -0.0020f;
			}
			if ( relative_angle_lock_error > 0.00f && relative_angle_lock_error < 0.80f) 
			{
				*yaw = -0.0010f;	
			}
			
			if ( relative_angle_lock_error < -2.0f )
			{
				*yaw = 0.0030f;		
			}
			if ( relative_angle_lock_error < -0.80f && relative_angle_lock_error > -2.0f )
			{
				*yaw = 0.0020f;
			}	
			if ( relative_angle_lock_error < 0.00f && relative_angle_lock_error > -0.80f) 
			{
				*yaw = 0.0010f;	
			}			
		}
		else
		{
			gimbal_control_set->gimbal_lock_loop = 0;  //��־λ��0			
		}
}

static void gimbal_init_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
		if (yaw == NULL || gimbal_control_set == NULL)
		{
				return;
		}
		
		//��̨��ʼ��ʱ������ر�
		lock_off();    //������ر�
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
		

//	 //yaw��Ƕ��������Ự�����仯���ϰ汾��ʼ����
//    *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
		
		//������̨��ʼ����Ҫ��������
		fp32 relative_angle_init_error;
		relative_angle_init_error = gimbal_control_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET;
		if ( relative_angle_init_error > 2.0f )
		{
			*yaw = -0.0030f;	
		}
		if ( relative_angle_init_error > 0.80f && relative_angle_init_error < 2.0f )
		{
			*yaw = -0.0030f;
		}
		if ( relative_angle_init_error > 0.00f && relative_angle_init_error < 0.80f) 
		{
			*yaw = -0.0010f;	
		}
		
		if ( relative_angle_init_error < -2.0f )
		{
			*yaw = 0.0030f;		
		}
		if ( relative_angle_init_error < -0.80f && relative_angle_init_error > -2.0f )
		{
			*yaw = 0.0030f;
		}	
		if ( relative_angle_init_error < 0.00f && relative_angle_init_error > -0.80f) 
		{
			*yaw = 0.0010f;	
		}
}

static void gimbal_absolute_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    lock_off();    //������ر�
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
    //����Ҫ����
}

static void gimbal_relative_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    lock_off();    //������ر�
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
    //����Ҫ����
}

static void gimbal_absolute_spin_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
		//������ʱ������ر�
		lock_off();    //������ر�
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
}

/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_motionless_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
}
