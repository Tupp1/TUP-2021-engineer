#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "buzzer.h"
#include "chassis_behaviour.h"
#include "control.h"
#include "user_lib.h"

extern Gimbal_Control_t gimbal_control;
//电磁锁标志位    0为关，1为开		
uint8_t lock_flag = 0; 	

void lock_on(void)     //电磁锁打开
{
	GPIO_SetBits(GPIOI,GPIO_Pin_0);   
  lock_flag = 1; 	
}	

void lock_off(void)    //电磁锁关闭
{
	GPIO_ResetBits(GPIOI,GPIO_Pin_0);  
  lock_flag = 0; 	
}			

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
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
  * @brief          云台校准的通过判断角速度来判断云台是否到达极限位置
  * @author         RM
  * @param[in]      对应轴的角速度，单位rad/s
  * @param[in]      计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param[in]      记录的角度 rad
  * @param[in]      反馈的角度 rad
  * @param[in]      记录的编码值 raw
  * @param[in]      反馈的编码值 raw
  * @param[in]      校准的步骤 完成一次 加一
  * @retval         返回空
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
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_motionless_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set);




void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
		static uint8_t lock_wait_flag = 0;        //大陀螺后锁模式标志位
		static fp32 lock_wait_time = 0;           //大陀螺后锁模式计时
		static uint8_t key_gimbal_spin_flag = 0;  //键盘大陀螺控制标志位
		static fp32 key_gimbal_spin_time = 0;     //键盘大陀螺控制标计时
		
		gimbal_mode_set->last_gimbal_motor_mode = gimbal_mode_set->gimbal_motor_mode;

/******************************************    一    **********************************************/	
		
	  //系统在初始化，为无力模式，优先级最高
    if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
		{
      gimbal_mode_set->gimbal_motor_mode = GIMBAL_ZERO_FORCE;			
			return ;
		}
		
/******************************************    一    **********************************************/	

/******************************************    二    **********************************************/	
		
    //QE长按电磁锁升降电机初始化，优先级第二
		//Q键控制大陀螺
		if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q && IF_KEY_PRESSED_E)
		{
			if (gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time < RC_KEY_LONG_TIME)
			 {
				 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time++;
				 if(gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time == RC_KEY_LONG_TIME)
				 {
			     gimbal_mode_set->lock_motor_mode = LOCK_MOTOR_INIT;  //电磁锁升降电机初化
					 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time = 0;	
				 }
			}	
		}
		else
		{
			gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_E_time = 0;
		}		
		
/******************************************    二    **********************************************/	
		
/******************************************    三    **********************************************/	
		
		//左通道在下，无力模式，优先级第三		
    if(switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0]))
    {
      gimbal_mode_set->gimbal_motor_mode = GIMBAL_ZERO_FORCE;
			//所有标志位计时归零
			lock_wait_flag = 0;      
			lock_wait_time = 0;          
			key_gimbal_spin_flag = 0;  
			key_gimbal_spin_time = 0;     
			return ; 
    }

/******************************************    三    **********************************************/	
	
/******************************************    四    **********************************************/	
		
		//初始化模式判断是否到达中值位置，优先级第四
    if (gimbal_mode_set->gimbal_motor_mode == GIMBAL_INIT)
    {
			static fp32 gimbal_init_time = 0; 
			static uint16_t gimbal_stop_time = 0;
			gimbal_init_time++;
			//判断yaw电机是否回到初始位置（当前相对角度小于允许的误差）
			if (fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR)
			{
				//如果到达初始位置，停止时间计时
				if (gimbal_stop_time < GIMBAL_INIT_STOP_TIME)
				{
						gimbal_stop_time++;
				}
			}
			//如果初始化时间到达一定值，虽未完成初始化，但为保证车继续工作，跳出初始化，进行后面的模式选择
			if (gimbal_init_time == 8000)
			{
				gimbal_stop_time = 0;
				gimbal_init_time = 0;	
				gimbal_mode_set->gimbal_motor_mode = GIMBAL_LOCK;	
				return; 					
			}				
			//如果停止时间没有到达一定值，并且遥控器左通道不在下，则直接返回，不进行后面的模式选择，使模式一直停在初始化模式中
			//（还说明无力模式比初始化模式优先级高）
			if (gimbal_stop_time < GIMBAL_INIT_STOP_TIME && gimbal_mode_set->gimbal_rc_ctrl->rc.s[0] != 2 )
			{    
				return; 
			}
			//如果已经达到初始位置（即到达并稳定一段时间），则完成初始化，时间归零，退出初始化模式，进行后面的模式选择
			else 
			{
				gimbal_stop_time = 0;
				gimbal_init_time = 0;
			}
    }

/******************************************    四    **********************************************/	


/******************************************   正常   **********************************************/	

	
		//左通道在中，绝对角度控制模式
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0]))
    {			
			//判断上一次是否为大陀螺模式，
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
				case 1:   //如果上一次为大陀螺模式，则用绝对角度控制，此时底盘模式为跟随云台，会进行归位
				{
					gimbal_mode_set->gimbal_motor_mode = GIMBAL_ABSOLUTE_ANGLE;
					//在允许的误差角度停留一段时间，则进入云台锁定模式
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

/******************************************   正常   **********************************************/	

/***************************************** 大陀螺判断 *********************************************/		
		
		//右拨杆在上，或者标志位为1，大陀螺
		if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0]) || key_gimbal_spin_flag == 1)
    {
      gimbal_mode_set->gimbal_motor_mode = GIMBAL_SPIN;	
    } 
		else   //按键计时归零
		{
      key_gimbal_spin_time = 0;				
		}		

/***************************************** 大陀螺判断 *********************************************/		

/***************************************** Q键大陀螺 *********************************************/

		//Q键控制大陀螺
		if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q && !IF_KEY_PRESSED_E)
		{
		  if(key_gimbal_spin_flag == 0)
		  {
			  if (gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time < RC_KEY_LONG_TIME)
				 {
					 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time++;
				   if(gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time == RC_KEY_LONG_TIME)
					 {
					   key_gimbal_spin_flag = 1;   //开启大陀螺
						 gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time = 0;	
					 }
				}
			}
			else if(key_gimbal_spin_flag == 1 && key_gimbal_spin_time > 1000)  //如果正在大陀螺，再按一下Q键关闭
			{
				key_gimbal_spin_flag = 0; 
			}		
		}
		else
		{
			gimbal_mode_set->RC_gimbal_ctrl_time.rc_keyQ_time = 0;
		}
		
/***************************************** Q键大陀螺 *********************************************/
		
		if(key_gimbal_spin_flag == 1)
		{
		  key_gimbal_spin_time ++;   //按键控制大陀螺开启，计时增加
		}
		
/**********************************  SHIFT+E 云台旋转90度 ****************************************/

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
		
/**********************************  SHIFT+E 云台旋转90度 ****************************************/
		
		//从无力模式切出，即进入初始化模式
		if (gimbal_mode_set->last_gimbal_motor_mode == GIMBAL_ZERO_FORCE && gimbal_mode_set->gimbal_motor_mode != GIMBAL_ZERO_FORCE)
		{
				gimbal_mode_set->gimbal_motor_mode = GIMBAL_INIT;
		}	
}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @author         RM
  * @param[in]      设置的yaw角度增加值，单位 rad
  * @param[in]      设置的pitch角度增加值，单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */

void gimbal_behaviour_control_set(fp32 *add_yaw,  Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw;
    static int16_t yaw_channel = 0;
    
    //将遥控器的数据处理死区 
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
	
	  rc_add_yaw = (yaw_channel * Yaw_RC_SEN 
		              - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen );	
		
		gimbal_control_set->chassis_w_offset = rc_add_yaw * 3.00f;      //变大

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
    //将控制增加量赋值
    *add_yaw = rc_add_yaw;
}

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @author         RM
  * @param[in]      void
  * @retval         返回空
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
		
		//在允许的误差范围内，电磁锁打开，将云台和底盘锁住
		if (fabs(gimbal_control_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_LOCK_ANGLE_ERROR )
		{
		  lock_on();     //电磁锁打开
		}
		else
		{
		  lock_off();    //电磁锁关闭
		}
}

static void gimbal_lock_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
		if (yaw == NULL || gimbal_control_set == NULL)
		{
				return;
		}
		
		//工程云台按键归位
		fp32 relative_angle_lock_error;
		relative_angle_lock_error = gimbal_control_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET;		

		//在允许的误差范围内，电磁锁打开，将云台和底盘锁住
		if (fabs(gimbal_control_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_LOCK_ANGLE_ERROR )
		{
			lock_on();     //电磁锁打开
			gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.min_angle_limit;
		}
		else
		{
			lock_off();    //电磁锁关闭
			gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
		}	
				
		//按下E键相对角度归零电磁锁关闭，角度增量
		if(gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E && !IF_KEY_PRESSED_Q && !IF_KEY_PRESSED_SHIFT)
		{
			gimbal_control_set->gimbal_lock_loop = 1;  //标志位置1
			lock_off();    //电磁锁关闭	
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
			gimbal_control_set->gimbal_lock_loop = 0;  //标志位置0			
		}
}

static void gimbal_init_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
		if (yaw == NULL || gimbal_control_set == NULL)
		{
				return;
		}
		
		//云台初始化时电磁锁关闭
		lock_off();    //电磁锁关闭
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
		

//	 //yaw轴角度增加量会话缓慢变化（老版本初始化）
//    *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
		
		//工程云台初始化需要缓慢进行
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
    lock_off();    //电磁锁关闭
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
    //不需要处理，
}

static void gimbal_relative_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    lock_off();    //电磁锁关闭
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
    //不需要处理，
}

static void gimbal_absolute_spin_angle_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
		//大陀螺时电磁锁关闭
		lock_off();    //电磁锁关闭
		gimbal_control_set->lock_motor.motor_angle_set = gimbal_control_set->lock_motor.mid_angle_limit;
}

/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_motionless_control(fp32 *yaw, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
}
