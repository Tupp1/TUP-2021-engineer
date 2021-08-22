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

		static uint8_t follow_flag = 0;            //大陀螺后底盘跟随云台标志位
		static fp32 follow_time = 0;               //大陀螺后底盘跟随云台计时
		static uint8_t key_chassis_spin_flag = 0;  //键盘大陀螺控制标志位
		static fp32 key_chassis_spin_time = 0;     //键盘大陀螺控制标计时
		static fp32 spin_wait_time = 0;	           //底盘大陀螺等待计时
		static uint8_t key_chassis_slow_flag = 0;  //底盘降速控制标志位
		
		static int no_move_time = 0;
		
/******************************************    一    **********************************************/	

		//系统初始化为无力，优先级最高
    if(SYSTEM_GetSystemState() == SYSTEM_STARTING)  
		{
			chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
			return;
		}
	
/******************************************    一    **********************************************/	

/******************************************    二    **********************************************/	
		
		//右拨杆在下，无力模式，优先级第二
    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
			chassis_move_mode->chassis_sub_mode = CHASSIS_SUB_NORMAL;
      chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
			//所有标志位计时归零
			follow_flag = 0;    
		  follow_time = 0;    
		  key_chassis_spin_flag = 0;  
			key_chassis_spin_time = 0; 
      spin_wait_time = 0;
			key_chassis_slow_flag = 0;
			return ;
    }
		
/******************************************    二    **********************************************/	
		
/******************************************    三    **********************************************/	
		
		//底盘不动一段时间
		if(chassis_move_mode->chassis_mode == CHASSIS_NO_MOVE)
		{
			no_move_time++;
			//如果云台完成初始化，或者不动计时大于20000，退出不动模式
			if(gimbal_control.gimbal_motor_mode != GIMBAL_INIT || no_move_time > 4000)  
			{	
				no_move_time = 0;
			}
			else 
			{
				return;
			}
		}
		
/******************************************    三    **********************************************/	


/******************************************   正常   **********************************************/	
		
		//右拨杆在中间正常
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
			//判断上一次是否为大陀螺模式，
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
				case 1:   //如果上一次为大陀螺模式，则用底盘跟随云台控制一段时间
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

/******************************************   正常    **********************************************/	

		
/***************************************** 大陀螺判断 *********************************************/		
		
		//右拨杆在上，或者标志位为1，大陀螺
		if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]) || key_chassis_spin_flag == 1)
    {
			spin_wait_time ++;
			chassis_move_mode->chassis_sub_mode = CHASSIS_SUB_NORMAL;
			if(spin_wait_time > 400)  //等待一段时间再大陀螺
			{
				chassis_move_mode->chassis_mode = CHASSIS_SPIN;			
			}
    }
		else   //否则等待计时归零，按键计时归零
		{
			spin_wait_time = 0;
			key_chassis_spin_time = 0;		
		}
/***************************************** 大陀螺判断 *********************************************/		
		
		
/***************************************** Q键大陀螺 *********************************************/
		
		if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q && !IF_KEY_PRESSED_E)  
		{
		  if(key_chassis_spin_flag == 0)
		  {
			  if (chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time < RC_KEY_LONG_TIME)
				 {
					 chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time++;
				   if(chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time == RC_KEY_LONG_TIME)
					 {
					   key_chassis_spin_flag = 1;   //开启大陀螺
						 chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time = 0;	
					 }
				}
			}
			else if(key_chassis_spin_flag == 1 && key_chassis_spin_time > 1000)  //如果正在大陀螺，再按一下Q键关闭
			{
				key_chassis_spin_flag = 0; 
			}		
		}
		else
		{
			chassis_move_mode->RC_chassis_ctrl_time.rc_keyQ_time = 0;
		}

/***************************************** Q键大陀螺 *********************************************/
		
		if(key_chassis_spin_flag == 1)
		{
		  key_chassis_spin_time ++;   //按键控制大陀螺开启，计时增加
		}

/*************************************** SHIFT底盘降速 *******************************************/		

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
		
		//底盘降速控制条件：工程车在某些模式下，或者按键控制底盘降速标志位置1
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
		
/*************************************** SHIFT底盘降速 *******************************************/		
		
		//只要从无力模式切换出去，就先切换为不动模式
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
		//处理前进平移速度
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
		
//		//始终保证底盘与云台角度一致
//    fp32 relative_angle_set;
//		//设置控制相对云台角度
//		chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(0);
//		//计算旋转PID角速度
//		relative_angle_set = -PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 
//																	  chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle, 
//																  	chassis_move_rc_to_vector->chassis_relative_angle_set)*12/10;

//    fp32 chassis_angle_keyset;		
//    //按键控制底盘旋转速度	
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

		//设置旋转速
    *wz_set =( chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * (-CHASSIS_WZ_RC_SEN) 	//遥控器旋转速度	
				     - chassis_move_rc_to_vector->chassis_RC->mouse.x * CHASSIS_ANGLE_MOUSE_Sen );              //鼠标旋转速度				 
//						 + chassis_angle_keyset );	 //按键旋转速度
		
}

static void chassis_spin_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = (4-fabs(chassis_move_rc_to_vector->vx_set)-fabs(chassis_move_rc_to_vector->vy_set)) * 1.30f;  //正 1.3
}

