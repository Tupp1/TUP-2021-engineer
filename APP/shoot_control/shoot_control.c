
#include "shoot_control.h"
#include "CAN_Receive.h"
#include "gimbal_behaviour.h"

#include "pid.h"
#include "parameters.h"
#include "pwm.h"
#include "arm_math.h"
#include "user_lib.h"
#include "pid.h"
#include "user_lib.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


//#define TURN_CLOCKWISE





#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义

static const RC_ctrl_t *shoot_rc; //遥控器指针

PidTypeDef trigger_motor_pid;         //电机PID
Shoot_Motor_t trigger_motor;          //射击数据
shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机
//微动开关IO
#define Butten_Trig_Pin GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10)

extern void getTriggerMotorMeasure(motor_measure_t *motor);



/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);





/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    fp32 Trigger_speed_pid[3] = {Slibing_Vec_PID_arg.kp, Slibing_Vec_PID_arg.ki, Slibing_Vec_PID_arg.kd};
    //遥控器指针
    shoot_rc = get_remote_control_point();
    //电机指针
    trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
    //初始化PID
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT);
    //更新数据
    Shoot_Feedback_Update();
    ramp_init(&trigger_motor.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_L_UP, Fric_OFF);
    ramp_init(&trigger_motor.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_R_UP, Fric_OFF);

    trigger_motor.ecd_count = 0;
    trigger_motor.angle = trigger_motor.shoot_motor_measure->ecd * Motor_ECD_TO_ANGLE;
    trigger_motor.given_current = 0;
    trigger_motor.move_flag = 0;
    trigger_motor.set_angle = trigger_motor.angle;
    trigger_motor.speed = 0.0f;
    trigger_motor.speed_set = 0.0f;
    trigger_motor.BulletShootCnt = 0;
}

/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
    {
        trigger_motor.ecd_count--;
    }
    else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
    {
        trigger_motor.ecd_count++;
    }

    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
    //鼠标按键
    trigger_motor.last_press_l = trigger_motor.press_l;
    trigger_motor.press_l = shoot_rc->mouse.press_l;
    //长按计时
    if (trigger_motor.press_l)
    {
        if (trigger_motor.press_l_time < PRESS_LONG_TIME)
        {
            trigger_motor.press_l_time++;
        }
    }
    else
    {
        trigger_motor.press_l_time = 0;
    }
    //射击开关下档时间计时
    if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
    {
        if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
        {
            trigger_motor.rc_s_time++;
        }
    }
    else
    {
        trigger_motor.rc_s_time = 0;
    }
}


/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */

static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;
		//下拨判断，摩擦轮和播弹同时关闭,主要也是跟触点开关有关
	
	  if (switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
    {
        shoot_mode = SHOOT_STOP;  //都停
    }
		else	if (switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel])&&!switch_is_up(last_s))
    {
				shoot_mode = SHOOT_BULLET; //播弹
		}
		else if(switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]))
		{
			  shoot_mode = SHOOT_READY;  //开摩擦轮
		}
		
		//如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_mode = SHOOT_STOP;
    }
		
    last_s = shoot_rc->rc.s[Shoot_RC_Channel];
}

 
int16_t shoot_control_loop(void)
{
    int16_t shoot_CAN_Set_Current; 
    static uint16_t fric_pwm1 = Fric_OFF;
    static uint16_t fric_pwm2 = Fric_OFF;
	  static fp32 angle_bias;
	  static int8_t last_s = RC_SW_UP;
   
    Shoot_Feedback_Update(); //更新数据
	  //发射状态控制
    if(switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel])&&!switch_is_up(last_s)) 
    {
       trigger_motor.fric1_ramp.out = Fric_L_UP;
       trigger_motor.fric2_ramp.out = Fric_R_UP;
			
#ifdef TURN_CLOCKWISE
			 trigger_motor.set_angle-=PI/6;
			 if(trigger_motor.set_angle<-PI)
			 {
				 trigger_motor.set_angle=trigger_motor.set_angle+2*PI;
			 }
#else
		   trigger_motor.set_angle += PI/6;
			 if(trigger_motor.set_angle>PI)
			 {
				 trigger_motor.set_angle=trigger_motor.set_angle-2*PI;
			 }
#endif
			 trigger_motor.move_flag=1;
			 
    }
		else if(switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]))
		{
			 trigger_motor.fric1_ramp.max_value = Fric_L_UP;
       trigger_motor.fric2_ramp.max_value = Fric_R_UP;
			 trigger_motor.speed_set = 0;
			 ramp_calc(&trigger_motor.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

			 if(trigger_motor.fric1_ramp.out == trigger_motor.fric1_ramp.max_value)
			 {
					ramp_calc(&trigger_motor.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
			 }
		}
		else if(switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
		{
			 trigger_motor.fric1_ramp.out = Fric_OFF;
       trigger_motor.fric2_ramp.out = Fric_OFF;
			 trigger_motor.speed_set = 0; 
		}
		
		//angle_bias=trigger_motor.set_angle - trigger_motor.angle;
		
    angle_bias=gothrough_origin(trigger_motor.set_angle - trigger_motor.angle);

		if(trigger_motor.move_flag==1)
		{
			 if(fabs(angle_bias)<0.1)
			 {
					trigger_motor.speed_set=0;
				  trigger_motor.move_flag=0;
			 }
			 else
			 {
#ifdef TURN_CLOCKWISE
				 trigger_motor.speed_set = -TRIGGER_SPEED;
#else
				 trigger_motor.speed_set = TRIGGER_SPEED;
#endif
			 }
		}
       
		

		PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
    trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
    shoot_CAN_Set_Current = trigger_motor.given_current;
		
		
	  last_s = shoot_rc->rc.s[Shoot_RC_Channel];
	  fric_pwm1 = (uint16_t)(trigger_motor.fric1_ramp.out);
    fric_pwm2 = (uint16_t)(trigger_motor.fric2_ramp.out);
    shoot_fric1_on(fric_pwm1);
    shoot_fric2_on(fric_pwm2);
    return shoot_CAN_Set_Current;
	  
}


