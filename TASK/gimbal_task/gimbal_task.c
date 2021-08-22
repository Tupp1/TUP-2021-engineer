#include "Gimbal_Task.h"
#include "main.h"
#include "system.h"
#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "INS_Task.h"
#include "remote.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "crc_check.h"
#include <stdio.h>
#include <math.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "calibrate_Task.h"
#include "kalman_filter.h"
#include "kalman.h"
#include "filter.h"
#include "ramp.h"
#include "vision.h"
#include "UDTrans_task.h"

//电机编码值规整 0—8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

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

		
//#define VISION_TEST
		
		
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

extern UDTrans_Control_t UDTrans_Control;
		
//云台控制所有相关数据
Gimbal_Control_t gimbal_control;
//发送的can指令，公用can，所以云台拨弹都要赋值
int16_t Can1_yaw_motor1_Set_Current = 0,Can1_yaw_motor2_Set_Current = 0,Shoot_Can_Set_Current = 0;

//云台初始化
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);

//云台状态设置
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);

//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);

//云台状态切换保存数据，例如从陀螺仪状态切换到编码器状态保存目标值
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);

//计算云台电机相对中值的相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//设置云台控制量
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//云台控制pid计算
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);
static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_zore_force_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_init_angle_control(Gimbal_Control_t *gimbal_control_init_anglr_loop);
static void gimbal_motor_ramp_angle_control(Gimbal_Control_t *gimbal_control_ramp_anglr_loop);
	
//在陀螺仪角度控制下，对控制的目标值进限制以防超最大相对角度
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_absolute_angle_NOlimit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
//PID初始化
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
//编码值加和函数
static void GIMBAL_UpdateMotorAngleSum(UPTrans_Motor_t *UPTrans_Motor);

//浮点数转换成字符串
void num2char(uint8_t *str, double number, uint8_t g, uint8_t l);
uint8_t UI_relative_angle[10];

#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif
  
void GIMBAL_task(void *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    GIMBAL_Init(&gimbal_control);
	  portTickType currentTime;	
    while (1)
    {
		   	//当前系统时间
			  currentTime = xTaskGetTickCount();         
			  //设置云台模式
        GIMBAL_Set_Mode(&gimbal_control);  
        //控制模式切换 控制数据过渡			
        GIMBAL_Mode_Change_Control_Transit(&gimbal_control); 
			  //云台数据更新
        GIMBAL_Feedback_Update(&gimbal_control);            			
		  	//遥控器数据处理
        GIMBAL_Set_Contorl(&gimbal_control); 
			  //PID计算
        GIMBAL_Control_loop(&gimbal_control); 
        //电流值赋值			
        Can1_yaw_motor1_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
			  Can1_yaw_motor2_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
			  //CAN1发送  标识符：2FF  YAW轴两个6020
        CAN1_CMD_2FF(Can1_yaw_motor1_Set_Current,Can1_yaw_motor2_Set_Current,0,0);    
				//CAN1发送  标识符：1FF  电磁锁升降电机 
		    CAN1_CMD_1FF(gimbal_control.lock_motor.given_current,0,0,0);			
        //绝对延时 2MS
        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const fp32* chassis_w_offset_point(void)
{
	return &gimbal_control.chassis_w_offset;
}

//初始化pid 数据指针
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{		 
	//获取遥控器数据指针
	gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	
	//获取电机数据指针
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor1_Measure_Point();   
	gimbal_init->lock_motor.uptrans_motor_measure = get_lock_Motor_Measure_Point(); 
	
	//陀螺仪数据指针获取
	gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
	gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point(); 
	
	//初始为无力模式
	gimbal_init->gimbal_motor_mode = GIMBAL_ZERO_FORCE;
	gimbal_init->gimbal_sub_motor_mode = GIMBAL_SUB_NAMORL;
	//电磁锁为正常模式
	gimbal_init->lock_motor_mode = LOCK_MOTOR_NAMORL;
  //电磁锁PID计算为0
	gimbal_init->gimbal_lock_loop = 0;
	//底盘位置为零
	gimbal_init->direction_change = 0;
	
	//需要更改
	gimbal_init->gimbal_yaw_motor.max_relative_angle =  YAW_SCOPE*Motor_Ecd_to_Rad;
	gimbal_init->gimbal_yaw_motor.min_relative_angle = -YAW_SCOPE*Motor_Ecd_to_Rad; 
	
	gimbal_init->lock_motor.motor_angle = 0;
	gimbal_init->lock_motor.motor_angle_set = gimbal_init->lock_motor.motor_angle;	
		 
	gimbal_init->lock_motor.min_angle_limit = 0;        //ID:5
	gimbal_init->lock_motor.mid_angle_limit = 180000;    //140000
  gimbal_init->lock_motor.max_angle_limit = 580000;    //535361
	gimbal_init->lock_motor.MAX_angle_limit = gimbal_init->lock_motor.max_angle_limit;
	
	const fp32 lock_motor_speed_pid[3] = {LOCKMOTOR_SPEED_PID_KP, LOCKMOTOR_SPEED_PID_KI, LOCKMOTOR_SPEED_PID_KD};
	const fp32 lock_motor_angle_pid[3] = {LOCKMOTOR_ANGLE_PID_KP, LOCKMOTOR_ANGLE_PID_KI, LOCKMOTOR_ANGLE_PID_KD};
	
	//金块抬升电机PID初始化 
	PID_Init(&(gimbal_init->lock_motor.motor_speed_pid), PID_POSITION, lock_motor_speed_pid,  
						LOCKMOTOR_SPEED_PID_MAX_OUT, LOCKMOTOR_SPEED_PID_MAX_IOUT);
	PID_Init(&(gimbal_init->lock_motor.motor_angle_pid), PID_POSITION, lock_motor_angle_pid, 
						LOCKMOTOR_ANGLE_PID_MAX_OUT, LOCKMOTOR_ANGLE_PID_MAX_IOUT); 
	
	//更新一次数据
	GIMBAL_Feedback_Update(gimbal_init);
	//这里感觉没什么用
	gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
	gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
	gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;
	
}

static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}


static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
		static fp32 true_relative_angle;	
		//每次机械拆卸后都需要更改！！！
		static fp32 ecd_1 = 2175;
		static fp32 ecd_2 = 143;		
    static fp32 ecd_change;	
		
		//判断云台是否旋转
		switch (gimbal_feedback_update->direction_change)
		{
			case 0 :
			{
				ecd_change = ecd_1;
				break ;
			}
			case 1 :
			{
				ecd_change = ecd_2;
				break ;
			}					
		}		
		
		//主控板上反馈的角度
    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle  = (*(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET));
   
		//相对角度计算
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                      ecd_change); 
		
		//角速度，仿照步兵用pitch轴相对角度计算获得，直接用陀螺仪反馈的角速度会出问题，别问，咱也不知道为啥，试出来的
		//后期也没有时间再纠结，就一直这么用，效果也不错
	  gimbal_feedback_update->gimbal_pitch_motor.relative_angle = -0.10f;    //将pitch轴相对角度设成一个定值
		
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro =  -(arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET)));

    //电磁锁升降电机数据更新
		gimbal_feedback_update->lock_motor.motor_speed = gimbal_feedback_update->lock_motor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
		GIMBAL_UpdateMotorAngleSum(&(gimbal_feedback_update->lock_motor));

    true_relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                      ecd_1); 	
																																											
//	num2char(UI_relative_angle, gimbal_feedback_update->gimbal_yaw_motor.relative_angle, 2, 3);
	  sprintf(UI_relative_angle, "%.2f", true_relative_angle);  //写UI的老板遗留的警告
}
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = (ecd - offset_ecd);
    if (relative_ecd > Half_ecd_range) //4096
    {
        relative_ecd -= ecd_range; //8191
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }
    return relative_ecd * Motor_Ecd_to_Rad;
}

//云台状态切换保存，用于状态切换过渡
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
		if(gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_INIT && gimbal_mode_change->gimbal_motor_mode == GIMBAL_INIT)
		{
			gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
		}
		else if(gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_LOCK && gimbal_mode_change->gimbal_motor_mode == GIMBAL_LOCK)
		{
			gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
		}
		else if(gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_ABSOLUTE_ANGLE && gimbal_mode_change->gimbal_motor_mode == GIMBAL_ABSOLUTE_ANGLE)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
		}
		else if(gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_RELATIVE_ANGLE && gimbal_mode_change->gimbal_motor_mode == GIMBAL_RELATIVE_ANGLE)
		{
			gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
		}
		else if(gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_SPIN && gimbal_mode_change->gimbal_motor_mode == GIMBAL_SPIN)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
		}
		
    //在锁模式下，相对角度过渡
		if(gimbal_mode_change->gimbal_motor_mode == GIMBAL_LOCK)
		{
			if(gimbal_mode_change->gimbal_lock_loop == 0)  //没有用到相对角度
			{
				gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
			}		
		}		
}

//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }		
    fp32 add_yaw_angle = 0.0f;
		
		//PID初始化
		fp32 Yaw_speed_pid[3] = {-7000.0f, -0.0f, -0.0f};   //-5500最好状态，但是因为陀螺仪飘并且云台过重导致电机过热
		//初始化绝对角度PID
		GIMBAL_PID_Init(&gimbal_set_control->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 
										 10, 3,     //10
										 -10.0f, -0.0f, -0.0f );  //        //-20最好状态，但是因为陀螺仪飘并且云台过重导致电机过热
		//初始化相对角度PID
		GIMBAL_PID_Init(&gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, 
										 15, 3, 
										 -13.0f, -0.0f, -0.0f );
		//初始化速度PID
		PID_Init(&gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, 
							30000, 5000); 
	
	  gimbal_behaviour_control_set(&add_yaw_angle, gimbal_set_control);

    if (gimbal_set_control->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_set_control->gimbal_yaw_motor.current_set = 0;
    }
    else if (gimbal_set_control->gimbal_motor_mode == GIMBAL_LOCK)
    {
			//一直为无力，要使用时用相对角度控制
			if(gimbal_set_control->gimbal_lock_loop == 0)
			{
				gimbal_set_control->gimbal_yaw_motor.current_set = 0;
			}
			else if(gimbal_set_control->gimbal_lock_loop == 1)
			{  
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);		
			}
    }
		else if (gimbal_set_control->gimbal_motor_mode == GIMBAL_INIT)
    {
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }	
    else if (gimbal_set_control->gimbal_motor_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);								
    }
    else if (gimbal_set_control->gimbal_motor_mode == GIMBAL_RELATIVE_ANGLE)
    {
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);					
    }
    else if (gimbal_set_control->gimbal_motor_mode == GIMBAL_SPIN)
    {
        GIMBAL_absolute_angle_NOlimit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }
		
}

//陀螺仪 控制量限制
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);  
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            //计算出一个最大的添加角度，
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

//陀螺仪 控制量无限制，小陀螺模式
static void GIMBAL_absolute_angle_NOlimit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}


static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}

//云台控制状态使用不同控制pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }	
    //yaw不同模式对于不同的控制函数
		//无力模式
    if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {		
      gimbal_motor_zore_force_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
	  //云台锁住时云台无力
    else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_LOCK)
    {
			//一直为无力，要使用时用相对角度控制
			if(gimbal_control_loop->gimbal_lock_loop == 0)
			{
				gimbal_motor_zore_force_control(&gimbal_control_loop->gimbal_yaw_motor);
			}
			else if(gimbal_control_loop->gimbal_lock_loop == 1)
			{
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
			}			
    }
		//初始化用相对角度控制，使相对角度为零，云台摆正，
		else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_INIT)
    {		
      gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
	  //正常为绝对角度控制
    else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
			gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
  	//很少用到相对角度模式
		else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_RELATIVE_ANGLE)
    {		
      gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
		//小陀螺模式用绝对角度控制，为了保持云台不动
		else if(gimbal_control_loop->gimbal_motor_mode == GIMBAL_SPIN)
		{
      gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
		}

		//电磁锁模式控制
		if(gimbal_control_loop->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {	
			//正常发送电流值为0
			gimbal_control_loop->lock_motor.given_current = 0;
			//无力模式下也可以进行初始化，
			if(gimbal_control_loop->lock_motor_mode == LOCK_MOTOR_INIT)
			{	
				gimbal_motor_init_angle_control(gimbal_control_loop);
			}
		}
		else if(gimbal_control_loop->lock_motor_mode == LOCK_MOTOR_INIT)
		{	
			gimbal_motor_init_angle_control(gimbal_control_loop);
		}		
		else if(gimbal_control_loop->lock_motor_mode == LOCK_MOTOR_NAMORL)
    {	
			gimbal_motor_ramp_angle_control(gimbal_control_loop);			
		}

}

static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
		                                                gimbal_motor->absolute_angle, 
		                                                gimbal_motor->absolute_angle_set, 
		                                                gimbal_motor->motor_gyro);
		
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, 
		                                      gimbal_motor->motor_gyro, 
		                                      gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, 
		                                                gimbal_motor->relative_angle, 
		                                                gimbal_motor->relative_angle_set, 
		                                                gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, 
																					gimbal_motor->motor_gyro, 
																					gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

//角度直接计算
static void gimbal_motor_init_angle_control(Gimbal_Control_t *gimbal_control_init_anglr_loop)
{
  	static fp32 angle_out;
		static uint8_t lock_motor_init_step = 1;
		static fp32 lock_motor_init_time = 0;
	
		lock_off();    //电磁锁关闭
	
		if(lock_motor_init_step == 1)
		{
			gimbal_control_init_anglr_loop->lock_motor.motor_angle = 0;
			gimbal_control_init_anglr_loop->lock_motor.init_angle_set = gimbal_control_init_anglr_loop->lock_motor.motor_angle; 
			lock_motor_init_step = 2;
		}
		else if(lock_motor_init_step == 2)
		{
			//慢速反向旋转
			gimbal_control_init_anglr_loop->lock_motor.init_angle_set += 70; 
			//判断是否到达限位点   注：fabs求浮点数绝对值，abs求整数绝对值
			if(fabs(gimbal_control_init_anglr_loop->lock_motor.init_angle_set - gimbal_control_init_anglr_loop->lock_motor.motor_angle) > 5000)   
			{
				lock_motor_init_time++;
			}
			if(lock_motor_init_time == 1000)
			{
				//此时找到
				gimbal_control_init_anglr_loop->lock_motor_mode = LOCK_MOTOR_NAMORL;
				gimbal_control_init_anglr_loop->lock_motor.motor_angle = gimbal_control_init_anglr_loop->lock_motor.max_angle_limit;
				gimbal_control_init_anglr_loop->lock_motor.motor_angle_set = gimbal_control_init_anglr_loop->lock_motor.max_angle_limit;
				lock_motor_init_step = 1;
				lock_motor_init_time = 0;
			}
		}
		
  	angle_out = PID_Calc(&gimbal_control_init_anglr_loop->lock_motor.motor_angle_pid, gimbal_control_init_anglr_loop->lock_motor.init_angle_set, gimbal_control_init_anglr_loop->lock_motor.motor_angle);
	  gimbal_control_init_anglr_loop->lock_motor.given_current = PID_Calc(&gimbal_control_init_anglr_loop->lock_motor.motor_speed_pid, gimbal_control_init_anglr_loop->lock_motor.motor_speed, angle_out);	
}
//角度斜坡计算
static void gimbal_motor_ramp_angle_control(Gimbal_Control_t *gimbal_control_ramp_anglr_loop)
{
  	static fp32 angle_out;
 
		if(gimbal_control_ramp_anglr_loop->lock_motor.set_ramp_angle != gimbal_control_ramp_anglr_loop->lock_motor.motor_angle_set)
		{
			gimbal_control_ramp_anglr_loop->lock_motor.set_ramp_angle = RAMP_float(gimbal_control_ramp_anglr_loop->lock_motor.motor_angle_set, gimbal_control_ramp_anglr_loop->lock_motor.set_ramp_angle,
			                                                                       800);
		}
		
  	angle_out = PID_Calc(&gimbal_control_ramp_anglr_loop->lock_motor.motor_angle_pid, gimbal_control_ramp_anglr_loop->lock_motor.set_ramp_angle, gimbal_control_ramp_anglr_loop->lock_motor.motor_angle);
	  gimbal_control_ramp_anglr_loop->lock_motor.given_current = PID_Calc(&gimbal_control_ramp_anglr_loop->lock_motor.motor_speed_pid, gimbal_control_ramp_anglr_loop->lock_motor.motor_speed, angle_out);	
}
	
//无力模式，电流值直接赋0
static void gimbal_motor_zore_force_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = 0;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
		 //   pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//编码值加和函数
static void GIMBAL_UpdateMotorAngleSum(UPTrans_Motor_t *UPTrans_Motor)
{
	if (abs(UPTrans_Motor->uptrans_motor_measure->ecd - UPTrans_Motor->uptrans_motor_measure->last_ecd) > 4095)
	{
		if (UPTrans_Motor->uptrans_motor_measure->ecd < UPTrans_Motor->uptrans_motor_measure->last_ecd)
		{
			UPTrans_Motor->motor_angle += 8191 -  UPTrans_Motor->uptrans_motor_measure->last_ecd + UPTrans_Motor->uptrans_motor_measure->ecd;
		}
		else
		{
			UPTrans_Motor->motor_angle -= 8191 - UPTrans_Motor->uptrans_motor_measure->ecd + UPTrans_Motor->uptrans_motor_measure->last_ecd;
		}
	}
	else
	{
		UPTrans_Motor->motor_angle += UPTrans_Motor->uptrans_motor_measure->ecd - UPTrans_Motor->uptrans_motor_measure->last_ecd;
	}
}

