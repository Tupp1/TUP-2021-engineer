/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "filter.h"
#include "UDTrans_task.h"

#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 5000.0f
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 3.0f

#define LIFT_SPEED_PID_MAX_OUT 30000.0f;
#define LIFT_SPEED_PID_MAX_IOUT 5000.0f;
#define LIFT_POSITION_PID_MAX_OUT 10.0f
#define LIFT_POSITION_PID_MAX_IOUT 10.0f

#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 20.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 4.0f



#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 3.0f


#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 3.0f

#define QE_SPEED  0.004f

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f


#define YAW_SCOPE 2000
#define PITCH_SCOPE 1000


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0
#define Mode2Channel 1

#define SpinChannel 1
//掉头180 按键
#define TurnKeyBoard KEY_PRESSED_OFFSET_F
//掉头云台速度
#define TurnSpeed 0.04f
//测试按键尚未使用
#define TestKeyBoard KEY_PRESSED_OFFSET_R
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 20
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.000006f
#define Pitch_RC_SEN -0.000006f //0.005
//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen 0.0001f  //0.00001f     //0.00005f  //工程没有锁的时候
#define Pitch_Mouse_Sen 0.0003f
//云台编码器控制时候使用的比例
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f
//云台控制周期
#define GIMBAL_CONTROL_TIME 1

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否反装
#define PITCH_TURN 1
#define YAW_TURN 0

//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.08f  //0.1
#define GIMBAL_INIT_STOP_TIME 300
#define GIMBAL_INIT_TIME 6000
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f   //0.005
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1

/*二阶卡尔曼*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2


//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

#define LOCKMOTOR_SPEED_PID_KP 1100.0f       //900
#define LOCKMOTOR_SPEED_PID_KI 0
#define LOCKMOTOR_SPEED_PID_KD 0
#define LOCKMOTOR_SPEED_PID_MAX_OUT 8000.0f
#define LOCKMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define LOCKMOTOR_ANGLE_PID_KP -0.0004f     //-0.0009f 
#define LOCKMOTOR_ANGLE_PID_KI 0
#define LOCKMOTOR_ANGLE_PID_KD 0
#define LOCKMOTOR_ANGLE_PID_MAX_OUT 6.0f    //10
#define LOCKMOTOR_ANGLE_PID_MAX_IOUT 10.0f

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //云台无力
  GIMBAL_INIT,           //云台初始化
  GIMBAL_LOCK,           //云台锁定
  GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
  GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
  GIMBAL_MOTIONLESS,     //云台在遥控器无输入一段时间后保持不动，避免陀螺仪漂移
	GIMBAL_SPIN,					 //小陀螺模式
} gimbal_behaviour_e;

typedef enum
{
  GIMBAL_SUB_SUPPLY,
	GIMBAL_SUB_OBLI,
	GIMBAL_SUB_NAMORL,
} gimbal_sub_behaviour_e;

typedef enum
{
	LOCK_MOTOR_INIT,
	LOCK_MOTOR_NAMORL,
} lock_motor_behaviour_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;


typedef struct
{
	float debug_y_sk;// = 38;//35;//30;//移动预测系数,越大预测越多
	float debug_y_sb_sk;//哨兵预测系数
	float debug_y_sb_brig_sk;//桥头哨兵
	float debug_p_sk;//移动预测系数,越大预测越多
	float debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
	float debug_auto_err_p;//pitch角度过大关闭预测
	float debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
	float debug_kf_speed_yl;//yaw速度过低关闭预测
	float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
	float debug_kf_speed_yh;//yaw速度过高关闭预测
	float debug_kf_speed_pl;//pitch速度过低关闭预测
	float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
	float debug_kf_p_angcon;//pitch预测量限幅
	
/*************卡尔曼滤波**************/
/*一阶卡尔曼*/
//云台角度误差卡尔曼
	
	extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//定义一个kalman指针
  extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//定义一个kalman指针
	extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//定义一个kalman指针
	extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//定义一个kalman指针
	extKalman_t Vision_Distance_Kalman;
	extKalman_t Gimbal_Buff_Yaw_Error_Kalman;//底盘打符
	extKalman_t Gimbal_Buff_Pitch_Error_Kalman;//
	extKalman_t Gimbal_Buff_Yaw_Error_Gim_Kalman;//云台打符
	extKalman_t Gimbal_Buff_Pitch_Error_Gim_Kalman;//


	speed_calc_data_t Vision_Yaw_speed_Struct;
	speed_calc_data_t Vision_Pitch_speed_Struct;
	kalman_filter_t yaw_kalman_filter;
	kalman_filter_t pitch_kalman_filter;
	
	/***************自瞄******************/
	float Auto_Error_Yaw[2];//    now/last
	float Auto_Error_Pitch[2];
	float Auto_Distance;//距离单目
	
	int vision_time_js;
	uint32_t Vision_Time[2];// NOW/LAST
	
	float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值
  float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度
	
} Gimbal_Kalman_t;



typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
	 
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;
    Gimbal_PID_t gimbal_motor_relative_angle_pid;
    PidTypeDef gimbal_motor_gyro_pid;
	
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} Gimbal_Motor_t;


typedef struct
{

	  const RC_ctrl_t *gimbal_rc_ctrl;
	  RC_ctrl_time_t RC_gimbal_ctrl_time;
	
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
	
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
		UPTrans_Motor_t lock_motor;
	
		gimbal_sub_behaviour_e gimbal_sub_motor_mode;
	
    gimbal_behaviour_e gimbal_motor_mode;
    gimbal_behaviour_e last_gimbal_motor_mode;
	
	  lock_motor_behaviour_e lock_motor_mode;
	  lock_motor_behaviour_e last_lock_motor_mode;
	
	  uint16_t gimbal_lock_loop;
  	uint16_t direction_change;
	  fp32 chassis_w_offset;
	
} Gimbal_Control_t;


extern const Gimbal_Motor_t *get_yaw_motor_point(void);

extern void GIMBAL_task(void *pvParameters);
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void set_cali_gimbal_hand_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

extern const fp32* chassis_w_offset_point(void);

#endif
