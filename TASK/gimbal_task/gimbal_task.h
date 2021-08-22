/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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


//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0
#define Mode2Channel 1

#define SpinChannel 1
//��ͷ180 ����
#define TurnKeyBoard KEY_PRESSED_OFFSET_F
//��ͷ��̨�ٶ�
#define TurnSpeed 0.04f
//���԰�����δʹ��
#define TestKeyBoard KEY_PRESSED_OFFSET_R
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_deadband 20
//yaw��pitch�Ƕ���ң�����������
#define Yaw_RC_SEN -0.000006f
#define Pitch_RC_SEN -0.000006f //0.005
//yaw,pitch�ǶȺ��������ı���
#define Yaw_Mouse_Sen 0.0001f  //0.00001f     //0.00005f  //����û������ʱ��
#define Pitch_Mouse_Sen 0.0003f
//��̨����������ʱ��ʹ�õı���
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f
//��̨��������
#define GIMBAL_CONTROL_TIME 1

//��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0

//����Ƿ�װ
#define PITCH_TURN 1
#define YAW_TURN 0

//�������ֵ����Լ���ֵ
#define Half_ecd_range 4096
#define ecd_range 8191
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR 0.08f  //0.1
#define GIMBAL_INIT_STOP_TIME 300
#define GIMBAL_INIT_TIME 6000
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f   //0.005
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1

/*���׿�����*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2


//�������ֵת���ɽǶ�ֵ
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
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_INIT,           //��̨��ʼ��
  GIMBAL_LOCK,           //��̨����
  GIMBAL_ABSOLUTE_ANGLE, //��̨�����Ǿ��ԽǶȿ���
  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���
  GIMBAL_MOTIONLESS,     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��
	GIMBAL_SPIN,					 //С����ģʽ
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
	float debug_y_sk;// = 38;//35;//30;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	float debug_y_sb_sk;//�ڱ�Ԥ��ϵ��
	float debug_y_sb_brig_sk;//��ͷ�ڱ�
	float debug_p_sk;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	float debug_auto_err_y;// = 10;//15;//10;//15;//yaw�Ƕȹ���ر�Ԥ��
	float debug_auto_err_p;//pitch�Ƕȹ���ر�Ԥ��
	float debug_kf_delay;// = 150;//100;//200;//120;//150;//Ԥ����ʱ����
	float debug_kf_speed_yl;//yaw�ٶȹ��͹ر�Ԥ��
	float debug_kf_speed_yl_sb;//̧ͷ���ڱ�ʱ��С��Ϳɿ�Ԥ����
	float debug_kf_speed_yh;//yaw�ٶȹ��߹ر�Ԥ��
	float debug_kf_speed_pl;//pitch�ٶȹ��͹ر�Ԥ��
	float debug_kf_y_angcon;// = 130;//125;//115;//135;//yawԤ�����޷�
	float debug_kf_p_angcon;//pitchԤ�����޷�
	
/*************�������˲�**************/
/*һ�׿�����*/
//��̨�Ƕ�������
	
	extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//����һ��kalmanָ��
  extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//����һ��kalmanָ��
	extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//����һ��kalmanָ��
	extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//����һ��kalmanָ��
	extKalman_t Vision_Distance_Kalman;
	extKalman_t Gimbal_Buff_Yaw_Error_Kalman;//���̴��
	extKalman_t Gimbal_Buff_Pitch_Error_Kalman;//
	extKalman_t Gimbal_Buff_Yaw_Error_Gim_Kalman;//��̨���
	extKalman_t Gimbal_Buff_Pitch_Error_Gim_Kalman;//


	speed_calc_data_t Vision_Yaw_speed_Struct;
	speed_calc_data_t Vision_Pitch_speed_Struct;
	kalman_filter_t yaw_kalman_filter;
	kalman_filter_t pitch_kalman_filter;
	
	/***************����******************/
	float Auto_Error_Yaw[2];//    now/last
	float Auto_Error_Pitch[2];
	float Auto_Distance;//���뵥Ŀ
	
	int vision_time_js;
	uint32_t Vision_Time[2];// NOW/LAST
	
	float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//�������˲��ٶȲ���ֵ
  float *yaw_kf_result, *pitch_kf_result;//���׿������˲����,0�Ƕ� 1�ٶ�
	
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
