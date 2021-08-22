#ifndef _UDTRANS_TASK_H
#define _UDTRANS_TASK_H

#include "system.h"
#include "pid.h"
#include "remote.h"
#include "CAN_receive.h"
#define UDTRANS_TASK_INIT_TIME 200
#define TIME_STAMP_1MS    1
#define TIME_STAMP_2MS    2
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f     
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define UPTRANS_X_CHANNEL 1 //ǰ���ң����ͨ������
#define UPTRANS_Y_CHANNEL 0 //���ҵ�ң����ͨ������
#define UPTRANS_W_CHANNEL 3 //ǰ���ң����ͨ������
#define UPTRANS_RC_DEADLINE 10

#define UPTRANS_VX_RC_SEN 0.0065f  //ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define UPTRANS_VY_RC_SEN 0.0055f  //ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���

#define UPDOWNMOTOR_SPEED_PID_KP 1000.0f       
#define UPDOWNMOTOR_SPEED_PID_KI 0
#define UPDOWNMOTOR_SPEED_PID_KD 0
#define UPDOWNMOTOR_SPEED_PID_MAX_OUT 11000.0f   //8000
#define UPDOWNMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define UPDOWNMOTOR_ANGLE_PID_KP -0.0009f
#define UPDOWNMOTOR_ANGLE_PID_KI 0
#define UPDOWNMOTOR_ANGLE_PID_KD 0
#define UPDOWNMOTOR_ANGLE_PID_MAX_OUT  10.0f
#define UPDOWNMOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define ROTATEMOTOR_SPEED_PID_KP 1200.0f         //900
#define ROTATEMOTOR_SPEED_PID_KI 0
#define ROTATEMOTOR_SPEED_PID_KD 0
#define ROTATEMOTOR_SPEED_PID_MAX_OUT 15000.0f   //8000
#define ROTATEMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define ROTATEMOTOR_ANGLE_PID_KP -0.001f         //-0.0009f
#define ROTATEMOTOR_ANGLE_PID_KI 0
#define ROTATEMOTOR_ANGLE_PID_KD 0
#define ROTATEMOTOR_ANGLE_PID_MAX_OUT  13.0f     //10
#define ROTATEMOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define GOLDMOTOR_SPEED_PID_KP 1100.0f           //900
#define GOLDMOTOR_SPEED_PID_KI 0
#define GOLDMOTOR_SPEED_PID_KD 0
#define GOLDMOTOR_SPEED_PID_MAX_OUT 11000.0f     //8000
#define GOLDMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define GOLDMOTOR_ANGLE_PID_KP -0.0004f          //-0.0009f 
#define GOLDMOTOR_ANGLE_PID_KI 0
#define GOLDMOTOR_ANGLE_PID_KD 0
#define GOLDMOTOR_ANGLE_PID_MAX_OUT  6.0f        //10
#define GOLDMOTOR_ANGLE_PID_MAX_IOUT 3.0f

#define SNAGMOTOR_SPEED_PID_KP 900.0f
#define SNAGMOTOR_SPEED_PID_KI 0
#define SNAGMOTOR_SPEED_PID_KD 0
#define SNAGMOTOR_SPEED_PID_MAX_OUT 11000.0f     //8000
#define SNAGMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define SNAGMOTOR_ANGLE_PID_KP -0.0009f
#define SNAGMOTOR_ANGLE_PID_KI 0
#define SNAGMOTOR_ANGLE_PID_KD 0
#define SNAGMOTOR_ANGLE_PID_MAX_OUT  10.0f
#define SNAGMOTOR_ANGLE_PID_MAX_IOUT 5.0f

typedef enum
{	
	UPTRANS_INIT,          //��ʼ��ģʽ
	UPTRANS_MANUAL,        //�ֶ�ģʽ
	UPTRANS_COLLECT_TINY,  //С��Դ��ȡ���ģʽ
  UPTRANS_COLLECT_HUGE,  //����Դ��ȡ���ģʽ
	UPTRANS_CONVERSION,    //�һ����ģʽ
	UPTRANS_SNAG,          //�������ģʽ  Ŀǰû���õ� ����
	UPTRANS_SUPPLY,        //����ģʽ
	UPTRANS_STOP,          //��ͣģʽ
	UPTRANS_RECOVER,       //�ָ�ģʽ
	UPTRANS_NOFORCE,       //����ģʽ
	UPTRANS_HELP,          //��Ԯģʽ      Ŀǰû���õ� ����
} uptrans_behaviour_e;


typedef struct
{
    const motor_measure_t *uptrans_motor_measure;
	
    PidTypeDef motor_angle_pid;
    PidTypeDef motor_speed_pid;
	
    fp32 motor_ecd_sum;     
	  fp32 motor_angle;
    fp32 motor_angle_set;   
	  fp32 set_ramp_angle;
	
    fp32 motor_speed_set;
    fp32 motor_speed;
	
    fp32 current_set;
    int16_t given_current;
	
	  fp32 min_angle_limit;		
	  fp32 mid_angle_limit;
	  fp32 max_angle_limit;
	  fp32 MAX_angle_limit;
	
		fp32 init_angle_set;  //�����̧������õ�
		
} UPTrans_Motor_t;

typedef struct
{
	  const RC_ctrl_t *udtrans_rc_ctrl;
	  RC_ctrl_time_t RC_udtrans_ctrl_time;
	
	  uptrans_behaviour_e uptrans_behaviour;
	  uptrans_behaviour_e last_uptrans_behaviour;
	
    UPTrans_Motor_t upmotor_L;
	  UPTrans_Motor_t upmotor_R;
	  UPTrans_Motor_t rotatemotor;
		UPTrans_Motor_t goldmotor;
		UPTrans_Motor_t snagmotor_L;
		UPTrans_Motor_t snagmotor_R;
	
} UDTrans_Control_t;

void UDTrans_task(void *pvParameters);

void Clamp_on(void);                  //��ȡװ���ɿ�
void Clamp_close(void);               //��ȡװ�üн�
void stretch_on(void);                //��ȡװ�����
void stretch_close(void);             //��ȡװ���ջ�
void rescue_on(void);                 //��Ԯ���
void rescue_close(void);              //��Ԯ�ջ�
void rotary_actuator_on(void);        //������ת���״�
void rotary_actuator_close(void);     //������ת���׹ر�
void supply1_on(void);                //����1����
void supply1_close(void);             //����1�ر�
void supply1_disable(void);           //����1ʧ�� 
void supply2_on(void);                //����2����
void supply2_close(void);             //����2�ر�
void supply2_disable(void);           //����2ʧ��  
void camera_up(void);                 //ͼ����
void camera_mid(void);                //ͼ����
void camera_down(void);               //ͼ����
void camera_screen(void);             //ͼ����ʾ��

//����״̬���ʱ��
#define COLLECT_TIME_PART    300             
#define CONVERSION_TIME_PART 400  

//����ٶ�
#define HAND_FAST       150
#define HAND_SLOW       70
#define TRANS_UP_SPEED  300
#define GOLD_UP_SPEED   200

//����Դ�����ʰȡǰ̧���ĸ߶�
#define COLLECT_HUGE_UP_HEIGHT_1    105000
//����Դ�����ʰȡ�н���̧�������߶�
#define COLLECT_HUGE_UP_HEIGHT_2    200000
//����Դ�����ʰȡץ�ֽǶ�
#define COLLECT_HUGE_HAND           80000

//���ʰȡ��Ϻ�̧�����ֵĸ߶�
#define COLLECT_UP_GOLD_OVER        180000
//���ʰȡȡ��Ϻ�ץ�ֱ��ֵĽǶ�
#define COLLECT_HAND_PICKUP_HEIGHT  40000


//���һ�ץ��ɨ��߶�
#define CONVERSION_HAND_SCAN        60000
//���һ�̧�������ֶ����ڸ߶�
#define CONVERSION_UP_ADJUST        300000
//���һ�̧���½�ȡ���ĸ߶�
#define CONVERSION_UP_PICKUP1       210000
#define CONVERSION_UP_PICKUP2       90000

 
//ʰȡ�һ��Զ�ģʽ
#define COLLECT_CONVERSION_AUTO
	
#endif


