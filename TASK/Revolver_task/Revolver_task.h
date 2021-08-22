#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H

#include "system.h"
#include "remote.h"
#include "can_receive.h"
#include "user_lib.h"
#include "pid.h"
#define REVOLVER_TASK_INIT_TIME 200
#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define PRESS_LONG_TIME 400
#define RevChannel 1

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

#define 	REVOL_SPEED_GRID      12			//���̸���
#define  	AN_BULLET         (24576.0f)		//�����ӵ����λ������ֵ
#define   REVOL_CAN_OPEN    350  //Ħ����ʵ���ٶȳ������ֵ��������ת��,����Ħ������СĿ���ٶ����ı�


#define REVOL_STEP0    0		//ʧ�ܱ�־
#define REVOL_STEP1    1		//SW1��λ��־
#define REVOL_STEP2    2		//���ֿ��ر�־


#define REVOLVER_SPEED_PID_KP 900.0f
#define REVOLVER_SPEED_PID_KI 0
#define REVOLVER_SPEED_PID_KD 0
#define REVOLVER_SPEED_PID_MAX_OUT 8000.0f
#define REVOLVER_SPEED_PID_MAX_IOUT 5000.0f

#define REVOLVER_POSITION_PID_KP 0.0008f
#define REVOLVER_POSITION_PID_KI 0
#define REVOLVER_POSITION_PID_KD 0
#define REVOLVER_POSITION_PID_MAX_OUT 10.0f
#define REVOLVER_POSITION_PID_MAX_IOUT 10.0f

//���rmp �仯�� ��ת�ٶȵı���


typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
	REVOL_STOP_MODE =2,
}eRevolverCtrlMode;


typedef enum
{
	SHOOT_NORMAL       =  0,//���ģʽѡ��,Ĭ�ϲ���
	SHOOT_SINGLE       =  1,//����
	SHOOT_TRIPLE       =  2,//������
	SHOOT_HIGHTF_LOWS  =  3,//����Ƶ������
	SHOOT_MIDF_HIGHTS  =  4,//����Ƶ������
	SHOOT_BUFF         =  5,//���ģʽ
	SHOOT_AUTO         =  6,//�����Զ����
}eShootAction;



typedef struct
{
	  const RC_ctrl_t *revolver_rc_ctrl;
	  RC_ctrl_time_t RC_revolver_ctrl_time;
    const motor_measure_t *revolver_motor_measure;
	  PidTypeDef revolver_motor_speed_pid;         //����ٶ�PID
	  PidTypeDef revolver_motor_position_pid;         //���λ��PID
	
	  //Ħ����
	  ramp_function_source_t fric1_ramp;
    ramp_function_source_t fric2_ramp;
	
  	eShootAction actShoot;
	  eRevolverCtrlMode Revolver_mode;
	  eRevolverCtrlMode last_Revolver_mode;
	
	  bool_t press_l;
    bool_t last_press_l;
    uint16_t press_l_time;
	   
	  uint16_t key_ShootNum;
	
	  fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
	  fp32 set_ramp_angle;
		int64_t sum_ecd;
		
		fp32 revolver_buff_ramp;
    int16_t given_current;
   
} Revolver_Control_t;





void Revolver_task(void *pvParameters);
portTickType REVOL_uiGetRevolTime(void);








#endif
