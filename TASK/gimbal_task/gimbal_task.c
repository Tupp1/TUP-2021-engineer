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

//�������ֵ���� 0��8191
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
		
//��̨���������������
Gimbal_Control_t gimbal_control;
//���͵�canָ�����can��������̨������Ҫ��ֵ
int16_t Can1_yaw_motor1_Set_Current = 0,Can1_yaw_motor2_Set_Current = 0,Shoot_Can_Set_Current = 0;

//��̨��ʼ��
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);

//��̨״̬����
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);

//��̨���ݸ���
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);

//��̨״̬�л��������ݣ������������״̬�л���������״̬����Ŀ��ֵ
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);

//������̨��������ֵ����ԽǶ�
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//������̨������
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//��̨����pid����
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);
static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_zore_force_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_init_angle_control(Gimbal_Control_t *gimbal_control_init_anglr_loop);
static void gimbal_motor_ramp_angle_control(Gimbal_Control_t *gimbal_control_ramp_anglr_loop);
	
//�������ǽǶȿ����£��Կ��Ƶ�Ŀ��ֵ�������Է��������ԽǶ�
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_absolute_angle_NOlimit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
//PID��ʼ��
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
//����ֵ�Ӻͺ���
static void GIMBAL_UpdateMotorAngleSum(UPTrans_Motor_t *UPTrans_Motor);

//������ת�����ַ���
void num2char(uint8_t *str, double number, uint8_t g, uint8_t l);
uint8_t UI_relative_angle[10];

#if GIMBAL_TEST_MODE
//j-scope ����pid����
static void J_scope_gimbal_test(void);
#endif
  
void GIMBAL_task(void *pvParameters)
{
    //�ȴ������������������������
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //��̨��ʼ��
    GIMBAL_Init(&gimbal_control);
	  portTickType currentTime;	
    while (1)
    {
		   	//��ǰϵͳʱ��
			  currentTime = xTaskGetTickCount();         
			  //������̨ģʽ
        GIMBAL_Set_Mode(&gimbal_control);  
        //����ģʽ�л� �������ݹ���			
        GIMBAL_Mode_Change_Control_Transit(&gimbal_control); 
			  //��̨���ݸ���
        GIMBAL_Feedback_Update(&gimbal_control);            			
		  	//ң�������ݴ���
        GIMBAL_Set_Contorl(&gimbal_control); 
			  //PID����
        GIMBAL_Control_loop(&gimbal_control); 
        //����ֵ��ֵ			
        Can1_yaw_motor1_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
			  Can1_yaw_motor2_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
			  //CAN1����  ��ʶ����2FF  YAW������6020
        CAN1_CMD_2FF(Can1_yaw_motor1_Set_Current,Can1_yaw_motor2_Set_Current,0,0);    
				//CAN1����  ��ʶ����1FF  ������������ 
		    CAN1_CMD_1FF(gimbal_control.lock_motor.given_current,0,0,0);			
        //������ʱ 2MS
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

//��ʼ��pid ����ָ��
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{		 
	//��ȡң��������ָ��
	gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	
	//��ȡ�������ָ��
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor1_Measure_Point();   
	gimbal_init->lock_motor.uptrans_motor_measure = get_lock_Motor_Measure_Point(); 
	
	//����������ָ���ȡ
	gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
	gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point(); 
	
	//��ʼΪ����ģʽ
	gimbal_init->gimbal_motor_mode = GIMBAL_ZERO_FORCE;
	gimbal_init->gimbal_sub_motor_mode = GIMBAL_SUB_NAMORL;
	//�����Ϊ����ģʽ
	gimbal_init->lock_motor_mode = LOCK_MOTOR_NAMORL;
  //�����PID����Ϊ0
	gimbal_init->gimbal_lock_loop = 0;
	//����λ��Ϊ��
	gimbal_init->direction_change = 0;
	
	//��Ҫ����
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
	
	//���̧�����PID��ʼ�� 
	PID_Init(&(gimbal_init->lock_motor.motor_speed_pid), PID_POSITION, lock_motor_speed_pid,  
						LOCKMOTOR_SPEED_PID_MAX_OUT, LOCKMOTOR_SPEED_PID_MAX_IOUT);
	PID_Init(&(gimbal_init->lock_motor.motor_angle_pid), PID_POSITION, lock_motor_angle_pid, 
						LOCKMOTOR_ANGLE_PID_MAX_OUT, LOCKMOTOR_ANGLE_PID_MAX_IOUT); 
	
	//����һ������
	GIMBAL_Feedback_Update(gimbal_init);
	//����о�ûʲô��
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
		//ÿ�λ�е��ж����Ҫ���ģ�����
		static fp32 ecd_1 = 2175;
		static fp32 ecd_2 = 143;		
    static fp32 ecd_change;	
		
		//�ж���̨�Ƿ���ת
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
		
		//���ذ��Ϸ����ĽǶ�
    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle  = (*(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET));
   
		//��ԽǶȼ���
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                      ecd_change); 
		
		//���ٶȣ����ղ�����pitch����ԽǶȼ����ã�ֱ���������Ƿ����Ľ��ٶȻ�����⣬���ʣ���Ҳ��֪��Ϊɶ���Գ�����
		//����Ҳû��ʱ���پ��ᣬ��һֱ��ô�ã�Ч��Ҳ����
	  gimbal_feedback_update->gimbal_pitch_motor.relative_angle = -0.10f;    //��pitch����ԽǶ����һ����ֵ
		
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro =  -(arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET)));

    //���������������ݸ���
		gimbal_feedback_update->lock_motor.motor_speed = gimbal_feedback_update->lock_motor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
		GIMBAL_UpdateMotorAngleSum(&(gimbal_feedback_update->lock_motor));

    true_relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                      ecd_1); 	
																																											
//	num2char(UI_relative_angle, gimbal_feedback_update->gimbal_yaw_motor.relative_angle, 2, 3);
	  sprintf(UI_relative_angle, "%.2f", true_relative_angle);  //дUI���ϰ������ľ���
}
//������ԽǶ�
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

//��̨״̬�л����棬����״̬�л�����
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
		
    //����ģʽ�£���ԽǶȹ���
		if(gimbal_mode_change->gimbal_motor_mode == GIMBAL_LOCK)
		{
			if(gimbal_mode_change->gimbal_lock_loop == 0)  //û���õ���ԽǶ�
			{
				gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
			}		
		}		
}

//��̨����������
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }		
    fp32 add_yaw_angle = 0.0f;
		
		//PID��ʼ��
		fp32 Yaw_speed_pid[3] = {-7000.0f, -0.0f, -0.0f};   //-5500���״̬��������Ϊ������Ʈ������̨���ص��µ������
		//��ʼ�����ԽǶ�PID
		GIMBAL_PID_Init(&gimbal_set_control->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 
										 10, 3,     //10
										 -10.0f, -0.0f, -0.0f );  //        //-20���״̬��������Ϊ������Ʈ������̨���ص��µ������
		//��ʼ����ԽǶ�PID
		GIMBAL_PID_Init(&gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, 
										 15, 3, 
										 -13.0f, -0.0f, -0.0f );
		//��ʼ���ٶ�PID
		PID_Init(&gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, 
							30000, 5000); 
	
	  gimbal_behaviour_control_set(&add_yaw_angle, gimbal_set_control);

    if (gimbal_set_control->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_set_control->gimbal_yaw_motor.current_set = 0;
    }
    else if (gimbal_set_control->gimbal_motor_mode == GIMBAL_LOCK)
    {
			//һֱΪ������Ҫʹ��ʱ����ԽǶȿ���
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

//������ ����������
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //��ǰ�������Ƕ�
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);  
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //�����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)
        {
            //�����һ��������ӽǶȣ�
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

//������ �����������ƣ�С����ģʽ
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
    //�Ƿ񳬹���� ��Сֵ
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}

//��̨����״̬ʹ�ò�ͬ����pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }	
    //yaw��ͬģʽ���ڲ�ͬ�Ŀ��ƺ���
		//����ģʽ
    if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {		
      gimbal_motor_zore_force_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
	  //��̨��סʱ��̨����
    else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_LOCK)
    {
			//һֱΪ������Ҫʹ��ʱ����ԽǶȿ���
			if(gimbal_control_loop->gimbal_lock_loop == 0)
			{
				gimbal_motor_zore_force_control(&gimbal_control_loop->gimbal_yaw_motor);
			}
			else if(gimbal_control_loop->gimbal_lock_loop == 1)
			{
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
			}			
    }
		//��ʼ������ԽǶȿ��ƣ�ʹ��ԽǶ�Ϊ�㣬��̨������
		else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_INIT)
    {		
      gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
	  //����Ϊ���ԽǶȿ���
    else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
			gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
  	//�����õ���ԽǶ�ģʽ
		else if (gimbal_control_loop->gimbal_motor_mode == GIMBAL_RELATIVE_ANGLE)
    {		
      gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
		//С����ģʽ�þ��ԽǶȿ��ƣ�Ϊ�˱�����̨����
		else if(gimbal_control_loop->gimbal_motor_mode == GIMBAL_SPIN)
		{
      gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
		}

		//�����ģʽ����
		if(gimbal_control_loop->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {	
			//�������͵���ֵΪ0
			gimbal_control_loop->lock_motor.given_current = 0;
			//����ģʽ��Ҳ���Խ��г�ʼ����
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
    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
		                                                gimbal_motor->absolute_angle, 
		                                                gimbal_motor->absolute_angle_set, 
		                                                gimbal_motor->motor_gyro);
		
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, 
		                                      gimbal_motor->motor_gyro, 
		                                      gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, 
		                                                gimbal_motor->relative_angle, 
		                                                gimbal_motor->relative_angle_set, 
		                                                gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, 
																					gimbal_motor->motor_gyro, 
																					gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

//�Ƕ�ֱ�Ӽ���
static void gimbal_motor_init_angle_control(Gimbal_Control_t *gimbal_control_init_anglr_loop)
{
  	static fp32 angle_out;
		static uint8_t lock_motor_init_step = 1;
		static fp32 lock_motor_init_time = 0;
	
		lock_off();    //������ر�
	
		if(lock_motor_init_step == 1)
		{
			gimbal_control_init_anglr_loop->lock_motor.motor_angle = 0;
			gimbal_control_init_anglr_loop->lock_motor.init_angle_set = gimbal_control_init_anglr_loop->lock_motor.motor_angle; 
			lock_motor_init_step = 2;
		}
		else if(lock_motor_init_step == 2)
		{
			//���ٷ�����ת
			gimbal_control_init_anglr_loop->lock_motor.init_angle_set += 70; 
			//�ж��Ƿ񵽴���λ��   ע��fabs�󸡵�������ֵ��abs����������ֵ
			if(fabs(gimbal_control_init_anglr_loop->lock_motor.init_angle_set - gimbal_control_init_anglr_loop->lock_motor.motor_angle) > 5000)   
			{
				lock_motor_init_time++;
			}
			if(lock_motor_init_time == 1000)
			{
				//��ʱ�ҵ�
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
//�Ƕ�б�¼���
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
	
//����ģʽ������ֱֵ�Ӹ�0
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

//����ֵ�Ӻͺ���
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

