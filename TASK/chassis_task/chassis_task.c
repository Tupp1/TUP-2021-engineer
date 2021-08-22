#include "chassis_task.h"
#include "rc.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote.h"
#include "INS_Task.h"
#include "chassis_behaviour.h"
#include "calibrate_Task.h"
#include "sys_config.h"

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

//�����˶�����
chassis_move_t chassis_move;
extern Gimbal_Control_t gimbal_control;
		
//���̳�ʼ������Ҫ��pid��ʼ��
static void chassis_init(chassis_move_t *chassis_move_init);
//����״̬��ѡ��ͨ��ң�����Ŀ���
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//�������ݸ���
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//����״̬�ı����������ĸı�static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//�������ø���ң����������
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//����PID�����Լ��˶��ֽ�
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

//������
void chassis_task(void *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //���̳�ʼ��
    chassis_init(&chassis_move);


    while (1)
    {
        //ң��������״̬
        chassis_set_mode(&chassis_move);   
        //ң����״̬�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //���̿���PID����
        chassis_control_loop(&chassis_move);
		  	//CAN1����
				CAN1_CMD_200(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
										 chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
   
        //�����ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
		//�����ٶȻ�pidֵ
		fp32 motor_speed_pid[3] = {Chassis_Vec_PID_arg.kp, Chassis_Vec_PID_arg.ki, Chassis_Vec_PID_arg.kd};
    //������ת��pidֵ
    fp32 chassis_yaw_pid[3] = {CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};
	
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
				
		uint8_t i;
		
		//���̿���״̬Ϊֹͣ
    chassis_move_init->chassis_mode = CHASSIS_ZERO_FORCE;
		chassis_move_init->chassis_sub_mode = CHASSIS_SUB_NORMAL;
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //��ȡ��̨�������ָ��
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
		
		chassis_move_init->chassis_w_offset = chassis_w_offset_point();
		
		//��ʼ��PID �˶�
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    }
		//��ʼ����תPID
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_X_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_Y_CONTROL_TIME, chassis_y_order_filter);

    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //����һ������
    chassis_feedback_update(chassis_move_init);
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    chassis_behaviour_mode_set(chassis_move_mode);  //��һ���ӿڣ�ʹ�ó����������
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL) 
		{
		  return;
		}		
		//ģʽ��ͬ��ֱ�ӷ���	
    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode) 
		{
		  return;
		}
			
    //ת��Ϊ������̨ģʽʱ����̨��ԽǶ�Ϊ��
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) && 
			   chassis_move_transit->chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_NO_MOVE) &&
			        chassis_move_transit->chassis_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
		//���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_NO_FOLLOW_YAW) && 
			        chassis_move_transit->chassis_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //����С����ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_SPIN) && 
			        chassis_move_transit->chassis_mode == CHASSIS_SPIN)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶ�
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }

    //���µ���ǰ���ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    chassis_move_update->chassis_yaw   = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET)   - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll  = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}

//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	  static fp32 vx_set_channel, vy_set_channel;
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //ң����ԭʼͨ��ֵ
    int16_t vx_channel, vy_channel;
    
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

		//����״̬
    if (gimbal_control.direction_change == 0)  
		{
		  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
      vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
			if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
			{
					vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
			}
			else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
			{
					vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
			}

			if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
			{
					vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
			}
			else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
			{
					vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
			}			
		}
		else  //��̨��ת90��ͨ��ֵ����
		{
		  vx_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
      vy_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
			if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
			{
					vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
			}
			else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
			{
					vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
			}

			if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
			{
					vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
			}
			else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
			{
					vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
			}			
		}

    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����  �˲������ṹ�����Ϊ����ٶ�
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���  
		//�൱�������������ƣ�ң�����������м�ʱ�ٶȽṹ���������Ϊ��
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
 
//����ң�������������
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    static fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    if (chassis_move_control == NULL)
    {
        return;
    }
		
    //�����ٶ�
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //������תPID���ٶ�
        chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set)*12/10;
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_control->wz_set = 0;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
			else if (chassis_move_control->chassis_mode == CHASSIS_SPIN)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);  //�˴����ܻ��޸�

        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
			
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        //����������̨
        //���ģʽ�£��Ƕ����õ�Ϊ ���ٶ�
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	//���������ģʽΪ����ģʽ
	if(chassis_move.chassis_sub_mode == CHASSIS_SUB_NORMAL && chassis_move.chassis_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		//�����ٶ�     =  ǰ���ٶ� ƽ���ٶ�+ ���������ٶȷ�Ȩ * �˶����ľ��� * ��ת�ٶ�
    wheel_speed[0] = (-vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[1] = ( vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[2] = ( vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[3] = (-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
	}
	//�������Ϊ����ģʽ�����Ӽ��٣�ǰ��ƽ�Ƽ��٣���ת��΢��һ��
	else if(chassis_move.chassis_sub_mode == CHASSIS_SUB_SLOW && chassis_move.chassis_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		wheel_speed[0] = (-vx_set * 0.4f - vy_set * 0.4f + 0.8f * ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[1] = ( vx_set * 0.4f - vy_set * 0.4f + 0.8f * ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[2] = ( vx_set * 0.4f + vy_set * 0.4f + 0.8f * (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[3] = (-vx_set * 0.4f + vy_set * 0.4f + 0.8f * (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
	}
	else if(chassis_move.chassis_sub_mode == CHASSIS_SUB_NORMAL && chassis_move.chassis_mode != CHASSIS_NO_FOLLOW_YAW)
	{
    wheel_speed[0] = (-vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
		wheel_speed[1] = ( vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
		wheel_speed[2] = ( vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
		wheel_speed[3] = (-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
	}
	else if(chassis_move.chassis_sub_mode == CHASSIS_SUB_SLOW && chassis_move.chassis_mode != CHASSIS_NO_FOLLOW_YAW)
	{
		wheel_speed[0] = (-vx_set * 0.4f - vy_set * 0.4f + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
		wheel_speed[1] = ( vx_set * 0.4f - vy_set * 0.4f + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
		wheel_speed[2] = ( vx_set * 0.4f + vy_set * 0.4f + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
		wheel_speed[3] = (-vx_set * 0.4f + vy_set * 0.4f + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)-(*chassis_move.chassis_w_offset)*40;
	}
  
	if(chassis_move.chassis_mode == CHASSIS_SPIN)
	{
    wheel_speed[0] = (-vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[1] = ( vx_set - vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[2] = ( vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
		wheel_speed[3] = (-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
	}
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        //��ֵ����ֵ
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw����ֱ�ӷ���
        return;
    }
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //����pid
    for (i = 0; i < 4; i++)
    {
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
