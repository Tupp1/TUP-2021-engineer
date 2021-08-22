
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
//#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "remote.h"
#include "user_lib.h"
//#include "chassis_behaviour.h"

#define RC_KEY_SHOT_TIME   300        //�����л�ģʽ�̰�ʱ��
#define RC_KEY_LONG_TIME   500        //�����л�ģʽ����ʱ��
#define RC_KEY_SUPPLY_TIME 700        //��겹������ʱ��
#define RC_KEY_LONG_LONG_TIME 2000    //�����л�ģʽ������ʱ��

#define CHASSIS_TASK_INIT_TIME 357 //����ʼ����һ��ʱ��


#define CHASSIS_X_CHANNEL 1 //ǰ���ң����ͨ������
#define CHASSIS_Y_CHANNEL 0 //���ҵ�ң����ͨ������
#define CHASSIS_WZ_CHANNEL 2 //������ģʽ�£�����ͨ��ң����������ת

#define MODE_CHANNEL 0  //ѡ�����״̬ ����ͨ����
#define SPIN_CHANNEL 1

#define CHASSIS_VX_RC_SEN 0.0065f  //ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.0055f  //ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���


#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f  //�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_WZ_RC_SEN 0.005f   //0.01  //��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���

#define CHASSIS_ACCEL_X_NUM 0.4f   //0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.2f

#define MOTOR_DISTANCE_TO_CENTER 0.32f

//#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//#define MOTOR_DISTANCE_TO_CENTER 0.4f
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002   
#define CHASSIS_X_CONTROL_TIME 0.002   //0.002
#define CHASSIS_Y_CONTROL_TIME 0.001   //0.002
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_R
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

////���̵������ٶ�
//#define MAX_WHEEL_SPEED 4.0f
////�����˶��������ǰ���ٶ�
//#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
////�����˶��������ƽ���ٶ�
//#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f

//���̵������ٶ�
#define MAX_WHEEL_SPEED 5.0f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 3.00f  //3.75f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.70f   //3.7f

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.0f

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f

//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

#define CHASSIS_ANGLE_PID_KP   15.0f   //10
#define CHASSIS_ANGLE_PID_KI   0.0f    //0
#define CHASSIS_ANGLE_PID_KD   0.0f    //0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  6   //6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f



typedef enum
{
  CHASSIS_ZERO_FORCE,                  //��������
  CHASSIS_NO_MOVE,                     //���̱��ֲ���
  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //�����������̸�����̨
	CHASSIS_SPIN,                        //С����
	CHASSIS_NO_FOLLOW_YAW,
} chassis_behaviour_e;

typedef enum
{
  CHASSIS_SUB_NORMAL,
	CHASSIS_SUB_SLOW,           //������ĳЩģʽ�µ��̼���  ��ä��
//	CHASSIS_SUB_SUPPLY,       //����վģʽ�������ƶ�
//	CHASSIS_SUB_UPUP,
//	CHASSIS_SUB_OBLI,
} chassis_sub_behaviour_e;


typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	
} Chassis_Motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
	RC_ctrl_time_t RC_chassis_ctrl_time;
  const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
  const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
	
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  chassis_behaviour_e chassis_mode;               //���̿���״̬��
  chassis_behaviour_e last_chassis_mode;          //�����ϴο���״̬��
	
	chassis_sub_behaviour_e chassis_sub_mode;   //������ģʽ
	
  Chassis_Motor_t motor_chassis[4];          //���̵������
	
  PidTypeDef motor_speed_pid[4];             //���̵���ٶ�pid
  PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //�˲��ٶ�
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
  fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
  fp32 chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad/s
  fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
  fp32 chassis_yaw_set;

  fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
	
  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
	
	const fp32 *     chassis_w_offset;
	
} chassis_move_t;

extern void chassis_task(void *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
