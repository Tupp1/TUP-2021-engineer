#include "UDTrans_task.h"
#include "gimbal_behaviour.h"
#include "system.h"
#include "control.h"
#include "user_lib.h"
#include "ramp.h"
#include "User_Task.h"
#include "calibrate_Task.h"
#include "chassis_task.h"
#include "io.h"
#include "Gimbal_Task.h"

//��������
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
/***********************************  ���̳�ʼ������  ************************************/
static void UDTRANS_Init(UDTrans_Control_t *udtrans_init);
/***********************************  ���̳�ʼ������  ************************************/
		
/**********************************  �����ڲ�ѭ������  ***********************************/
static void UPTrans_Set_Mode(UDTrans_Control_t *udtrans_set_mode);
static void UPTrans_Mode_Change_Control_Transit(UDTrans_Control_t *udtrans_mode_change);
static void UDTrans_Feedback_Update(UDTrans_Control_t *UDTrans_Feedback);
static void UPTrans_control_loop(UDTrans_Control_t *revolver_control);
/**********************************  �����ڲ�ѭ������  ***********************************/

/************************************  ģʽ������  *************************************/
static void UPTrans_init_control(UDTrans_Control_t *udtrans_init_control);
static void UPTrans_manual_control(UDTrans_Control_t *udtrans_manual_control);
static void UPTrans_stop_control(UDTrans_Control_t *udtrans_stop_control);
static void UPTrans_recover_control(UDTrans_Control_t *udtrans_recover_control);
static void UPTrans_help_control(UDTrans_Control_t *udtrans_help_control);	
static void UPTrans_snag_control(UDTrans_Control_t *udtrans_snag_control);		
static void UPTrans_supply_control(UDTrans_Control_t *udtrans_supply_control);	
static void UPTrans_collect_tiny_control(UDTrans_Control_t *udtrans_collect_tiny_control);
static void UPTrans_collect_huge_control(UDTrans_Control_t *udtrans_collect_huge_control);
static void UPTrans_conversion_control(UDTrans_Control_t *udtrans_conversion_control);
/************************************  ģʽ������  *************************************/

/**************************************  ��������  ***************************************/
static void UPTRANS_UpdateMotorAngleSum(UPTrans_Motor_t *UPTrans_Motor);
static fp32 fp32_constrain_upmotor_angle(UPTrans_Motor_t* motor);
static fp32 fp32_constrain_upmotor_ramp(UPTrans_Motor_t* motor);
/**************************************  ��������  ***************************************/

/**************************************  ��������  ****************************************/
static uint8_t supply1_step = 1;      //����1���Ʊ�־λ   
static uint8_t supply2_step = 1;	    //����2���Ʊ�־λ  
static uint8_t help_step    = 1;      //��Ԯ�������׿��Ʊ�־λ   
static uint8_t snag_step    = 1;	    //������ת���׿��Ʊ�־λ
static uint8_t stop_step    = 1;      //ֹͣ���裬ֻ���ڽ��ֹͣ״̬ʱΪ1
static uint8_t trans_step   = 1;	    //���̴󲿷������裬������ģʽӰ��
static fp32    trans_time   = 0;	    //���̴󲿷������ڲ���ʱ��������ģʽӰ��
static uint8_t gold_number  = 0;      //����������ڽ��ʰȡ�ͽ��һ�ģʽ�н�������
static uint8_t collect_gold = 0;      //���ʰȡ�����ڲ�����
static uint8_t conversion_gold  = 0;  //���һ������ڲ�����
static uint8_t conversion_mouse = 1;  //���һ�������Ҽ���־λ��������������
static uint8_t press_L_step = 0;      //��������־λ
static uint8_t press_R_step = 0;      //��������־λ
static uint8_t camera_place = 1;      //ͼ��λ�ñ�־λ
/**************************************  ��������  ****************************************/
	
/******************************  �������׺Ͷ��״̬��־λ  ********************************/
uint8_t supply1_flag = 0;             //����1��־λ    0Ϊ�أ�1Ϊ��
uint8_t supply2_flag = 0;	            //����2��־λ    0Ϊ�أ�1Ϊ��
uint8_t help_flag    = 0;             //��Ԯ��־λ     0Ϊ�أ�1Ϊ��
uint8_t snag_flag    = 0;	            //���ϱ�־λ     0Ϊ�أ�1Ϊ��
uint8_t clamp_flag   = 0;             //��ȡ��־λ     0Ϊ�أ�1Ϊ��
uint8_t stretch_flag = 0;	            //������־λ     0Ϊ�أ�1Ϊ��
/******************************  �������׺Ͷ��״̬��־λ  ********************************/

/**************************************  ��������  ****************************************/
extern Gimbal_Control_t gimbal_control;

UDTrans_Control_t UDTrans_Control;
/**************************************  ��������  ****************************************/

void UDTrans_task(void *pvParameters)
{
	portTickType currentTime;	
	vTaskDelay(UDTRANS_TASK_INIT_TIME);
	UDTRANS_Init(&UDTrans_Control);
	for(;;)
	{	
		currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��
		//����ģʽ
		UPTrans_Set_Mode(&UDTrans_Control);
		//ģʽ�л����ݴ���
		UPTrans_Mode_Change_Control_Transit(&UDTrans_Control);
		//���ݸ���
	  UDTrans_Feedback_Update(&UDTrans_Control);
		//ģʽ����PID����
		UPTrans_control_loop(&UDTrans_Control);
	  //CAN1����  ��ʶ����1FF  ̧�����*2 ��ȡ��ת��� ���̧�����
		CAN2_CMD_200(UDTrans_Control.upmotor_L.given_current   , UDTrans_Control.upmotor_R.given_current,
		             UDTrans_Control.rotatemotor.given_current , UDTrans_Control.goldmotor.given_current);
		//CAN2����  ��ʶ����200  �ϰ����� 
//    CAN2_CMD_1FF(UDTrans_Control.snagmotor_L.given_current , UDTrans_Control.snagmotor_R.given_current,0,0);             		 
		//������ʱ 1MS
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS); 
	}
}

static void UDTRANS_Init(UDTrans_Control_t *udtrans_init)
{
   //̧�����pid��������
	 const fp32 updownmotor_speed_pid[3] = {UPDOWNMOTOR_SPEED_PID_KP, UPDOWNMOTOR_SPEED_PID_KI, UPDOWNMOTOR_SPEED_PID_KD};
	 const fp32 updownmotor_angle_pid[3] = {UPDOWNMOTOR_ANGLE_PID_KP, UPDOWNMOTOR_ANGLE_PID_KI, UPDOWNMOTOR_ANGLE_PID_KD};
	 //��ȡ��תװ��pid��������
	 const fp32 rotatemotor_speed_pid[3] = {ROTATEMOTOR_SPEED_PID_KP, ROTATEMOTOR_SPEED_PID_KI, ROTATEMOTOR_SPEED_PID_KD};
	 const fp32 rotatemotor_angle_pid[3] = {ROTATEMOTOR_ANGLE_PID_KP, ROTATEMOTOR_ANGLE_PID_KI, ROTATEMOTOR_ANGLE_PID_KD};
	 //��ȡ��תװ��pid��������
	 const fp32 goldmotor_speed_pid[3]   = {GOLDMOTOR_SPEED_PID_KP, GOLDMOTOR_SPEED_PID_KI, GOLDMOTOR_SPEED_PID_KD};
	 const fp32 goldmotor_angle_pid[3]   = {GOLDMOTOR_ANGLE_PID_KP, GOLDMOTOR_ANGLE_PID_KI, GOLDMOTOR_ANGLE_PID_KD};
	 //��ȡ��תװ��pid��������
	 const fp32 snagmotor_speed_pid[3]   = {SNAGMOTOR_SPEED_PID_KP, SNAGMOTOR_SPEED_PID_KI, SNAGMOTOR_SPEED_PID_KD};
	 const fp32 snagmotor_angle_pid[3]   = {SNAGMOTOR_ANGLE_PID_KP, SNAGMOTOR_ANGLE_PID_KI, SNAGMOTOR_ANGLE_PID_KD};
	 
	 //̧�����PID��ʼ��
	 PID_Init(&(udtrans_init->upmotor_R.motor_speed_pid), PID_POSITION, updownmotor_speed_pid, //�ٶȻ�
	            UPDOWNMOTOR_SPEED_PID_MAX_OUT, UPDOWNMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->upmotor_R.motor_angle_pid), PID_POSITION, updownmotor_angle_pid, //�ǶȻ�
	            UPDOWNMOTOR_ANGLE_PID_MAX_OUT, UPDOWNMOTOR_ANGLE_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->upmotor_L.motor_speed_pid), PID_POSITION, updownmotor_speed_pid,  
	            UPDOWNMOTOR_SPEED_PID_MAX_OUT, UPDOWNMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->upmotor_L.motor_angle_pid), PID_POSITION, updownmotor_angle_pid, 
	            UPDOWNMOTOR_ANGLE_PID_MAX_OUT, UPDOWNMOTOR_ANGLE_PID_MAX_IOUT); 
	 
	 //��ȡ��ת���PID��ʼ��
	 PID_Init(&(udtrans_init->rotatemotor.motor_speed_pid), PID_POSITION, rotatemotor_speed_pid, 
	            ROTATEMOTOR_SPEED_PID_MAX_OUT, ROTATEMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->rotatemotor.motor_angle_pid), PID_POSITION, rotatemotor_angle_pid, 
	            ROTATEMOTOR_ANGLE_PID_MAX_OUT, ROTATEMOTOR_ANGLE_PID_MAX_IOUT);
	 
	 //���̧�����PID��ʼ�� 
	 PID_Init(&(udtrans_init->goldmotor.motor_speed_pid), PID_POSITION, goldmotor_speed_pid,  
	            GOLDMOTOR_SPEED_PID_MAX_OUT, GOLDMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->goldmotor.motor_angle_pid), PID_POSITION, goldmotor_angle_pid, 
	            GOLDMOTOR_ANGLE_PID_MAX_OUT, GOLDMOTOR_ANGLE_PID_MAX_IOUT); 

	 //�ϰ�����PID��ʼ�� 
	 PID_Init(&(udtrans_init->snagmotor_L.motor_speed_pid), PID_POSITION, snagmotor_speed_pid,  
	            SNAGMOTOR_SPEED_PID_MAX_OUT, SNAGMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->snagmotor_L.motor_angle_pid), PID_POSITION, snagmotor_angle_pid, 
	            SNAGMOTOR_ANGLE_PID_MAX_OUT, SNAGMOTOR_ANGLE_PID_MAX_IOUT); 
	 PID_Init(&(udtrans_init->snagmotor_R.motor_speed_pid), PID_POSITION, snagmotor_speed_pid,  
	            SNAGMOTOR_SPEED_PID_MAX_OUT, SNAGMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->snagmotor_R.motor_angle_pid), PID_POSITION, snagmotor_angle_pid, 
	            SNAGMOTOR_ANGLE_PID_MAX_OUT, SNAGMOTOR_ANGLE_PID_MAX_IOUT); 
	  
   //ң����ָ��
   udtrans_init->udtrans_rc_ctrl = get_remote_control_point();
	 
	 //���ָ��
   udtrans_init->upmotor_L.uptrans_motor_measure   = get_updown_l_Motor_Measure_Point();
	 udtrans_init->upmotor_R.uptrans_motor_measure   = get_updown_r_Motor_Measure_Point();
	 udtrans_init->rotatemotor.uptrans_motor_measure = get_rotate_Motor_Measure_Point();
   udtrans_init->goldmotor.uptrans_motor_measure   = get_gold_Motor_Measure_Point();
   udtrans_init->snagmotor_L.uptrans_motor_measure = get_snag_L_Motor_Measure_Point();
   udtrans_init->snagmotor_R.uptrans_motor_measure = get_snag_R_Motor_Measure_Point();
	 
	 //��ʼΪ����ģʽ
	 udtrans_init->uptrans_behaviour = UPTRANS_NOFORCE;
	 
	 //�����ʼ�Ƕ�
	 udtrans_init->upmotor_L.motor_angle = 0;
	 udtrans_init->upmotor_R.motor_angle = 0;
	 udtrans_init->rotatemotor.motor_angle = 0;
	 udtrans_init->goldmotor.motor_angle = 0;
	 udtrans_init->snagmotor_L.motor_angle = 0;
	 udtrans_init->snagmotor_R.motor_angle = 0;
	 
	 //����Ƕ�����Ϊ��ʼ�Ƕȣ����㣩
	 udtrans_init->upmotor_R.motor_angle_set = udtrans_init->upmotor_R.motor_angle;  
	 udtrans_init->upmotor_L.motor_angle_set = udtrans_init->upmotor_L.motor_angle; 
	 udtrans_init->rotatemotor.motor_angle_set = udtrans_init->rotatemotor.motor_angle;	
	 udtrans_init->goldmotor.motor_angle_set = udtrans_init->goldmotor.motor_angle;	
	 udtrans_init->snagmotor_L.motor_angle_set = udtrans_init->snagmotor_L.motor_angle;	
	 udtrans_init->snagmotor_R.motor_angle_set = udtrans_init->snagmotor_R.motor_angle;	
 
   //�������λ�ø�ֵ   ��Ҫ��������������������������
   udtrans_init->upmotor_L.min_angle_limit = 0;        //ID:1  ��ʱ��
   udtrans_init->upmotor_L.mid_angle_limit = 150000;   
	 udtrans_init->upmotor_L.max_angle_limit = 300000;   //С��Դ���߶� 305000 ʵ�����λ�� �ټ�һ�� 
	 udtrans_init->upmotor_L.MAX_angle_limit = udtrans_init->upmotor_L.max_angle_limit + 5000;
	 udtrans_init->upmotor_R.min_angle_limit = -300000;
   udtrans_init->upmotor_R.mid_angle_limit = -150000;
	 udtrans_init->upmotor_R.max_angle_limit = 0;
	 udtrans_init->upmotor_R.MAX_angle_limit = udtrans_init->upmotor_R.min_angle_limit - 5000;
	 
	 udtrans_init->rotatemotor.min_angle_limit = 0;      //ID:2
   udtrans_init->rotatemotor.mid_angle_limit = 50000;
   udtrans_init->rotatemotor.max_angle_limit = 77000;
	 udtrans_init->rotatemotor.MAX_angle_limit = 83400;

	 udtrans_init->goldmotor.min_angle_limit = 0;        //ID:4
	 udtrans_init->goldmotor.mid_angle_limit = 80000;    //������ʱ�����ܹ�
   udtrans_init->goldmotor.max_angle_limit = 190000;   //��߸߶ȣ��ӽ��߶�
	 udtrans_init->goldmotor.MAX_angle_limit = udtrans_init->goldmotor.max_angle_limit;

	 udtrans_init->snagmotor_L.min_angle_limit = 0;      //ID:6  ��ʱ��
	 udtrans_init->snagmotor_L.mid_angle_limit = 40000;
   udtrans_init->snagmotor_L.max_angle_limit = 73000; 
	 udtrans_init->snagmotor_L.MAX_angle_limit = udtrans_init->snagmotor_L.max_angle_limit + 20000;
	 udtrans_init->snagmotor_R.min_angle_limit = -73000; //ID:7
	 udtrans_init->snagmotor_R.mid_angle_limit = -40000;
   udtrans_init->snagmotor_R.max_angle_limit = 0; 
	 udtrans_init->snagmotor_R.MAX_angle_limit = udtrans_init->snagmotor_R.max_angle_limit - 20000;
	  
	 //���׳�ʼ��״̬
	 Clamp_on();                  //��ȡװ���ɿ�
	 stretch_close();             //��ȡװ���ջ� 
	 rescue_close();              //��Ԯ�ջ� 
	 rotary_actuator_close();     //���Ϲر� 
	 //�����ʼ��״̬
   supply1_close();             //����1�ر� 
	 supply2_close();             //����2�ر� 
 
	 UDTrans_Feedback_Update(udtrans_init); 
}

static void UPTrans_Set_Mode(UDTrans_Control_t *udtrans_set_mode)
{
	if (udtrans_set_mode == NULL)
	{
			return;
	}
	
	udtrans_set_mode->last_uptrans_behaviour = udtrans_set_mode->uptrans_behaviour; 

	
	/*************************************ͼ��λ��ת��*****************************************/	
				
	//R������ͼ��λ��
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R )  
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time == RC_KEY_LONG_TIME)
			{						
				switch (camera_place)
				{
					case 0:
					{
						camera_mid();             //ͼ����
						camera_place = 1;
						break;
					}
					case 1:
					{
						camera_down();            //ͼ����
						camera_place = 2;
						break;
					}
					case 2:
					{
						camera_screen();          //ͼ����ʾ��λ��
						camera_place = 3;
						break;
					}
					case 3:
					{
						camera_up();              //ͼ����
						camera_place = 0;
						break;
					}					
				}				
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time = 0;
	}
	/*************************************ͼ��λ��ת��*****************************************/	
	
	
	/*************************************ң����û��һֱ����*********************************/
	if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
	{
		udtrans_set_mode->uptrans_behaviour=UPTRANS_NOFORCE;
	}
	/*************************************ң����û��һֱ����*********************************/

	
	/**************************************ģʽ���ȼ�ת��**************************************/
	/************************************����ģʽ���ȼ����************************************/
	
	//����ģʽ���ȼ����
	//ϵͳ�ڳ�ʼ����Ϊ����ģʽ
	if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
	{
	  udtrans_set_mode->uptrans_behaviour = UPTRANS_NOFORCE;
		return ;  //ֱ�ӷ���
	}
		
	//ң������ͨ�����£�Ϊ����ģʽ
	if(switch_is_down(udtrans_set_mode->udtrans_rc_ctrl->rc.s[0]))
	{
	  udtrans_set_mode->uptrans_behaviour = UPTRANS_NOFORCE;
		return ;  //ֱ�ӷ���
	}
	
	/************************************����ģʽ���ȼ����************************************/
	
	
	/***********************************��ʼ��ģʽ���ȼ��ڶ�***********************************/

	//��ʼ��ģʽ���ȼ��ڶ�
	static fp32 init_mode_time = 0;
	if(udtrans_set_mode->uptrans_behaviour == UPTRANS_INIT)
	{
		init_mode_time++;
    if(init_mode_time == 80000)
		{
			//��ʱ����
		  init_mode_time = 0;
			//�л�Ϊֹͣģʽ
	  	udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;
		}	
		else
		{
			return ;   //ֱ�ӷ���		
		}
	}
	else
	{
	  //��ʱ����
	  init_mode_time = 0;		
	}
	/***********************************��ʼ��ģʽ���ȼ��ڶ�***********************************/

	
	/******************************������ģʽ��Ϊֹͣ���ȼ�����********************************/
	
	//������̨������ģʽ��Ϊֹͣģʽ�����ȼ�����			
	if(gimbal_control.gimbal_motor_mode == GIMBAL_SPIN)
	{
		rescue_close();           //��Ԯ�����ջأ���ֹ������ʱ������ײ
		help_step = 1;            //��Ԯ��������Ϊ1����ֹ�´ΰ�F����ʱ��Ԯ�����
		rotary_actuator_close();  //���������ջأ���ֹ������ʱ������ײ
		snag_step = 1;            //���ϲ�������Ϊ1����ֹ�´ΰ�B����ʱ���ϲ����
		udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;
		return ;   //ֱ�ӷ���
	}	

	/******************************������ģʽ��Ϊֹͣ���ȼ�����********************************/

	/***********************************�г�������Ϊֹͣģʽ***********************************/
	
	if(udtrans_set_mode->last_uptrans_behaviour == UPTRANS_NOFORCE &&
		 !switch_is_down(udtrans_set_mode->udtrans_rc_ctrl->rc.s[0]))
	{	
	  	udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;		
	}
	
	/***********************************�г�������Ϊֹͣģʽ***********************************/
	
	//����ģʽ���ȼ���ͬ
	//����CTRL��ֹͣģʽ��������CTRL�������ֶ�ģʽ
	if(udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time < RC_KEY_LONG_LONG_TIME)
		{
			 udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time++;
			 if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time == RC_KEY_LONG_TIME)
			 {
				 udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;     
			 }
			 if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time == RC_KEY_LONG_LONG_TIME)
			 {
				 udtrans_set_mode->uptrans_behaviour = UPTRANS_MANUAL;
			 }
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time = 0;
	}	

  //��ʱ��¼��
	//ң��������ͨ�����ϣ�Ϊ�ֶ�ģʽ
	if(switch_is_up(udtrans_set_mode->udtrans_rc_ctrl->rc.s[0]) && switch_is_up(udtrans_set_mode->udtrans_rc_ctrl->rc.s[1]))
	{
	  udtrans_set_mode->uptrans_behaviour = UPTRANS_MANUAL;
	}
	
	//Z��С��Դ��ʰȡ�����ſڣ�
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_COLLECT_TINY;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time = 0;
	}
	
	//X������Դ��ʰȡ  
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_COLLECT_HUGE;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time = 0;
	}
	
	//C���һ�
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_CONVERSION;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time = 0;
	}
		
	//G���ָ�
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_RECOVER;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time = 0;
	}
	
	//V������ģʽ
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_SUPPLY;				
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time = 0;
	}
	
	static  fp32 help_time = 0;
	//F����Ԯ
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time == RC_KEY_LONG_TIME)
			{
				switch (help_step)      //help_step����Ԯ���裩ȫ�ֱ�����������ģʽӰ��
				{
					case 1:
					{
						rescue_on();        //��Ԯ���
						help_step = 2;	
						break;
					}
					case 2:
					{
						rescue_close();    //��Ԯ�ջ�
						help_step = 1;	
						break;
					}	
				}
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time = 0;
		//��ֹ��Ԯ����δ�رգ��ƶ�������ײ��ǽ���������
		if(help_step == 2)
		{
			help_time ++;
			if(help_time == 20000) 
			{
				rescue_close();    //��Ԯ�ջ�
				help_step = 1;	   //��Ԯ��������Ϊ1
				help_time = 0;			
			}			
		}
		else	
		{
		  help_time = 0;			
		}
	}

  static  fp32 snag_time = 0;
	//����B�����׳��ϣ�������B�������ʼ��
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time < RC_KEY_LONG_LONG_TIME)
		{
			//�����������
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time == RC_KEY_LONG_TIME)
			{						
				switch (snag_step)           //snag_step�����ϲ��裩ȫ�ֱ�����������ģʽӰ��
				{
					case 1:
					{
						rotary_actuator_on();     //������ת���״�
						snag_step = 2;	
						break;
					}
					case 2:
					{
						rotary_actuator_close();  //��ת���׹ر�
						snag_step = 1;	
						break;
					}
				}
			}
			//�����������ʼ��
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time == RC_KEY_LONG_LONG_TIME)
			{	
				 udtrans_set_mode->uptrans_behaviour = UPTRANS_INIT;						
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time = 0;
		//��ֹ��ת����δ�رգ����ٹ��쵼�·���
		if(snag_step == 2)
		{
			snag_time ++;
			if(snag_time == 10000) 
			{
				rotary_actuator_close();   //������ת���׹ر�
				snag_step = 1;		         //���ϲ���ָ�Ϊ1
				snag_time = 0;			
			}			
		}
		else	
		{
			snag_time = 0;			
		}
	}
	
  /******************************************ģʽ���ȼ�ת��********************************************/					
}

static void UPTrans_Mode_Change_Control_Transit(UDTrans_Control_t *udtrans_mode_change)
{
	if (udtrans_mode_change == NULL)
	{
		return;
	}
	//����ģʽ�л��Ƕ�����ֵΪ��ǰֵ����ֹ�´ν���ĳЩģʽʱ�����ת
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour || udtrans_mode_change->uptrans_behaviour == UPTRANS_NOFORCE)
	{
  	udtrans_mode_change->upmotor_L.motor_angle_set   = udtrans_mode_change->upmotor_L.motor_angle;
  	udtrans_mode_change->upmotor_R.motor_angle_set   = udtrans_mode_change->upmotor_R.motor_angle;
  	udtrans_mode_change->rotatemotor.motor_angle_set = udtrans_mode_change->rotatemotor.motor_angle;
  	udtrans_mode_change->goldmotor.motor_angle_set   = udtrans_mode_change->goldmotor.motor_angle;
  	udtrans_mode_change->snagmotor_L.motor_angle_set = udtrans_mode_change->snagmotor_L.motor_angle;
  	udtrans_mode_change->snagmotor_R.motor_angle_set = udtrans_mode_change->snagmotor_R.motor_angle;
		
  	udtrans_mode_change->upmotor_L.set_ramp_angle   = udtrans_mode_change->upmotor_L.motor_angle;
  	udtrans_mode_change->upmotor_R.set_ramp_angle   = udtrans_mode_change->upmotor_R.motor_angle;
  	udtrans_mode_change->rotatemotor.set_ramp_angle = udtrans_mode_change->rotatemotor.motor_angle;
  	udtrans_mode_change->goldmotor.set_ramp_angle   = udtrans_mode_change->goldmotor.motor_angle;
  	udtrans_mode_change->snagmotor_L.set_ramp_angle = udtrans_mode_change->snagmotor_L.motor_angle;
  	udtrans_mode_change->snagmotor_R.set_ramp_angle = udtrans_mode_change->snagmotor_R.motor_angle;
	}	
	//�ж�����ģʽ��ͬ�����ҵ�ǰΪ��ʼ��ģʽ����ô����step����Ϊ1������time����Ϊ0��gold����Ϊ1��Ŀ����Ϊ���´ν�������ģʽʱȷ��������һ��
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_INIT)
	{
		snag_step = 1;	
		help_step = 1;
		trans_step = 1;
		trans_time = 0;
    collect_gold = 0;      
		conversion_gold = 0;
    conversion_mouse = 1;
    camera_place = 1;
	}	
	//�ж�����ģʽ��ͬ�����ҵ�ǰΪ�ֶ�ģʽ��ץ�ּ�ȡ���������Ʊ�־λת����Ŀ����Ϊ�˽����ֶ�ģʽʱץ���Դ�����ǰ��״̬����갴����־λ���㣬���ҽ����صĸ�������Ҳ�ص���ʼ��ֵ
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_MANUAL)
	{
	  press_L_step = clamp_flag;
	  press_R_step = stretch_flag;
		trans_step = 1;	
		trans_time = 0;
    collect_gold = 0;      	
		conversion_gold = 0;
		conversion_mouse = 1;
	}	
	//�ж�����ģʽ��ͬ��������һ��Ϊֹͣģʽ����ģʽ��ֹͣ�л���ȥ����ֹͣ��������Ϊ1��Ŀ����Ϊ���´ν���ֹͣģʽʱȷ��������һ����ֹͣ�Ƕȱ��棩
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 udtrans_mode_change->last_uptrans_behaviour == UPTRANS_STOP )
	{
		stop_step = 1;			
	}
	//�ж�����ģʽ��ͬ�����ҵ�ǰΪ�ָ�ģʽ����ô����step����Ϊ1������time����Ϊ0��gold����Ϊ1��Ŀ����Ϊ���´ν�������ģʽʱȷ��������һ��
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 udtrans_mode_change->uptrans_behaviour == UPTRANS_RECOVER )
	{
		snag_step = 1;	
		help_step = 1;
		trans_step = 1;
		trans_time = 0;
		conversion_gold = 0;
    conversion_mouse = 1;
    camera_place = 1;
		switch (collect_gold)    //���������ֵ�жϣ�ʰȡ��������             
		{
			case 0:                             //һ��ûȡ��        
			{
	  	  gold_number = collect_gold;	      //ֱ�Ӹ�ֵΪ0
	    	collect_gold = 0;						      //ֱ�Ӹ�ֵΪ0
				break;
			}
			case 1:                             //���ȡ��һ��	
			{
	  	  gold_number = collect_gold + 1;		//������һ����������		
				break; 
			}
			case 2:                             //���ȡ������
			{
	  	  gold_number = collect_gold + 1;	  //������һ����������				
				break;
			}
			case 3:                             //���ȡ������
			{
	  	  gold_number = collect_gold;	      //ֱ�Ӹ�ֵΪ3 
	    	collect_gold = 0;			            //���� 		
				break;
			}
			default:
	  	{
				collect_gold = 0;		              //�������ǣ����㴦��
				break;
		  }
		}	
	}
	//�ж�����ģʽ��ͬ��������һ��ΪС��Դ��ȡ�󡢴���Դ��ȡ�󡢽��һ�ģʽ����ͼ���ص��м�λ�ã���־λ��1
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 (udtrans_mode_change->last_uptrans_behaviour == UPTRANS_COLLECT_TINY ||
	    udtrans_mode_change->last_uptrans_behaviour == UPTRANS_COLLECT_HUGE ||
	    udtrans_mode_change->last_uptrans_behaviour == UPTRANS_CONVERSION   ) )
	{
			camera_mid();             //ͼ����
			camera_place = 1;			
	}
	//�ж�����ģʽ��ͬ��������һ��Ϊ����ģʽ����ģʽ�Ӳ����л���ȥ����������ʹ�ã�����Ȼ�رգ���ʹ���ˣ���ʧ�ܣ���ֹ��������ջ�
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 udtrans_mode_change->last_uptrans_behaviour == UPTRANS_SUPPLY )
	{
		switch (supply1_step)
		{
			case 1:
			{
			  supply1_close();     //����1��Ȼ�ر�
				break ;
			}
			case 2:
			{
			  supply1_disable();   //����1ʧ��
				break ;
			}
		}
		switch (supply2_step)
		{
			case 1:
			{
			  supply2_close();     //����2��Ȼ�ر�
				break ;
			}
			case 2:
			{
			  supply2_disable();   //����2ʧ��
				break ;
			}
		}
	}
	//�ж�����ģʽ��ͬ��������һ��Ϊ�һ�ģʽ���ʰȡ��������	
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 (udtrans_mode_change->uptrans_behaviour == UPTRANS_CONVERSION) )	
	{
		collect_gold = 0;			     
	}
	//�ֶ�ģʽ��ȡ��ģʽ���һ�ģʽ������ģʽ������ģʽ������ģʽ�£���Ԯ�����ջأ�������ת���׹رգ���ֹ����
	if(udtrans_mode_change->uptrans_behaviour == UPTRANS_MANUAL ||
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_COLLECT_TINY ||
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_COLLECT_HUGE ||
     udtrans_mode_change->uptrans_behaviour == UPTRANS_CONVERSION ||
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_SNAG ||	
     udtrans_mode_change->uptrans_behaviour == UPTRANS_SUPPLY ||		
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_NOFORCE )	
	{ 
		rescue_close();            //��Ԯ�ջ�
		help_step = 1;	           //��Ԯ��������Ϊ1	
		rotary_actuator_close();   //������ת���׹ر�
		snag_step = 1;		         //���ϲ���ָ�Ϊ1		
	}
}

static void UDTrans_Feedback_Update(UDTrans_Control_t *UDTrans_Feedback)
{
	if (UDTrans_Feedback == NULL)
	{
		return;
	}	
	static fp32 Updatetime;
	//rpm������ٶ�
	UDTrans_Feedback->upmotor_L.motor_speed   = UDTrans_Feedback->upmotor_L.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->upmotor_R.motor_speed   = UDTrans_Feedback->upmotor_R.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->rotatemotor.motor_speed = UDTrans_Feedback->rotatemotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->goldmotor.motor_speed   = UDTrans_Feedback->goldmotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->snagmotor_L.motor_speed = UDTrans_Feedback->goldmotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->snagmotor_R.motor_speed = UDTrans_Feedback->goldmotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	//����ֵ�Ӻ�
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->upmotor_L));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->upmotor_R));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->rotatemotor));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->goldmotor));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->snagmotor_L));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->snagmotor_R));
	
  //�ϵ��820R���ϵ���ϵ����ֵ����ң�ʹ���λ
  Updatetime++;
	if (Updatetime < 5000)
	{
		UDTrans_Feedback->goldmotor.motor_angle = 0;
	}	
}

static void UPTrans_control_loop(UDTrans_Control_t *udtrans_control)
{
	if (udtrans_control == NULL)
	{
		return;
	}	

	if(udtrans_control->uptrans_behaviour == UPTRANS_NOFORCE)
	{
		udtrans_control->upmotor_L.given_current   = 0;
		udtrans_control->upmotor_R.given_current   = 0;
		udtrans_control->rotatemotor.given_current = 0;	
		udtrans_control->goldmotor.given_current   = 0;
		udtrans_control->snagmotor_L.given_current = 0;
		udtrans_control->snagmotor_R.given_current = 0;
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_INIT)
	{
		UPTrans_init_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_MANUAL)
	{
		UPTrans_manual_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_STOP)
	{
		UPTrans_stop_control(udtrans_control);
	}
	else if(udtrans_control->uptrans_behaviour == UPTRANS_RECOVER)
	{
		UPTrans_recover_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_SUPPLY)
	{
		UPTrans_supply_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_COLLECT_TINY)
	{
		UPTrans_collect_tiny_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_COLLECT_HUGE)
	{
		UPTrans_collect_huge_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_CONVERSION)
	{
		UPTrans_conversion_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_HELP)
	{
		UPTrans_help_control(udtrans_control);
	}
	else if(udtrans_control->uptrans_behaviour == UPTRANS_SNAG)
	{
		UPTrans_snag_control(udtrans_control);
	}		
}


//��Ҫ���ģ�����
static void UPTrans_init_control(UDTrans_Control_t *udtrans_init_control)
{
	static uint8_t udtrans_init_step = 1;
	static fp32 UPTrans_init_time = 0;
	static fp32 angle_out;
	//��һ�����ָ��������ס����״̬
	if(udtrans_init_step == 1)  
	{
		//���׻ص���ʼ��״̬   
		Clamp_on();                  //��ȡװ�üн� 
		stretch_close();             //��ȡװ���ջ� 
		rescue_close();              //��Ԯ�ջ� 
		rotary_actuator_close();     //���Ϲر� 
		//����ص���ʼ��״̬
		//�޸ģ�ֻ�ָ�ͼ��������������Ӧ��Ҳͬʱ�رգ���Ϊ�˷�ֹ������ȣ������йرմ���		
//    supply1_close();             //����1�ر�
//    supply2_close();             //����2�ر�		
    camera_mid();                //ͼ����				
		//��ʹ����Ƕȹ��㣨ֻ����һ�Σ���ȷ����������ʼ������׼ȷ
		udtrans_init_control->upmotor_L.motor_angle   = 0;
		udtrans_init_control->upmotor_R.motor_angle   = 0;
    udtrans_init_control->rotatemotor.motor_angle = 0;
		udtrans_init_control->goldmotor.motor_angle   = 0;
		udtrans_init_control->snagmotor_L.motor_angle = 0;
		udtrans_init_control->snagmotor_R.motor_angle = 0;
		//���е�2��ʱ����������ֲ�����ȷ������������׼ȷ
	  udtrans_init_control->upmotor_R.motor_angle_set   = udtrans_init_control->upmotor_R.motor_angle;  
	  udtrans_init_control->upmotor_L.motor_angle_set   = udtrans_init_control->upmotor_L.motor_angle; 
	  udtrans_init_control->rotatemotor.motor_angle_set = udtrans_init_control->rotatemotor.motor_angle;	
	  udtrans_init_control->goldmotor.motor_angle_set   = udtrans_init_control->goldmotor.motor_angle;	
	  udtrans_init_control->snagmotor_L.motor_angle_set = udtrans_init_control->snagmotor_L.motor_angle;	
	  udtrans_init_control->snagmotor_R.motor_angle_set = udtrans_init_control->snagmotor_R.motor_angle;	
		udtrans_init_step = 2;
  }	
	//�ڶ�������ʼ�����̧�������Ѱ����λ�㣬ʹ�Ƕȹ���
	else if(udtrans_init_step == 2)
	{
		//���̧�����������ת
	  udtrans_init_control->goldmotor.motor_angle_set -= 10; 
		//�ж��Ƿ񵽴���λ��   ע��fabs�󸡵�������ֵ��abs����������ֵ
	  if(fabs(udtrans_init_control->goldmotor.motor_angle_set - udtrans_init_control->goldmotor.motor_angle) > 5000)   
		{
		  UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000 || IF_MOUSE_PRESSED_RIGH)   //���������������˲���
		{
			//��ʱ�ҵ�������Ϊ��
		  udtrans_init_control->goldmotor.motor_angle = 0;
		  udtrans_init_control->goldmotor.motor_angle_set = 0;
	    udtrans_init_step = 3;
			UPTrans_init_time = 0;
		}

	}
	//����������ʼ��̧�������Ѱ����λ�㣬ʹ�Ƕȹ���
	else if(udtrans_init_step == 3)
	{	
		//̧�����������ת
	  udtrans_init_control->upmotor_L.motor_angle_set -= 10; 
		//�ж��Ƿ񵽴���λ��   ע��fabs�󸡵�������ֵ��abs����������ֵ
	  if(fabs(udtrans_init_control->upmotor_L.motor_angle_set - udtrans_init_control->upmotor_L.motor_angle) > 3000)   
		{
		  UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000 || IF_MOUSE_PRESSED_RIGH)   //���������������˲���
		{
			//��ʱ�ҵ�������Ϊ��
		  udtrans_init_control->upmotor_L.motor_angle = 0;
		  udtrans_init_control->upmotor_L.motor_angle_set = 0;
			udtrans_init_control->upmotor_R.motor_angle = 0;
		  udtrans_init_control->upmotor_R.motor_angle_set = 0;
	    udtrans_init_step = 4;
			UPTrans_init_time = 0;
		}
	}
	//���Ĳ�����ʼ����ȡװ����ת�����Ѱ����λ�㣬ʹ�Ƕȹ���
	else if(udtrans_init_step == 4)
	{
		//��ȡװ����ת���������ת
		udtrans_init_control->rotatemotor.motor_angle_set -= 10; 
		//�ж��Ƿ񵽴���λ��
	  if(fabs(udtrans_init_control->rotatemotor.motor_angle_set - udtrans_init_control->rotatemotor.motor_angle) > 3000)
		{
			UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000 || IF_MOUSE_PRESSED_RIGH)    //���������������˲���
		{
			//��ʱ�ҵ�������Ϊ��
		  udtrans_init_control->rotatemotor.motor_angle = 0;
		  udtrans_init_control->rotatemotor.motor_angle_set = 0;
			//��ʼ���������л�Ϊֹͣģʽ����ʼ����������Ϊ1����ʱ����
      udtrans_init_control->uptrans_behaviour = UPTRANS_STOP;
	    udtrans_init_step = 1;
			UPTrans_init_time = 0;
		}
	}
	
#if 0   //�ϰ�������ʼ����Ŀǰû���õ�
	//�ڶ�������ʼ���ϰ�������Ѱ����λ�㣬ʹ�Ƕȹ���
	else if(udtrans_init_step == 2)
	{
		//̧�����������ת
	  udtrans_init_control->snagmotor_L.motor_angle_set -= 10; 
		//�ж��Ƿ񵽴���λ��   ע��fabs�󸡵�������ֵ��abs����������ֵ
	  if(fabs(udtrans_init_control->snagmotor_L.motor_angle_set - udtrans_init_control->snagmotor_L.motor_angle) > 3000)   
		{
		  UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000)
		{
			//��ʱ�ҵ�������Ϊ��
		  udtrans_init_control->snagmotor_L.motor_angle = 0;
		  udtrans_init_control->snagmotor_L.motor_angle_set = 0;
			udtrans_init_control->snagmotor_R.motor_angle = 0;
		  udtrans_init_control->snagmotor_R.motor_angle_set = 0;
	    udtrans_init_step = 3;
			UPTrans_init_time = 0;
		}
	}
#endif	

	//̧�����PID����
	angle_out = PID_Calc(&udtrans_init_control->upmotor_L.motor_angle_pid, udtrans_init_control->upmotor_L.motor_angle_set, udtrans_init_control->upmotor_L.motor_angle);
	udtrans_init_control->upmotor_L.given_current = PID_Calc(&udtrans_init_control->upmotor_L.motor_speed_pid, udtrans_init_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��
	angle_out = PID_Calc(&udtrans_init_control->upmotor_R.motor_angle_pid, -udtrans_init_control->upmotor_L.motor_angle_set, udtrans_init_control->upmotor_R.motor_angle);
	udtrans_init_control->upmotor_R.given_current = PID_Calc(&udtrans_init_control->upmotor_R.motor_speed_pid, udtrans_init_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_init_control->rotatemotor.motor_angle_pid, udtrans_init_control->rotatemotor.motor_angle_set, udtrans_init_control->rotatemotor.motor_angle);
	udtrans_init_control->rotatemotor.given_current = PID_Calc(&udtrans_init_control->rotatemotor.motor_speed_pid, udtrans_init_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	angle_out = PID_Calc(&udtrans_init_control->goldmotor.motor_angle_pid, udtrans_init_control->goldmotor.motor_angle_set, udtrans_init_control->goldmotor.motor_angle);
	udtrans_init_control->goldmotor.given_current = PID_Calc(&udtrans_init_control->goldmotor.motor_speed_pid, udtrans_init_control->goldmotor.motor_speed, angle_out);	
  //�ϰ�����PID����
	angle_out = PID_Calc(&udtrans_init_control->snagmotor_L.motor_angle_pid, udtrans_init_control->snagmotor_L.motor_angle_set, udtrans_init_control->snagmotor_L.motor_angle);
	udtrans_init_control->snagmotor_L.given_current = PID_Calc(&udtrans_init_control->snagmotor_L.motor_speed_pid, udtrans_init_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_init_control->snagmotor_R.motor_angle_pid, -udtrans_init_control->snagmotor_L.motor_angle_set, udtrans_init_control->snagmotor_R.motor_angle);
	udtrans_init_control->snagmotor_R.given_current = PID_Calc(&udtrans_init_control->snagmotor_R.motor_speed_pid, udtrans_init_control->snagmotor_R.motor_speed, angle_out);
}

//ֻ�賤��CTRL�������л����ֶ�ģʽ
static void UPTrans_manual_control(UDTrans_Control_t *udtrans_manual_control)
{
	static fp32 angle_out;
	static fp32 vx_set_channel;
	int16_t vx_channel;	 //ң����ԭʼͨ��ֵ
	
	//�������ƣ���Ϊң�������ܴ��ڲ��죬ҡ�����м䣬��ֵ��Ϊ0
	rc_deadline_limit(udtrans_manual_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//ͨ��ֵ��ֵ
	vx_set_channel = vx_channel;
	
	//ң������ͨ�����£����Ƽ�ȡװ����ת���
	if(switch_is_down(udtrans_manual_control->udtrans_rc_ctrl->rc.s[1]))
	{
		//����Ŀ��Ƕȣ��ۼӣ�
    udtrans_manual_control->rotatemotor.motor_angle_set += vx_set_channel*0.3f; 
	}
	//ң������ͨ�����У�����̧�����
	if(switch_is_mid(udtrans_manual_control->udtrans_rc_ctrl->rc.s[1]))
	{
		//����Ŀ��Ƕȣ��ۼӣ�
    udtrans_manual_control->upmotor_L.motor_angle_set += vx_set_channel*0.3f; 
	}
	//ң������ͨ�����ϣ����ƽ����
	if(switch_is_up(udtrans_manual_control->udtrans_rc_ctrl->rc.s[1]))
	{
		//����Ŀ��Ƕȣ��ۼӣ�
    udtrans_manual_control->goldmotor.motor_angle_set += vx_set_channel*0.3f; 
	}
	
	//���������
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time < RC_KEY_LONG_TIME)
		{
			udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time++;
			if(udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time == RC_KEY_LONG_TIME)
			{
				switch (press_L_step)         
				{
					case 0:
					{
            Clamp_close();
						press_L_step = 1;
						break;
					}
					case 1:
					{
            Clamp_on();
						press_L_step = 0;
						break;
					}
				}
				udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
			}
		}
	}
	else
	{
		udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
	}
	
	//����Ҽ����
	if (IF_MOUSE_PRESSED_RIGH)
	{
		if (udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time < RC_KEY_LONG_TIME)
		{
			udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time++;
			if(udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time == RC_KEY_LONG_TIME)
			{
				//����Ҽ����	
				switch (press_R_step)         
				{
					case 0:
					{
            stretch_on();
						press_R_step = 1;
						break;
					}
					case 1:
					{
            stretch_close();
						press_R_step = 0;
						break;
					}
				}
				udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
			}
		}
	}
	else
	{
		udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
	}

  //�޷�����
	udtrans_manual_control->upmotor_L.motor_angle_set=fp32_constrain_upmotor_angle(&udtrans_manual_control->upmotor_L);
	udtrans_manual_control->rotatemotor.motor_angle_set=fp32_constrain_upmotor_angle(&udtrans_manual_control->rotatemotor);
	udtrans_manual_control->goldmotor.motor_angle_set=fp32_constrain_upmotor_angle(&udtrans_manual_control->goldmotor);

	//̧�����PID����
	angle_out = PID_Calc(&udtrans_manual_control->upmotor_L.motor_angle_pid, udtrans_manual_control->upmotor_L.motor_angle_set, udtrans_manual_control->upmotor_L.motor_angle);
	udtrans_manual_control->upmotor_L.given_current = PID_Calc(&udtrans_manual_control->upmotor_L.motor_speed_pid, udtrans_manual_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��
	angle_out = PID_Calc(&udtrans_manual_control->upmotor_R.motor_angle_pid,-udtrans_manual_control->upmotor_L.motor_angle_set, udtrans_manual_control->upmotor_R.motor_angle);
	udtrans_manual_control->upmotor_R.given_current = PID_Calc(&udtrans_manual_control->upmotor_R.motor_speed_pid, udtrans_manual_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_manual_control->rotatemotor.motor_angle_pid, udtrans_manual_control->rotatemotor.motor_angle_set, udtrans_manual_control->rotatemotor.motor_angle);
	udtrans_manual_control->rotatemotor.given_current = PID_Calc(&udtrans_manual_control->rotatemotor.motor_speed_pid, udtrans_manual_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	angle_out = PID_Calc(&udtrans_manual_control->goldmotor.motor_angle_pid, udtrans_manual_control->goldmotor.motor_angle_set, udtrans_manual_control->goldmotor.motor_angle);
	udtrans_manual_control->goldmotor.given_current = PID_Calc(&udtrans_manual_control->goldmotor.motor_speed_pid, udtrans_manual_control->goldmotor.motor_speed, angle_out);	
  //�ϰ�����PID����
	angle_out = PID_Calc(&udtrans_manual_control->snagmotor_L.motor_angle_pid, udtrans_manual_control->snagmotor_L.motor_angle_set, udtrans_manual_control->snagmotor_L.motor_angle);
	udtrans_manual_control->snagmotor_L.given_current = PID_Calc(&udtrans_manual_control->snagmotor_L.motor_speed_pid, udtrans_manual_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_manual_control->snagmotor_R.motor_angle_pid, udtrans_manual_control->snagmotor_R.motor_angle_set, udtrans_manual_control->snagmotor_R.motor_angle);
	udtrans_manual_control->snagmotor_R.given_current = PID_Calc(&udtrans_manual_control->snagmotor_R.motor_speed_pid, udtrans_manual_control->snagmotor_R.motor_speed, angle_out);	
}

static void UPTrans_stop_control(UDTrans_Control_t *udtrans_stop_control)
{
 	static fp32 angle_out;
	static fp32 upmotor_L_stop_angle,upmotor_R_stop_angle,rotatemotor_stop_angle;
	static fp32 goldmotor_stop_angle,snagmotor_L_stop_angle,snagmotor_R_stop_angle;

	if(stop_step == 1)     //stop_step��ֹͣ���裩ȫ�ֱ�����������ģʽӰ��
	{
		//ֻ����һ����ͣλ�ñ���
		upmotor_L_stop_angle   = udtrans_stop_control->upmotor_L.motor_angle;
		upmotor_R_stop_angle   = udtrans_stop_control->upmotor_R.motor_angle;
		rotatemotor_stop_angle = udtrans_stop_control->rotatemotor.motor_angle;
		goldmotor_stop_angle   = udtrans_stop_control->goldmotor.motor_angle;
		snagmotor_L_stop_angle = udtrans_stop_control->snagmotor_L.motor_angle;
		snagmotor_R_stop_angle = udtrans_stop_control->snagmotor_R.motor_angle;
		stop_step = 0;       //ֻ�����л�ģʽ��Ż���0
	}
	
	udtrans_stop_control->upmotor_L.motor_angle_set   = upmotor_L_stop_angle;
  udtrans_stop_control->upmotor_R.motor_angle_set   = upmotor_R_stop_angle;
  udtrans_stop_control->rotatemotor.motor_angle_set = rotatemotor_stop_angle;
  udtrans_stop_control->goldmotor.motor_angle_set   = goldmotor_stop_angle;
  udtrans_stop_control->snagmotor_L.motor_angle_set = snagmotor_L_stop_angle;
  udtrans_stop_control->snagmotor_R.motor_angle_set = snagmotor_R_stop_angle;
	
	//̧�����PID����
	angle_out = PID_Calc(&udtrans_stop_control->upmotor_L.motor_angle_pid, udtrans_stop_control->upmotor_L.motor_angle_set, udtrans_stop_control->upmotor_L.motor_angle);
	udtrans_stop_control->upmotor_L.given_current = PID_Calc(&udtrans_stop_control->upmotor_L.motor_speed_pid, udtrans_stop_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��
	angle_out = PID_Calc(&udtrans_stop_control->upmotor_R.motor_angle_pid, udtrans_stop_control->upmotor_R.motor_angle_set, udtrans_stop_control->upmotor_R.motor_angle);
	udtrans_stop_control->upmotor_R.given_current = PID_Calc(&udtrans_stop_control->upmotor_R.motor_speed_pid, udtrans_stop_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_stop_control->rotatemotor.motor_angle_pid, udtrans_stop_control->rotatemotor.motor_angle_set, udtrans_stop_control->rotatemotor.motor_angle);
	udtrans_stop_control->rotatemotor.given_current = PID_Calc(&udtrans_stop_control->rotatemotor.motor_speed_pid, udtrans_stop_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	angle_out = PID_Calc(&udtrans_stop_control->goldmotor.motor_angle_pid, udtrans_stop_control->goldmotor.motor_angle_set, udtrans_stop_control->goldmotor.motor_angle);
	udtrans_stop_control->goldmotor.given_current = PID_Calc(&udtrans_stop_control->goldmotor.motor_speed_pid, udtrans_stop_control->goldmotor.motor_speed, angle_out);	
  //�ϰ�����PID����
	angle_out = PID_Calc(&udtrans_stop_control->snagmotor_L.motor_angle_pid, udtrans_stop_control->snagmotor_L.motor_angle_set, udtrans_stop_control->snagmotor_L.motor_angle);
	udtrans_stop_control->snagmotor_L.given_current = PID_Calc(&udtrans_stop_control->snagmotor_L.motor_speed_pid, udtrans_stop_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_stop_control->snagmotor_R.motor_angle_pid, udtrans_stop_control->snagmotor_R.motor_angle_set, udtrans_stop_control->snagmotor_R.motor_angle);
	udtrans_stop_control->snagmotor_R.given_current = PID_Calc(&udtrans_stop_control->snagmotor_R.motor_speed_pid, udtrans_stop_control->snagmotor_R.motor_speed, angle_out);
}

static void UPTrans_recover_control(UDTrans_Control_t *udtrans_recover_control)
{
	static fp32 angle_out;
	static uint8_t recover_step = 1;
  static fp32    recover_time = 0; 

#if 0	
	//��һ�����ָ��������ס����״̬
	if(recover_step == 1)  
	{
		//���׻ص���ʼ��״̬   ���⣺����һ�����ջ��Ƿ���Ӱ�죿
		Clamp_on();                  //��ȡװ���ɿ� 
		stretch_close();             //��ȡװ���ջ� 
		rescue_close();              //��Ԯ�ջ� 
		rotary_actuator_close();     //���Ϲر� 
		//����ص���ʼ��״̬ 
		recover_step = 2;
	}
	//�ڶ��������̧������
	else if(recover_step == 2)
	{
		udtrans_recover_control->goldmotor.motor_angle_set = udtrans_recover_control->goldmotor.min_angle_limit;    
		if(udtrans_recover_control->goldmotor.set_ramp_angle != udtrans_recover_control->goldmotor.motor_angle_set)
		{
			udtrans_recover_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_recover_control->goldmotor.motor_angle_set, udtrans_recover_control->goldmotor.set_ramp_angle, 300);
		}	
		
		if(udtrans_recover_control->goldmotor.set_ramp_angle == udtrans_recover_control->goldmotor.motor_angle_set )
		{
			recover_step = 3;
		}	
	}	
	//��������̧�����£���ȡװ����ת��λ 
	else if(recover_step == 3)
	{
		//̧������
		udtrans_recover_control->upmotor_L.motor_angle_set = udtrans_recover_control->upmotor_L.min_angle_limit;    
		if(udtrans_recover_control->upmotor_L.set_ramp_angle != udtrans_recover_control->upmotor_L.motor_angle_set)
		{
			udtrans_recover_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_recover_control->upmotor_L.motor_angle_set, udtrans_recover_control->upmotor_L.set_ramp_angle, 300);
		}
		//��ȡװ����ת��λ
		udtrans_recover_control->rotatemotor.motor_angle_set = udtrans_recover_control->rotatemotor.min_angle_limit; 
		if(udtrans_recover_control->rotatemotor.set_ramp_angle != udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			udtrans_recover_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_recover_control->rotatemotor.motor_angle_set, udtrans_recover_control->rotatemotor.set_ramp_angle, 200);
		}
		//�жϻָ��Ƿ����
		if(udtrans_recover_control->upmotor_L.set_ramp_angle == udtrans_recover_control->upmotor_L.motor_angle_set &&
			 udtrans_recover_control->rotatemotor.set_ramp_angle == udtrans_recover_control->rotatemotor.motor_angle_set)
		{
      recover_step = 4;
		}	
	}
	//���Ĳ����ϰ�������λ
	else if(recover_step == 4)
	{
		udtrans_recover_control->snagmotor_L.motor_angle_set = udtrans_recover_control->snagmotor_L.min_angle_limit;    
		if(udtrans_recover_control->snagmotor_L.set_ramp_angle != udtrans_recover_control->snagmotor_L.motor_angle_set)
		{
			udtrans_recover_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_recover_control->snagmotor_L.motor_angle_set, udtrans_recover_control->snagmotor_L.set_ramp_angle, 50);
		}	
		
		if(udtrans_recover_control->snagmotor_L.set_ramp_angle == udtrans_recover_control->snagmotor_L.motor_angle_set )
		{
			//ѭ��һ��ʱ�䣬��֤�ָ�����ʼλ��
			recover_time++;
			if(recover_time == 500) 
			{
				//�ָ���ɣ��л�Ϊֹͣģʽ���ָ���������Ϊ1
				udtrans_recover_control->uptrans_behaviour = UPTRANS_STOP; 
				recover_step = 1;		
        recover_time = 0;			
			}
		}	
	}	
#else
	//��һ����ץ�ֵ����̧��
	if(recover_step == 1) 
	{
		//��ȡװ����ת��λ
		udtrans_recover_control->rotatemotor.motor_angle_set = udtrans_recover_control->rotatemotor.mid_angle_limit; 
		if(udtrans_recover_control->rotatemotor.set_ramp_angle != udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			udtrans_recover_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_recover_control->rotatemotor.motor_angle_set, udtrans_recover_control->rotatemotor.set_ramp_angle, 
			                                                                 HAND_FAST);
		}
		if(udtrans_recover_control->rotatemotor.set_ramp_angle == udtrans_recover_control->rotatemotor.motor_angle_set)
		{
      recover_step = 2;		
		}		
	}
	//�ڶ������ָ��������ס����״̬
	if(recover_step == 2)  
	{
    recover_time++;
		//���׻ص���ʼ��״̬   ���⣺����һ�����ջ��Ƿ���Ӱ�죿
		Clamp_on();                  //��ȡװ���ɿ� 
		stretch_close();             //��ȡװ���ջ� 
		rescue_close();              //��Ԯ�ջ� 
		rotary_actuator_close();     //���Ϲر� 
		//����ص���ʼ��״̬ 
		//�޸ģ�ֻ�ָ�ͼ��������������Ӧ��Ҳͬʱ�رգ���Ϊ�˷�ֹ������ȣ������йرմ��� ����ԭ�򣺻�е��װ����ʱ���Ȳ��������������ȱ��
//    supply1_close();             //����1�ر�
//    supply2_close();             //����2�ر�		
    camera_mid();                //ͼ����		
		//�ȴ�һ��ʱ�������һ��
		if(recover_time == 500)
		{	
      recover_step = 3;						
			recover_time = 0;
		}
	}
	//�����������̧������
	else if(recover_step == 3)
	{
		udtrans_recover_control->goldmotor.motor_angle_set = udtrans_recover_control->goldmotor.min_angle_limit;    
		if(udtrans_recover_control->goldmotor.set_ramp_angle != udtrans_recover_control->goldmotor.motor_angle_set)
		{
			udtrans_recover_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_recover_control->goldmotor.motor_angle_set, udtrans_recover_control->goldmotor.set_ramp_angle, 
			                                                               GOLD_UP_SPEED);
		}	
		
		if(udtrans_recover_control->goldmotor.set_ramp_angle == udtrans_recover_control->goldmotor.motor_angle_set )
		{
			recover_step = 4;
		}	
	}	
	//���Ĳ���̧�����£���ȡװ����ת��λ 
	else if(recover_step == 4)
	{
		//̧������
		udtrans_recover_control->upmotor_L.motor_angle_set = udtrans_recover_control->upmotor_L.min_angle_limit;    
		if(udtrans_recover_control->upmotor_L.set_ramp_angle != udtrans_recover_control->upmotor_L.motor_angle_set)
		{
			udtrans_recover_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_recover_control->upmotor_L.motor_angle_set, udtrans_recover_control->upmotor_L.set_ramp_angle, 
			                                                               TRANS_UP_SPEED);
		}
		//��ȡװ����ת��λ
		udtrans_recover_control->rotatemotor.motor_angle_set = udtrans_recover_control->rotatemotor.min_angle_limit; 
		if(udtrans_recover_control->rotatemotor.set_ramp_angle != udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			udtrans_recover_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_recover_control->rotatemotor.motor_angle_set, udtrans_recover_control->rotatemotor.set_ramp_angle,
		                                                                 	 HAND_FAST);
		}
		//�жϻָ��Ƿ����
		if(udtrans_recover_control->upmotor_L.set_ramp_angle == udtrans_recover_control->upmotor_L.motor_angle_set &&
			 udtrans_recover_control->rotatemotor.set_ramp_angle == udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			//ѭ��һ��ʱ�䣬��֤�ָ�����ʼλ��
			recover_time++;
			if(recover_time == 500) 
			{
				//�ָ���ɣ��л�Ϊֹͣģʽ���ָ���������Ϊ1
				udtrans_recover_control->uptrans_behaviour = UPTRANS_STOP; 
				recover_step = 1;		
        recover_time = 0;			
			}
		}	
	}

#endif
	
  //̧�����PID����
	angle_out = PID_Calc(&udtrans_recover_control->upmotor_L.motor_angle_pid, udtrans_recover_control->upmotor_L.set_ramp_angle, udtrans_recover_control->upmotor_L.motor_angle);
	udtrans_recover_control->upmotor_L.given_current = PID_Calc(&udtrans_recover_control->upmotor_L.motor_speed_pid, udtrans_recover_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_recover_control->upmotor_R.motor_angle_pid,-udtrans_recover_control->upmotor_L.set_ramp_angle, udtrans_recover_control->upmotor_R.motor_angle);
	udtrans_recover_control->upmotor_R.given_current = PID_Calc(&udtrans_recover_control->upmotor_R.motor_speed_pid, udtrans_recover_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_recover_control->rotatemotor.motor_angle_pid, udtrans_recover_control->rotatemotor.set_ramp_angle, udtrans_recover_control->rotatemotor.motor_angle);
	udtrans_recover_control->rotatemotor.given_current = PID_Calc(&udtrans_recover_control->rotatemotor.motor_speed_pid, udtrans_recover_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	angle_out = PID_Calc(&udtrans_recover_control->goldmotor.motor_angle_pid, udtrans_recover_control->goldmotor.set_ramp_angle, udtrans_recover_control->goldmotor.motor_angle);
	udtrans_recover_control->goldmotor.given_current = PID_Calc(&udtrans_recover_control->goldmotor.motor_speed_pid, udtrans_recover_control->goldmotor.motor_speed, angle_out);
  //�ϰ�����PID����
	angle_out = PID_Calc(&udtrans_recover_control->snagmotor_L.motor_angle_pid, udtrans_recover_control->snagmotor_L.set_ramp_angle, udtrans_recover_control->snagmotor_L.motor_angle);
	udtrans_recover_control->snagmotor_L.given_current = PID_Calc(&udtrans_recover_control->snagmotor_L.motor_speed_pid, udtrans_recover_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_recover_control->snagmotor_R.motor_angle_pid,-udtrans_recover_control->snagmotor_L.set_ramp_angle, udtrans_recover_control->snagmotor_R.motor_angle);
	udtrans_recover_control->snagmotor_R.given_current = PID_Calc(&udtrans_recover_control->snagmotor_R.motor_speed_pid, udtrans_recover_control->snagmotor_R.motor_speed, angle_out);
}

static void UPTrans_snag_control(UDTrans_Control_t *udtrans_snag_control)
{
	static fp32 angle_out;
	static fp32 vx_set_channel;  //
	int16_t vx_channel;	         //ң����ԭʼͨ��ֵ
	
	//��һ�����ϰ�������λ��
	if (trans_step == 1)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.max_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}		
		else
		{
		  trans_step = 2;	
		}
	}
	//�ڶ������ϰ�������ǰ�����ƶ��������ϰ�ץ�ִ��أ�ʹ��ͬ��
	else if(trans_step == 2)
	{
		//̧�����������ת
	  udtrans_snag_control->snagmotor_L.motor_angle_set += 20; 
		//�ж��Ƿ񵽴���λ��   ע��fabs�󸡵�������ֵ��abs����������ֵ
	  if(fabs(udtrans_snag_control->snagmotor_L.motor_angle_set - udtrans_snag_control->snagmotor_L.motor_angle) > 5000)   
		{
		  trans_time++;
		}
		if(trans_time == 1000)
		{
	    udtrans_snag_control->snagmotor_L.MAX_angle_limit = udtrans_snag_control->snagmotor_L.motor_angle;
	    trans_step = 3;
			trans_time = 0;
		}
	}
	//���������ֶ�����λ��
	else if (trans_step == 3)
	{
		rc_deadline_limit(udtrans_snag_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
    //ͨ��ֵ��ֵ
		vx_set_channel = vx_channel;
		//����Ŀ��Ƕȣ��ۼӣ�
		udtrans_snag_control->snagmotor_L.set_ramp_angle += vx_set_channel*0.3f;
    //�޷�����
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle > udtrans_snag_control->snagmotor_L.max_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.max_angle_limit;
		}
		else if(udtrans_snag_control->snagmotor_L.set_ramp_angle < udtrans_snag_control->snagmotor_L.min_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.min_angle_limit;
		}	
		
		//������ϣ��ٰ���V��������һ��
		if (udtrans_snag_control->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
		{
			if (udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time < RC_KEY_LONG_TIME)
			{
				udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time++;
				if(udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time == RC_KEY_LONG_TIME)
				{
					trans_step = 4;
					udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
				}
			}
		}
		else
		{
			udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
		}
	}
	//���Ĳ����ϰ�����м�λ�ã������ϰ��飩
	else if (trans_step == 4)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.mid_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}	
		//�������λ�ã��ٰ���B��������һ��
		if (udtrans_snag_control->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
		{
			if (udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time < RC_KEY_LONG_TIME)
			{
				udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time++;
				if(udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time == RC_KEY_LONG_TIME)
				{
					trans_step = 5;
					udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time = 0;
				}
			}
		}
		else
		{
			udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time = 0;
		}		
	}
	//���岽���ϰ�������λ�ã������ϰ��飩
	else if (trans_step == 5)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.max_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}		
		else
		{
		  trans_step = 6;	
		}
	}
	//���������ֶ�����
	else if (trans_step == 6)
	{	

		rc_deadline_limit(udtrans_snag_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
    //ͨ��ֵ��ֵ
		vx_set_channel = vx_channel;
		//����Ŀ��Ƕȣ��ۼӣ�
		udtrans_snag_control->snagmotor_L.set_ramp_angle += vx_set_channel*0.3f;
    //�޷�����
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle > udtrans_snag_control->snagmotor_L.max_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.max_angle_limit;
		}
		else if(udtrans_snag_control->snagmotor_L.set_ramp_angle < udtrans_snag_control->snagmotor_L.min_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.min_angle_limit;
		}	
		
		//������ϣ��ٰ���V��������һ��
		if (udtrans_snag_control->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
		{
			if (udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time < RC_KEY_LONG_TIME)
			{
				udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time++;
				if(udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time == RC_KEY_LONG_TIME)
				{
					trans_step = 7;
					udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
				}
			}
		}
		else
		{
			udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
		}
	}
	//���߲����ϰ�����ջ�
	else if (trans_step == 7)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.min_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}		
		else
		{
			trans_time++;
			//ѭ��һ��ʱ�䣬��֤�ϰ������ص���ʼλ��
			if( trans_time == 500 )  
			{
				//������ɣ��л�Ϊֹͣģʽ�����ϲ�������Ϊ1����ʱ����
				udtrans_snag_control->uptrans_behaviour = UPTRANS_STOP; 
				trans_step = 1;		
        trans_time = 0;			
			}
		}
	}
	
	//�޷�����
	if (udtrans_snag_control->snagmotor_L.set_ramp_angle > udtrans_snag_control->snagmotor_L.MAX_angle_limit)
	{
		udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.MAX_angle_limit;
	}
	else if (udtrans_snag_control->snagmotor_L.set_ramp_angle < udtrans_snag_control->snagmotor_L.min_angle_limit)
	{
		udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.min_angle_limit;
	}
	//̧�����PID����
	//����ȡ���Ϲ������ò������̧��������Ƕ����ò��䣨motor_angle_set��
	angle_out = PID_Calc(&udtrans_snag_control->upmotor_L.motor_angle_pid, udtrans_snag_control->upmotor_L.motor_angle_set, udtrans_snag_control->upmotor_L.motor_angle);
	udtrans_snag_control->upmotor_L.given_current = PID_Calc(&udtrans_snag_control->upmotor_L.motor_speed_pid, udtrans_snag_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	//upmotor_R.motor_angle_set���ܲ���ı䣬����ֱ��ʹupmotor_R.motor_angle_setȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_snag_control->upmotor_R.motor_angle_pid,-udtrans_snag_control->upmotor_L.motor_angle_set, udtrans_snag_control->upmotor_R.motor_angle);
	udtrans_snag_control->upmotor_R.given_current = PID_Calc(&udtrans_snag_control->upmotor_R.motor_speed_pid, udtrans_snag_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
  //���ڳ��Ϲ������ò������̧��������Ƕ����ò��䣨motor_angle_set��
	angle_out = PID_Calc(&udtrans_snag_control->rotatemotor.motor_angle_pid, udtrans_snag_control->rotatemotor.motor_angle_set, udtrans_snag_control->rotatemotor.motor_angle);
	udtrans_snag_control->rotatemotor.given_current = PID_Calc(&udtrans_snag_control->rotatemotor.motor_speed_pid, udtrans_snag_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	//����ȡ���Ϲ������ò������̧��������Ƕ����ò��䣨motor_angle_set��
	angle_out = PID_Calc(&udtrans_snag_control->goldmotor.motor_angle_pid, udtrans_snag_control->goldmotor.motor_angle_set, udtrans_snag_control->goldmotor.motor_angle);
	udtrans_snag_control->goldmotor.given_current = PID_Calc(&udtrans_snag_control->goldmotor.motor_speed_pid, udtrans_snag_control->goldmotor.motor_speed, angle_out);	
  //�ϰ�����PID����
	angle_out = PID_Calc(&udtrans_snag_control->snagmotor_L.motor_angle_pid, udtrans_snag_control->snagmotor_L.set_ramp_angle, udtrans_snag_control->snagmotor_L.motor_angle);
	udtrans_snag_control->snagmotor_L.given_current = PID_Calc(&udtrans_snag_control->snagmotor_L.motor_speed_pid, udtrans_snag_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��		
	angle_out = PID_Calc(&udtrans_snag_control->snagmotor_R.motor_angle_pid,-udtrans_snag_control->snagmotor_L.set_ramp_angle, udtrans_snag_control->snagmotor_R.motor_angle);
	udtrans_snag_control->snagmotor_R.given_current = PID_Calc(&udtrans_snag_control->snagmotor_R.motor_speed_pid, udtrans_snag_control->snagmotor_R.motor_speed, angle_out);
}

//����ģʽ
static void UPTrans_supply_control(UDTrans_Control_t *udtrans_supply_control)
{
	//���������
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time < RC_KEY_LONG_TIME)
		{
			udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time++;
			if(udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time == RC_KEY_LONG_TIME)
			{
				supply1_on();         //����1����
				supply1_step = 2;	
				udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
			}
		}
	}
	else
	{
		udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
	}
	
	//����Ҽ����
	if (IF_MOUSE_PRESSED_RIGH)
	{
		if (udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time < RC_KEY_LONG_TIME)
		{
			udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time++;
			if(udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time == RC_KEY_LONG_TIME)
			{
				supply2_on();        //����2����
				supply2_step = 2;	
				udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
			}
		}
	}
	else
	{
		udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
	}
	
}

//��Ԯģʽ  Ԥ����λ�ã���еû��������һ���ܣ�ֻ��ˢ����Ԯ
static void UPTrans_help_control(UDTrans_Control_t *udtrans_help_control)
{
	static fp32 angle_out;	
//	static fp32 vx_set_channel;
//	int16_t vx_channel;	 //ң����ԭʼͨ��ֵ
//	
//	rc_deadline_limit(udtrans_help_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
//	//ͨ��ֵ��ֵ
//	vx_set_channel = vx_channel;
	
	
	//̧�����PID����
	angle_out = PID_Calc(&udtrans_help_control->upmotor_L.motor_angle_pid, udtrans_help_control->upmotor_L.motor_angle_set, udtrans_help_control->upmotor_L.motor_angle);
	udtrans_help_control->upmotor_L.given_current = PID_Calc(&udtrans_help_control->upmotor_L.motor_speed_pid, udtrans_help_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_help_control->upmotor_R.motor_angle_pid,-udtrans_help_control->upmotor_L.motor_angle_set, udtrans_help_control->upmotor_R.motor_angle);
	udtrans_help_control->upmotor_R.given_current = PID_Calc(&udtrans_help_control->upmotor_R.motor_speed_pid, udtrans_help_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_help_control->rotatemotor.motor_angle_pid, udtrans_help_control->rotatemotor.motor_angle_set, udtrans_help_control->rotatemotor.motor_angle);
	udtrans_help_control->rotatemotor.given_current = PID_Calc(&udtrans_help_control->rotatemotor.motor_speed_pid, udtrans_help_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	angle_out = PID_Calc(&udtrans_help_control->goldmotor.motor_angle_pid, udtrans_help_control->goldmotor.motor_angle_set, udtrans_help_control->goldmotor.motor_angle);
	udtrans_help_control->goldmotor.given_current = PID_Calc(&udtrans_help_control->goldmotor.motor_speed_pid, udtrans_help_control->goldmotor.motor_speed, angle_out);	
  //�ϰ�����PID����
	angle_out = PID_Calc(&udtrans_help_control->snagmotor_L.motor_angle_pid, udtrans_help_control->snagmotor_L.motor_angle_set, udtrans_help_control->snagmotor_L.motor_angle);
	udtrans_help_control->snagmotor_L.given_current = PID_Calc(&udtrans_help_control->snagmotor_L.motor_speed_pid, udtrans_help_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��		
	angle_out = PID_Calc(&udtrans_help_control->snagmotor_R.motor_angle_pid,-udtrans_help_control->snagmotor_L.motor_angle_set, udtrans_help_control->snagmotor_R.motor_angle);
	udtrans_help_control->snagmotor_R.given_current = PID_Calc(&udtrans_help_control->snagmotor_R.motor_speed_pid, udtrans_help_control->snagmotor_R.motor_speed, angle_out);
}


//С��Դ����ȡ������̧��200����ȡ����Ҫ���
static void UPTrans_collect_tiny_control(UDTrans_Control_t *udtrans_collect_tiny_control)
{
	static fp32 angle_out;	
	static fp32 vx_set_channel;
	int16_t vx_channel;	      //ң����ԭʼͨ��ֵ
	
	rc_deadline_limit(udtrans_collect_tiny_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//ͨ��ֵ��ֵ
	vx_set_channel = vx_channel;
	
#ifdef COLLECT_CONVERSION_AUTO	 	
	//��һ������ȡװ�ô򿪣�̧�������̧��       
	if(trans_step == 1)      
	{
		Clamp_on();              //��ȡװ�ô�  
		camera_screen();         //ͼ������Ƶλ��
		
		//̧��
		udtrans_collect_tiny_control->upmotor_L.motor_angle_set = udtrans_collect_tiny_control->upmotor_L.max_angle_limit;   
		//set_ramp_angleΪ�����Ƕ�  
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle != udtrans_collect_tiny_control->upmotor_L.motor_angle_set)
		{
			//set_ramp_angle��������ΪĿ��Ƕ�
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->upmotor_L.motor_angle_set, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}	
		
    //���̧������Ӧλ�ã���ֹ������¿�ס		
		switch (collect_gold)         
		{
			case 0:
			{
	    	udtrans_collect_tiny_control->goldmotor.motor_angle_set = udtrans_collect_tiny_control->goldmotor.max_angle_limit; 		
				break;
			}
			case 1:
	 	 	{
		  	udtrans_collect_tiny_control->goldmotor.motor_angle_set = udtrans_collect_tiny_control->goldmotor.mid_angle_limit; 	
				break;
			}
			case 2:
			{
		  	udtrans_collect_tiny_control->goldmotor.motor_angle_set = udtrans_collect_tiny_control->goldmotor.min_angle_limit; 	
				break;
			}
		}
		
		if(udtrans_collect_tiny_control->goldmotor.set_ramp_angle != udtrans_collect_tiny_control->goldmotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->goldmotor.motor_angle_set, udtrans_collect_tiny_control->goldmotor.set_ramp_angle, 
			                                                                    GOLD_UP_SPEED);
		}
		
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle   == udtrans_collect_tiny_control->upmotor_L.motor_angle_set &&
			 udtrans_collect_tiny_control->goldmotor.set_ramp_angle   == udtrans_collect_tiny_control->goldmotor.motor_angle_set)
		{
			 trans_step = 2;
		}
	}
	//�ڶ�������ȡװ����ת�����λ��
	else if(trans_step == 2)  
	{
		//��ȡװ����ת�����λ��
		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = udtrans_collect_tiny_control->rotatemotor.max_angle_limit; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_FAST);
		}	

		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			 trans_step = 3;
		}			
	}		
	//���������ȴ������ֵ���λ�ã��ֶ����ڣ���������������ڶ���  
	else if(trans_step == 3)  
	{	
		//ң������ͨ�����£����Ƽ�ȡװ����ת���
		if(switch_is_down(udtrans_collect_tiny_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		//ң������ͨ�����У�����̧�����
		if(switch_is_mid(udtrans_collect_tiny_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		
		//����������������һ��
		if (IF_MOUSE_PRESSED_LEFT)
		{		
      trans_step = 4;
		}	
	}        
	//���Ĳ�����ȡװ�üн�           
	else if(trans_step == 4)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART)			Clamp_close();        //��ȡװ�üн� 
		if(trans_time == 2*COLLECT_TIME_PART)
		{
			trans_step = 5;
			trans_time = 0;		
		}
	}
	//���岽����ȡװ����ת�м�λ�ã����̻ζ������󣬰���������������һ��          
	else if(trans_step == 5)
	{
		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = udtrans_collect_tiny_control->rotatemotor.mid_angle_limit; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}

		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			//�γ����󣬰���������������һ��
			if (IF_MOUSE_PRESSED_LEFT)
			{
				switch (collect_gold)         
				{
					case 0:
					{
						collect_gold ++;    //ʰȡ������+1   
						trans_step = 6; 				
						break;
					}
					case 1:
					{
						collect_gold ++;    //ʰȡ������+1   
						trans_step = 6; 				
						break;
					}
					case 2:
					{
						collect_gold ++;    //ʰȡ������+1   
						trans_step = 8;
						break;
					}
				}				
			}
			else if(IF_KEY_PRESSED_Z)  
			{
			  trans_step = 1;   //û�ме�������Ҽ��ص��ڶ���
			} 
		}
	}
	//��������̧���䵽��Ӧλ�ã���ȡװ����ת��λ
	else if(trans_step == 6)
	{	
		switch (collect_gold)         
		{
			case 1:
			{
  	  	udtrans_collect_tiny_control->upmotor_L.motor_angle_set = udtrans_collect_tiny_control->upmotor_L.min_angle_limit;   
				break;
			}
			case 2:
			{
  	  	udtrans_collect_tiny_control->upmotor_L.motor_angle_set = udtrans_collect_tiny_control->upmotor_L.mid_angle_limit;  		
				break;
			}
		}	
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle != udtrans_collect_tiny_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->upmotor_L.motor_angle_set, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}			

		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = udtrans_collect_tiny_control->rotatemotor.min_angle_limit; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle   == udtrans_collect_tiny_control->upmotor_L.motor_angle_set   &&
		   udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set )
		{
			trans_step = 7;        
		} 
	}
	//���߲�����ȡװ���ɿ����������
	else if(trans_step == 7)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART) 	Clamp_on();       //��ȡװ���ɿ�
		if(trans_time == 2*COLLECT_TIME_PART) 
		{
			trans_step = 1;   //�ص���һ��
			trans_time = 0;				
		}	
	}
	//�ڰ˲���̧�����£������飩
	else if(trans_step == 8)
	{
    //̧������һ���߶ȣ���֤�ڲ�������鲻��˦��
	  udtrans_collect_tiny_control->upmotor_L.motor_angle_set = COLLECT_UP_GOLD_OVER; 	
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle != udtrans_collect_tiny_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->upmotor_L.motor_angle_set, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}
		
		//ץ�ֵ������߶�
		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = COLLECT_HAND_PICKUP_HEIGHT; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle == udtrans_collect_tiny_control->upmotor_L.motor_angle_set &&
			 udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{				
			trans_time++;
			//ѭ��һ��ʱ�䣬��̧֤���ص���ʼλ��
			if( trans_time == 500 )  
			{
				//���ʰȡ��ɣ��л�Ϊֹͣģʽ����������Ϊ1����ʱ���㣬���������ֵ��ʰȡ���������
				udtrans_collect_tiny_control->uptrans_behaviour = UPTRANS_STOP; 
				trans_step = 1;
				trans_time = 0;		
	  	  gold_number = collect_gold;	    
	    	collect_gold = 0;			           	
				camera_mid();           //ͼ���ص��м�λ��					
			}
		}
	}

#else
//����д�ڶ���ȡ������	
#endif

  //set_ramp_angle�޷�����
	udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_tiny_control->upmotor_L);
	udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_tiny_control->rotatemotor);
	
	//̧�����PID����
	angle_out = PID_Calc(&udtrans_collect_tiny_control->upmotor_L.motor_angle_pid, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, udtrans_collect_tiny_control->upmotor_L.motor_angle);
	udtrans_collect_tiny_control->upmotor_L.given_current = PID_Calc(&udtrans_collect_tiny_control->upmotor_L.motor_speed_pid, udtrans_collect_tiny_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_collect_tiny_control->upmotor_R.motor_angle_pid,-udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, udtrans_collect_tiny_control->upmotor_R.motor_angle);
	udtrans_collect_tiny_control->upmotor_R.given_current = PID_Calc(&udtrans_collect_tiny_control->upmotor_R.motor_speed_pid, udtrans_collect_tiny_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_collect_tiny_control->rotatemotor.motor_angle_pid, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, udtrans_collect_tiny_control->rotatemotor.motor_angle);
	udtrans_collect_tiny_control->rotatemotor.given_current = PID_Calc(&udtrans_collect_tiny_control->rotatemotor.motor_speed_pid, udtrans_collect_tiny_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	angle_out = PID_Calc(&udtrans_collect_tiny_control->goldmotor.motor_angle_pid, udtrans_collect_tiny_control->goldmotor.set_ramp_angle, udtrans_collect_tiny_control->goldmotor.motor_angle);
	udtrans_collect_tiny_control->goldmotor.given_current = PID_Calc(&udtrans_collect_tiny_control->goldmotor.motor_speed_pid, udtrans_collect_tiny_control->goldmotor.motor_speed, angle_out);	
  //�ϰ�����PID����
	//����ȡ���������ò����ϰ��������Ƕ����ò��䣨motor_angle_set��
	angle_out = PID_Calc(&udtrans_collect_tiny_control->snagmotor_L.motor_angle_pid, udtrans_collect_tiny_control->snagmotor_L.motor_angle_set, udtrans_collect_tiny_control->snagmotor_L.motor_angle);
	udtrans_collect_tiny_control->snagmotor_L.given_current = PID_Calc(&udtrans_collect_tiny_control->snagmotor_L.motor_speed_pid, udtrans_collect_tiny_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	//snagmotor_R.motor_angle_set���ܲ���ı䣬����ֱ��ʹsnagmotor_L.motor_angle_setȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_collect_tiny_control->snagmotor_R.motor_angle_pid,-udtrans_collect_tiny_control->snagmotor_L.motor_angle_set, udtrans_collect_tiny_control->snagmotor_R.motor_angle);
	udtrans_collect_tiny_control->snagmotor_R.given_current = PID_Calc(&udtrans_collect_tiny_control->snagmotor_R.motor_speed_pid, udtrans_collect_tiny_control->snagmotor_R.motor_speed, angle_out);
}

//����Դ����ȡ������������������Ҫ̧������ȡװ����Ҫ���
static void UPTrans_collect_huge_control(UDTrans_Control_t *udtrans_collect_huge_control)
{
	static fp32 angle_out;	
	static fp32 vx_set_channel;
	int16_t vx_channel;	 //ң����ԭʼͨ��ֵ	
	
	rc_deadline_limit(udtrans_collect_huge_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//ͨ��ֵ��ֵ
	vx_set_channel = vx_channel;
	
#ifdef COLLECT_CONVERSION_AUTO	 	
	//��һ������ȡװ�ô򿪣�����������̧����߸߶�         
	if(trans_step == 1)    
	{
		trans_time++;
		Clamp_on();              //��ȡװ�ô� 
		camera_screen();         //ͼ������Ƶλ��
		if(trans_time > COLLECT_TIME_PART)	  	
		{
			stretch_on();         //��ȡװ����� 
			
			//̧������Ŀ��߶�
			udtrans_collect_huge_control->upmotor_L.motor_angle_set = COLLECT_HUGE_UP_HEIGHT_1;   
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
			{
				udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
																																						TRANS_UP_SPEED);
			}			
      //���̧������Ӧλ�ã���ֹ������¿�ס		
			switch (collect_gold)         
			{
				case 0:
				{
					udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.max_angle_limit; 		
					break;
				}
				case 1:
				{
					udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.min_angle_limit; 	
					break;
				}
				case 2:
				{
					udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.min_angle_limit; 	
					break;
				}
			} 	
			
			if(udtrans_collect_huge_control->goldmotor.set_ramp_angle != udtrans_collect_huge_control->goldmotor.motor_angle_set)
			{
				udtrans_collect_huge_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->goldmotor.motor_angle_set, udtrans_collect_huge_control->goldmotor.set_ramp_angle, 
				                                                                    GOLD_UP_SPEED);
			}	
			
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set &&
				 udtrans_collect_huge_control->goldmotor.set_ramp_angle == udtrans_collect_huge_control->goldmotor.motor_angle_set  )
			{
				trans_step = 2;   //����ڶ���
			  trans_time = 0;
			}
		}
	}
	//�ڶ�������ȡװ����ת�����λ��
	else if(trans_step == 2)  
	{	
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = COLLECT_HUGE_HAND; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_FAST);
		}	
		
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
		  trans_step = 3;   			
		} 	
	}
	//���������ȴ������ֵ���λ�ã��ֶ����ڣ�����������������һ�� 
	else if(trans_step == 3)  
	{	
		//ң������ͨ�����£����Ƽ�ȡװ����ת���
		if(switch_is_down(udtrans_collect_huge_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		//ң������ͨ�����У�����̧�����
		if(switch_is_mid(udtrans_collect_huge_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_huge_control->upmotor_L.set_ramp_angle += vx_set_channel * 0.3f; 
		}		
		
		//����������������һ��
		if (IF_MOUSE_PRESSED_LEFT)
		{		
      trans_step = 4;
		}	
	} 
	//���Ĳ�����ȡװ�üн�            
	else if(trans_step == 4)
	{
		trans_time++;
		Clamp_close();        //��ȡװ�üн� 
		if(trans_time > COLLECT_TIME_PART)		
		{
			//̧������һ�θ߶�
			udtrans_collect_huge_control->upmotor_L.motor_angle_set = COLLECT_HUGE_UP_HEIGHT_2;   
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
			{
				udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
																																						TRANS_UP_SPEED);
			}		
			
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set)
			{
				trans_step = 5;
				trans_time = 0;				
			}		
		}
	}      
	//���岽����ȡװ����ת���м�λ��
	else if(trans_step == 5)
	{
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = COLLECT_HAND_PICKUP_HEIGHT; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			
			camera_mid();     //ͼ�����Ƿ����������
			
			if(IF_MOUSE_PRESSED_LEFT)  //���н��ȡ��һ��
			{					
				switch (collect_gold)         
				{
					case 0:
					{
	      		collect_gold ++;						
						trans_step = 6; 				
						break;
					}
					case 1:
					{
		      	collect_gold ++;
						trans_step = 6; 				
						break;
					}
					case 2:     //ȡ�����������ھŲ�
					{
	      		collect_gold ++;
						trans_step = 9;
						break;
					}
				}													
			}
			else if(IF_MOUSE_PRESSED_RIGH)  //û�н�󣬾�����
			{	
	      collect_gold ++;
				trans_step = 9;	
			}		
			else if(IF_KEY_PRESSED_X)  
			{
			  trans_step = 1;          //û�ме���X���ص���һ��
			} 				
		}
	}
	//����������ȡװ���ջأ��ж�Ҫ������һ�� 
	else if(trans_step == 6)
	{
		trans_time++;
		if(trans_time ==    COLLECT_TIME_PART)			stretch_close();       //��ȡװ���ջ�
		if(trans_time ==  2*COLLECT_TIME_PART)
		{
			trans_step = 7;	
			trans_time = 0;					
		}		

	}
	//���߲�����ȡװ����ת��λͬʱ̧���ص���С�߶�
	else if(trans_step == 7)
	{	
		//��ȡװ����ת��λ
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = udtrans_collect_huge_control->rotatemotor.min_angle_limit; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}

		//̧���ص���С�߶�
		udtrans_collect_huge_control->upmotor_L.motor_angle_set = udtrans_collect_huge_control->upmotor_L.min_angle_limit;   
		if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}	
		
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set &&
			 udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set)
		{
      trans_step = 8;			
		}
	}
	//�ڰ˲�����ȡװ���ɿ���������룬���һ��ʰȡ
	else if(trans_step == 8)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART)		Clamp_on();        //��ȡװ���ɿ�	
		if(trans_time == 2*COLLECT_TIME_PART)
		{
			trans_step = 1;	
			trans_time = 0;			
		}	
	}	
	//�ھŲ�����ȡװ���ջ�
	else if(trans_step == 9)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART)		stretch_close();   //��ȡװ���ջ�	
		if(trans_time == 2*COLLECT_TIME_PART)
		{
			trans_step = 10;	
			trans_time = 0;			
		}	
	}	
	//�ھŲ���̧�����£����̧������
	else if(trans_step == 10)
	{
		//̧�������Ӧ�߶�
		switch (collect_gold)         
		{
			case 1:  //ȡ��һ����̧���ص���С�߶�
			{
			  udtrans_collect_huge_control->upmotor_L.motor_angle_set = udtrans_collect_huge_control->upmotor_L.min_angle_limit; 
				break;
			}
			case 2:  //ȡ��������̧���ص���С�߶�
			{
	  		udtrans_collect_huge_control->upmotor_L.motor_angle_set = udtrans_collect_huge_control->upmotor_L.min_angle_limit; 
				break;
			}
			case 3:  //ȡ��������̧������һ���߶ȣ���֤�ڲ�������鲻��˦��
			{
		    udtrans_collect_huge_control->upmotor_L.motor_angle_set = COLLECT_UP_GOLD_OVER; 		
				break;
			}
		}			

		if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}

		//ץ�ֵ������߶�
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = COLLECT_HAND_PICKUP_HEIGHT; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.min_angle_limit; 			
		if(udtrans_collect_huge_control->goldmotor.set_ramp_angle != udtrans_collect_huge_control->goldmotor.motor_angle_set)
		{
			udtrans_collect_huge_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->goldmotor.motor_angle_set, udtrans_collect_huge_control->goldmotor.set_ramp_angle, 
			                                                                    GOLD_UP_SPEED);
		}	
		
		if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set     &&
		   udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set &&
			 udtrans_collect_huge_control->goldmotor.set_ramp_angle == udtrans_collect_huge_control->goldmotor.motor_angle_set )
		{	
			trans_time++;
			//ѭ��һ��ʱ�䣬��֤�ص���ʼλ��
			if( trans_time == 500 )  
			{
				//���ʰȡ��ɣ��л�Ϊֹͣģʽ����������Ϊ1����ʱ���㣬���������ֵ��ʰȡ���������
				udtrans_collect_huge_control->uptrans_behaviour = UPTRANS_STOP; 
				trans_step = 1;
				trans_time = 0;
	  	  gold_number = collect_gold;	      
	    	collect_gold = 0;			            
				camera_mid();           //ͼ���ص��м�λ��				
			}			
		}
	}
#else
//����д�ڶ���ȡ������	
#endif
	
  //set_ramp_angle�޷�����
	udtrans_collect_huge_control->upmotor_L.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_huge_control->upmotor_L);
	udtrans_collect_huge_control->rotatemotor.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_huge_control->rotatemotor);
	
	//̧�����PID����
	angle_out = PID_Calc(&udtrans_collect_huge_control->upmotor_L.motor_angle_pid, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, udtrans_collect_huge_control->upmotor_L.motor_angle);
	udtrans_collect_huge_control->upmotor_L.given_current = PID_Calc(&udtrans_collect_huge_control->upmotor_L.motor_speed_pid, udtrans_collect_huge_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_collect_huge_control->upmotor_R.motor_angle_pid,-udtrans_collect_huge_control->upmotor_L.set_ramp_angle, udtrans_collect_huge_control->upmotor_R.motor_angle);
	udtrans_collect_huge_control->upmotor_R.given_current = PID_Calc(&udtrans_collect_huge_control->upmotor_R.motor_speed_pid, udtrans_collect_huge_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_collect_huge_control->rotatemotor.motor_angle_pid, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, udtrans_collect_huge_control->rotatemotor.motor_angle);
	udtrans_collect_huge_control->rotatemotor.given_current = PID_Calc(&udtrans_collect_huge_control->rotatemotor.motor_speed_pid, udtrans_collect_huge_control->rotatemotor.motor_speed, angle_out);	
	//���̧�����PID����
	angle_out = PID_Calc(&udtrans_collect_huge_control->goldmotor.motor_angle_pid, udtrans_collect_huge_control->goldmotor.set_ramp_angle, udtrans_collect_huge_control->goldmotor.motor_angle);
	udtrans_collect_huge_control->goldmotor.given_current = PID_Calc(&udtrans_collect_huge_control->goldmotor.motor_speed_pid, udtrans_collect_huge_control->goldmotor.motor_speed, angle_out);
  //�ϰ�����PID����
	//����ȡ���������ò����ϰ��������Ƕ����ò��䣨motor_angle_set��
	angle_out = PID_Calc(&udtrans_collect_huge_control->snagmotor_L.motor_angle_pid, udtrans_collect_huge_control->snagmotor_L.motor_angle_set, udtrans_collect_huge_control->snagmotor_L.motor_angle);
	udtrans_collect_huge_control->snagmotor_L.given_current = PID_Calc(&udtrans_collect_huge_control->snagmotor_L.motor_speed_pid, udtrans_collect_huge_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	//snagmotor_R.motor_angle_set���ܲ���ı䣬����ֱ��ʹsnagmotor_L.motor_angle_setȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_collect_huge_control->snagmotor_R.motor_angle_pid,-udtrans_collect_huge_control->snagmotor_L.motor_angle_set, udtrans_collect_huge_control->snagmotor_R.motor_angle);
	udtrans_collect_huge_control->snagmotor_R.given_current = PID_Calc(&udtrans_collect_huge_control->snagmotor_R.motor_speed_pid, udtrans_collect_huge_control->snagmotor_R.motor_speed, angle_out);
}

//���һ���̧��100����ȡװ�����
static void UPTrans_conversion_control(UDTrans_Control_t *udtrans_conversion_control)
{
	static fp32 angle_out;
	static fp32 vx_set_channel;
	int16_t vx_channel;	 //ң����ԭʼͨ��ֵ	
	
	//�������ƣ���Ϊң�������ܴ��ڲ��죬ҡ�����м䣬��ֵ��Ϊ0
	rc_deadline_limit(udtrans_conversion_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//ͨ��ֵ��ֵ
	vx_set_channel = vx_channel;

#ifdef COLLECT_CONVERSION_AUTO	 		
	//��һ����̧����ɨ���ά��߶�
	if(trans_step == 1)       
	{
    camera_up();     //ͼ����
		udtrans_conversion_control->upmotor_L.motor_angle_set = udtrans_conversion_control->upmotor_L.max_angle_limit;    
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
			                                                                  TRANS_UP_SPEED);
		}
		
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle == udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			trans_step = 2;   
		}
	}
	//�ڶ�������ȡװ�����
	else if(trans_step == 2)  
	{
		trans_time++;
		if(trans_time ==   CONVERSION_TIME_PART)		stretch_on();         //��ȡװ�����   
		if(trans_time == 2*CONVERSION_TIME_PART)	  
		{
			trans_step = 3; 
      trans_time = 0;	
		}			
	}	
  //����������ȡװ����ת���һ�̨ɨ��߶�
	else if(trans_step == 3)  
	{
		udtrans_conversion_control->rotatemotor.motor_angle_set = CONVERSION_HAND_SCAN; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_SLOW);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set)
		{ 			
     	trans_step = 4;	
		} 
	}
	//���Ĳ���ɨ�룬ң�������ڸ߶�
	else if(trans_step == 4)  
	{
		trans_time++;	
		
		//ң������ͨ�����£����Ƽ�ȡװ����ת���
		if(switch_is_down(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		//ң������ͨ�����У�����̧�����
		if(switch_is_mid(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle += vx_set_channel * 0.3f; 
		}
				
		
		if (trans_time > CONVERSION_TIME_PART)
		{
			//����������������һ��
			if (IF_MOUSE_PRESSED_LEFT)
			{		
				Clamp_on();          //��ȡװ���ɿ� 
				camera_screen();     //ͼ����ʾ��
				trans_time = 0;
				trans_step = 5;	
			}		
			//����C���ص����߲���ȡ�洢�Ľ��
			if (IF_KEY_PRESSED_C)
			{		
				trans_time = 0;
				trans_step = 7;	
			}		
			//����Z�����������Ϊ2
			if (IF_KEY_PRESSED_Z)
			{	
        conversion_gold = 0;							
        gold_number = 2;
			}		
			//����X�����������Ϊ3
			if (IF_KEY_PRESSED_X)
			{	
				conversion_gold = 0;				
        gold_number = 3;
			}				
		}

	}
	//���岽��̧�����ֶ����ڸ߶ȣ���ȡװ���ջز��н�
	else if(trans_step == 5)  
	{
		trans_time++;
		
		if (trans_time >= CONVERSION_TIME_PART)
		{	
			//̧���½����ֶ����ڸ߶�
			udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_ADJUST;    
			if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
			{
				udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
				                                                                  TRANS_UP_SPEED);
			}
			//ץ�ֵ���ص����λ��
			udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.max_angle_limit; 
			if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
			{
				udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
				                                                                    HAND_FAST);
			}
		}
		
		if(trans_time ==    CONVERSION_TIME_PART)
		{
			stretch_close();     //��ȡװ���ջ�
		}			
		if(trans_time ==  2*CONVERSION_TIME_PART)	  		
		{  
			Clamp_close();       //��ȡװ�üн�	
		  trans_step = 6; 
      trans_time = 0;			
		} 

	}
	//�������������ֶ��ƽ��״̬ͬʱ���̧������Ӧλ��
	else if(trans_step == 6)  
	{ 
		//�ж��ϴ�Ϊ������Դ��
		switch (gold_number)
		{	
			case 3:  //��������
			{
				if(conversion_gold == 0)      
				{
					//��������м�λ��
					udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.mid_angle_limit; 	
				}
				else if(conversion_gold == 1)  
				{
					//����������λ��
					udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.max_angle_limit; 	
				}
				break;
			}
			case 2:	 //����һ��	
			{  
				//����������λ��
				udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.max_angle_limit; 	
				break;
			}
		}
		
		if(udtrans_conversion_control->goldmotor.set_ramp_angle != udtrans_conversion_control->goldmotor.motor_angle_set)
		{
			udtrans_conversion_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->goldmotor.motor_angle_set, udtrans_conversion_control->goldmotor.set_ramp_angle, 
			                                                                  GOLD_UP_SPEED);
		}	

		//ң������ͨ�����£����Ƽ�ȡװ����ת���
		if(switch_is_down(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			//����Ŀ��Ƕȣ��ۼӣ�
			udtrans_conversion_control->rotatemotor.set_ramp_angle += vx_set_channel*0.3f; 
		}
		//ң������ͨ�����У�����̧�����
		if(switch_is_mid(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			//����Ŀ��Ƕȣ��ۼӣ�
			udtrans_conversion_control->upmotor_L.set_ramp_angle += vx_set_channel*0.3f; 
		}

		//��ȡװ���������ƣ�
		//����Ҽ����
		if (IF_MOUSE_PRESSED_RIGH)
		{
			if (udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time < RC_KEY_SHOT_TIME)
			{
				udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time++;
				if(udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time == RC_KEY_SHOT_TIME)
				{
					//����Ҽ����	
					switch (conversion_mouse)         
					{
						case 1:
						{
							stretch_on();    //��ȡװ�����							
							conversion_mouse = 0;
							break;
						}
						case 0:
						{
							stretch_close(); //��ȡװ���ջ�
							conversion_mouse = 1;
							break;
						}
					}
					udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
				}
			}
		}
		else
		{
			udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
		}	
	
		//����������������һ��
		if (IF_MOUSE_PRESSED_LEFT)
		{		
			conversion_gold++;            //�һ�������һ
			conversion_mouse = 1;         //��������־����
			if(conversion_gold == 1 )     //��ɵ�һ�ζһ�
			{
				switch (gold_number)
		    {	
		     	case 1:  trans_step = 11; break;   //ȡ��һ����������ʮһ����ֱ������
					case 2:  trans_step = 7;  break;   //ȡ���������������߲�
					case 3:  trans_step = 7;  break;   //ȡ���������������߲�
		   	}				
			}
			else if(conversion_gold == 2) 				
			{
				switch (gold_number)
		    {	
					case 2:  trans_step = 11; break;  //ȡ��������������ʮһ����ֱ������
		     	case 3:  trans_step = 7;  break;  //ȡ���������������߲�
		   	}			   
			}
			else if(conversion_gold == 3)
			{
				trans_step = 11;       //������ζһ���������ʮһ��	
			}
		}		
	}
	//���߲�����ȡװ�ô��ջأ���ת��λ
	else if(trans_step == 7)  
	{
	  Clamp_on();           //��ȡװ���ɿ�	
		trans_time++;		
		if(trans_time == CONVERSION_TIME_PART)	
		{
			stretch_close();    //��ȡװ���ջ�			
		}				
		
		udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.min_angle_limit; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_SLOW);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set &&
			 trans_time > 2*CONVERSION_TIME_PART)
		{ 
      trans_step = 8;		
      trans_time = 0;					
		} 	

	}
	//�ڰ˲���̧��������洢�߶�
	else if(trans_step == 8) 
	{
		switch (gold_number)
		{	
			case 3:  //��������
			{
				if(conversion_gold == 1)      
				{
					//̧�������һ�����߶�
					udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_PICKUP1;	
				}
				else if(conversion_gold == 2)  
				{
					//̧������ڶ������߶�
					udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_PICKUP2;	
				}
				break;
			}
			case 2:	 //����һ����ֱ�ӵ��ڶ������߶�
			{  
				udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_PICKUP2;	
				break;
			}
		}
		    
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
		                                                                  	TRANS_UP_SPEED);
		}

		if(udtrans_conversion_control->upmotor_L.set_ramp_angle == udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
      trans_step = 9;				
		}
		
	}			
	//�ھŲ�����ȡװ�üн�
	else if(trans_step == 9)  
	{
		trans_time++;		
		if(trans_time ==   CONVERSION_TIME_PART)	  Clamp_close();        //��ȡװ�üн�	
		if(trans_time == 2*CONVERSION_TIME_PART)				
		{
      trans_step = 10;     
      trans_time = 0;			
		}			
	}
	//��ʮ������ȡװ����ת���м�λ��
	else if(trans_step == 10)  
	{
		udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.mid_angle_limit; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_SLOW);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set)
		{ 
			trans_step = 1;     //�ص���һ��
		} 
	}
	//��ʮһ������ȡװ���ջ��ɿ�
	else if(trans_step == 11)  
	{
		camera_mid();           //ͼ���ص��м�λ��
		trans_time++;		
		if(trans_time ==   CONVERSION_TIME_PART)	   
		{
			Clamp_on();         //��ȡװ���ɿ�	
			stretch_close();    //��ȡװ���ջ�				
		}
		if(trans_time == 2*CONVERSION_TIME_PART)    		
		{  
      trans_step = 12;
      trans_time = 0;			
		}  
	}
	//��ʮ��������ȡװ����ת��λ̧�����½��̧������
	else if(trans_step == 12)  
	{
		udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.min_angle_limit; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_FAST);
		}
		
		udtrans_conversion_control->upmotor_L.motor_angle_set = udtrans_conversion_control->upmotor_L.min_angle_limit; 
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
			                                                                  TRANS_UP_SPEED);
		}

		udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.min_angle_limit; 
		if(udtrans_conversion_control->goldmotor.set_ramp_angle != udtrans_conversion_control->goldmotor.motor_angle_set)
		{
			udtrans_conversion_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->goldmotor.motor_angle_set, udtrans_conversion_control->goldmotor.set_ramp_angle, 
			                                                                  GOLD_UP_SPEED);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set &&
			 udtrans_conversion_control->upmotor_L.set_ramp_angle == udtrans_conversion_control->upmotor_L.motor_angle_set     &&
			 udtrans_conversion_control->goldmotor.set_ramp_angle == udtrans_conversion_control->goldmotor.motor_angle_set)
		{		
			trans_time++;
			//ѭ��һ��ʱ�䣬��̧֤���ص���ʼλ��
			if( trans_time == 500 )  
			{
		  	//�һ���ɣ��л�Ϊֹͣģʽ����������Ϊ1����ʱ���㣬�һ���������㣬����Ҽ����Ƽ�ȡװ��������־λ����Ϊ1
		  	udtrans_conversion_control->uptrans_behaviour = UPTRANS_STOP; 
		  	trans_step = 1;
		  	trans_time = 0;
		  	conversion_gold = 0;
	    	conversion_mouse = 1;   //��������־����
			}			
		}
	}
#else
//������д�ڶ��ֶһ�����
#endif
	
  //set_ramp_angle�޷�����
	udtrans_conversion_control->upmotor_L.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_conversion_control->upmotor_L);
	udtrans_conversion_control->rotatemotor.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_conversion_control->rotatemotor);
	
	//̧�����PID����
	angle_out = PID_Calc(&udtrans_conversion_control->upmotor_L.motor_angle_pid, udtrans_conversion_control->upmotor_L.set_ramp_angle, udtrans_conversion_control->upmotor_L.motor_angle);
	udtrans_conversion_control->upmotor_L.given_current = PID_Calc(&udtrans_conversion_control->upmotor_L.motor_speed_pid, udtrans_conversion_control->upmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_conversion_control->upmotor_R.motor_angle_pid,-udtrans_conversion_control->upmotor_L.set_ramp_angle, udtrans_conversion_control->upmotor_R.motor_angle);
	udtrans_conversion_control->upmotor_R.given_current = PID_Calc(&udtrans_conversion_control->upmotor_R.motor_speed_pid, udtrans_conversion_control->upmotor_R.motor_speed, angle_out);
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_conversion_control->rotatemotor.motor_angle_pid, udtrans_conversion_control->rotatemotor.set_ramp_angle, udtrans_conversion_control->rotatemotor.motor_angle);
	udtrans_conversion_control->rotatemotor.given_current = PID_Calc(&udtrans_conversion_control->rotatemotor.motor_speed_pid, udtrans_conversion_control->rotatemotor.motor_speed, angle_out);	
	//ץ����ת���PID����
	angle_out = PID_Calc(&udtrans_conversion_control->goldmotor.motor_angle_pid, udtrans_conversion_control->goldmotor.set_ramp_angle, udtrans_conversion_control->goldmotor.motor_angle);
	udtrans_conversion_control->goldmotor.given_current = PID_Calc(&udtrans_conversion_control->goldmotor.motor_speed_pid, udtrans_conversion_control->goldmotor.motor_speed, angle_out);	
  //�ϰ�����PID����
	//����ȡ���������ò����ϰ��������Ƕ����ò��䣨motor_angle_set��
	angle_out = PID_Calc(&udtrans_conversion_control->snagmotor_L.motor_angle_pid, udtrans_conversion_control->snagmotor_L.motor_angle_set, udtrans_conversion_control->snagmotor_L.motor_angle);
	udtrans_conversion_control->snagmotor_L.given_current = PID_Calc(&udtrans_conversion_control->snagmotor_L.motor_speed_pid, udtrans_conversion_control->snagmotor_L.motor_speed, angle_out);			
	//������� ����ֱ������һ������Ƕ�ȡ��ʹ��	
	//snagmotor_R.motor_angle_set���ܲ���ı䣬����ֱ��ʹsnagmotor_L.motor_angle_setȡ��ʹ��	
	angle_out = PID_Calc(&udtrans_conversion_control->snagmotor_R.motor_angle_pid,-udtrans_conversion_control->snagmotor_L.motor_angle_set, udtrans_conversion_control->snagmotor_R.motor_angle);
	udtrans_conversion_control->snagmotor_R.given_current = PID_Calc(&udtrans_conversion_control->snagmotor_R.motor_speed_pid, udtrans_conversion_control->snagmotor_R.motor_speed, angle_out);
}

//����ֵ�Ӻͺ���
static void UPTRANS_UpdateMotorAngleSum(UPTrans_Motor_t *UPTrans_Motor)
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

//motor_angle_set�޷����� 
static fp32 fp32_constrain_upmotor_angle(UPTrans_Motor_t* motor)
{
	  if (motor->motor_angle_set < motor->min_angle_limit)
        return motor->min_angle_limit;
    else if (motor->motor_angle_set > motor->MAX_angle_limit)
        return motor->MAX_angle_limit;
    else
        return motor->motor_angle_set;
}

//set_ramp_angle�޷����� 
static fp32 fp32_constrain_upmotor_ramp(UPTrans_Motor_t* motor)
{
	  if (motor->set_ramp_angle < motor->min_angle_limit)
        return motor->min_angle_limit;
    else if (motor->set_ramp_angle > motor->MAX_angle_limit)
        return motor->MAX_angle_limit;
    else
        return motor->set_ramp_angle;
}

/********************************�������׺Ͷ��ʹ�ú���************************************/
//��ȡ���׳�ʼ��״̬ ����û���ϵ�Ҳ�Ǵ�״̬
void Clamp_on(void)                  //��ȡװ���ɿ����أ�
{
	GPIO_ResetBits(GPIOI,GPIO_Pin_2);
	clamp_flag = 0;
}

void Clamp_close(void)               //��ȡװ�üн�������
{
	GPIO_SetBits(GPIOI,GPIO_Pin_2);
	clamp_flag = 1;
}

void stretch_close(void)             //��ȡװ���ջأ��أ�
{
	GPIO_ResetBits(GPIOI,GPIO_Pin_7);
	stretch_flag = 0;	
}

void stretch_on(void)                //��ȡװ�����������
{
	GPIO_SetBits(GPIOI,GPIO_Pin_7);
	stretch_flag = 1;	
}

void rescue_close(void)              //��Ԯ�ջ�
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);
	help_flag = 0; 
}

void rescue_on(void)                 //��Ԯ���
{
	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	help_flag = 1; 
}

void rotary_actuator_close(void)     //���Ϲر�
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	snag_flag = 0;
}

void rotary_actuator_on(void)        //���ϴ�
{
	GPIO_SetBits(GPIOD,GPIO_Pin_13);
	snag_flag = 1;
}

void supply1_close(void)             //����1�ر�  PA0
{
  TIM_SetCompare1(TIM2,2050); 
  supply1_flag = 0;	
}

void supply1_on(void)                //����1����  
{
  TIM_SetCompare1(TIM2,2800);
  supply1_flag = 1;	
}

void supply1_disable(void)           //����1ʧ��  
{
  TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
  supply1_flag = 1;	
}

void supply2_close(void)             //����2�ر�  PA1
{
  TIM_SetCompare2(TIM2,2450);
	supply2_flag = 0;
}

void supply2_on(void)                //����2����  
{
  TIM_SetCompare2(TIM2,1850);
	supply2_flag = 1;
}

void supply2_disable(void)           //����2ʧ��  
{
  TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
  supply2_flag = 1;	
}

void camera_up(void)                 //ͼ����     PA3
{
  TIM_SetCompare3(TIM2,3150);
}

void camera_mid(void)                //ͼ����  ����λ��
{
  TIM_SetCompare3(TIM2,2950);
}

void camera_down(void)               //ͼ����
{
  TIM_SetCompare3(TIM2,2400);
}

void camera_screen(void)             //ͼ����ʾ��
{
  TIM_SetCompare3(TIM2,2000);
}

/********************************�������׺Ͷ��ʹ�ú���************************************/


