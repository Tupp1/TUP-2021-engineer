#include "start_task.h"
#include "INS_task.h"
#include "main.h"
#include "chassis_task.h"
#include "system.h"
#include "calibrate_task.h"
#include "Revolver_task.h"
#include "UDTrans_task.h"
#include "rc.h"
#include "led.h"


//���񴴽�
#define START_TASK_PRIO  11  				//�������ȼ�
TaskHandle_t StartTask_Handler; 			//������
void Start_Task(void *pvParameters); 		//������

#define INS_TASK_PRIO 20
#define INS_TASK_SIZE 512
static TaskHandle_t INSTask_Handler;

/*ʧ�ر���*/
#define TASK_Control_Protect_PRIO 2  		//�������ȼ�,ʧ��������
TaskHandle_t  Task_Control_Protect_Handler; //������
void Task_Control_Protect(void *pvParameters);

#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 512
TaskHandle_t GIMBALTask_Handler;

#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;

#define CALIBRATE_TASK_PRIO 5
#define CALIBRATE_STK_SIZE 512
static TaskHandle_t CalibrateTask_Handler;

//#define REVOLVER_TASK_PRIO 5
//#define REVOLVER_STK_SIZE 512
//static TaskHandle_t RevolverTask_Handler;

#define UPTRANS_TASK_PRIO 20
#define UPTRANS_STK_SIZE 512
static TaskHandle_t UPTransTask_Handler;

#define SYSTEMRESET_TASK_PRIO 17
#define SYSTEMRESET_STK_SIZE 128
static TaskHandle_t SYSTEMRESET_Handler;
void Task_SystemReset(void *pvParameters);

#define TASK_500ms_PRIO 10  					//�������ȼ�
TaskHandle_t  Task_500ms_Handler; 			//������
void Task_500ms(void *pvParameters); 		//������


/***********************************************************/
void App_Task_Create(void)
{
	xTaskCreate((TaskFunction_t )Start_Task, 			//������
				(const char* )"Start_Task", 			//��������
				(uint16_t )STK_SIZE_128, 				//�����ջ��С
				(void* )NULL, 							//���ݸ��������Ĳ���
				(UBaseType_t )START_TASK_PRIO, 			//�������ȼ�
				(TaskHandle_t* )&StartTask_Handler); 	//������

}


/**
  * @brief  ������ʼ����������
  * @param  void
  * @retval void
  * @attention ���������ڴ˴���
  */
void Start_Task(void *pvParameters)
{
		
	taskENTER_CRITICAL(); 	//�����ٽ���
								 
	/*------------------------------------------------*/

	 xTaskCreate((TaskFunction_t)INSTask,
						   (const char *)"INSTask",
						   (uint16_t)INS_TASK_SIZE,
						   (void *)NULL,
						   (UBaseType_t)INS_TASK_PRIO,
						   (TaskHandle_t *)&INSTask_Handler);
	
	 xTaskCreate((TaskFunction_t)calibrate_task,
								(const char *)"CaliTask",
								(uint16_t)CALIBRATE_STK_SIZE,
								(void *)NULL,
								(UBaseType_t)CALIBRATE_TASK_PRIO,
								(TaskHandle_t *)&CalibrateTask_Handler);
								
	 xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
				
   xTaskCreate((TaskFunction_t)GIMBAL_task,
                (const char *)"GIMBAL_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GIMBALTask_Handler);
								
	xTaskCreate((TaskFunction_t )UDTrans_task, 
								(const char* )"UDTrans_Task",
								(uint16_t )UPTRANS_STK_SIZE,
								(void* )NULL,
								(UBaseType_t )UPTRANS_TASK_PRIO,
								(TaskHandle_t* )&UPTransTask_Handler);
								
	xTaskCreate((TaskFunction_t )Task_SystemReset, 
								(const char* )"Task_SystemReset",
								(uint16_t )SYSTEMRESET_STK_SIZE,
								(void* )NULL,
								(UBaseType_t )SYSTEMRESET_TASK_PRIO,
								(TaskHandle_t* )&SYSTEMRESET_Handler);		
			
	//���� 500ms ����
	xTaskCreate((TaskFunction_t )Task_500ms,
								(const char* )"Task_500ms",
								(uint16_t )STK_SIZE_128,
								(void* )NULL,
								(UBaseType_t )TASK_500ms_PRIO,
								(TaskHandle_t* )&Task_500ms_Handler);

//	xTaskCreate((TaskFunction_t )Revolver_task,
//								(const char* )"Task_Revolver",
//								(uint16_t )STK_SIZE_128,
//								(void* )NULL,
//								(UBaseType_t )REVOLVER_TASK_PRIO,
//								(TaskHandle_t* )&RevolverTask_Handler);
								
	vTaskDelay(500);		
  vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL(); 								//�˳��ٽ���		
}


//ÿ500msִ��һ��������
void Task_500ms(void *pvParameters)
{
	for(;;)
	{	
		led_green_toggle();
		vTaskDelay(TIME_STAMP_200MS);				//500ms
    //���벿��		
		//Send_to_Teammate();
		
	}
}

//ϵͳ��λ������
void Task_SystemReset(void *pvParameters)
{
	RC_ctrl_time_t   RTask_500ms_RC_time;	
	while(1)
	{
		
		//����Ҽ����
		if (IF_MOUSE_PRESSED_RIGH)
		{
			if (RTask_500ms_RC_time.mouse_right_time < SYSTEM_RESET_RC_time)
			{
				RTask_500ms_RC_time.mouse_right_time++;
				if(RTask_500ms_RC_time.mouse_right_time == SYSTEM_RESET_RC_time)
				{
	    		__set_FAULTMASK(1);   //���ж�
	    		NVIC_SystemReset(); 	//��λ  	
					RTask_500ms_RC_time.mouse_right_time = 0;
				}
			}
		}
		else
		{
			RTask_500ms_RC_time.mouse_right_time = 0;
		}
		
		//�����ʱ
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}


