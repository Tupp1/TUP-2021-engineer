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


//任务创建
#define START_TASK_PRIO  11  				//任务优先级
TaskHandle_t StartTask_Handler; 			//任务句柄
void Start_Task(void *pvParameters); 		//任务函数

#define INS_TASK_PRIO 20
#define INS_TASK_SIZE 512
static TaskHandle_t INSTask_Handler;

/*失控保护*/
#define TASK_Control_Protect_PRIO 2  		//任务优先级,失控任务函数
TaskHandle_t  Task_Control_Protect_Handler; //任务句柄
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

#define TASK_500ms_PRIO 10  					//任务优先级
TaskHandle_t  Task_500ms_Handler; 			//任务句柄
void Task_500ms(void *pvParameters); 		//任务函数


/***********************************************************/
void App_Task_Create(void)
{
	xTaskCreate((TaskFunction_t )Start_Task, 			//任务函数
				(const char* )"Start_Task", 			//任务名称
				(uint16_t )STK_SIZE_128, 				//任务堆栈大小
				(void* )NULL, 							//传递给任务函数的参数
				(UBaseType_t )START_TASK_PRIO, 			//任务优先级
				(TaskHandle_t* )&StartTask_Handler); 	//任务句柄

}


/**
  * @brief  创建开始任务任务函数
  * @param  void
  * @retval void
  * @attention 控制任务在此创建
  */
void Start_Task(void *pvParameters)
{
		
	taskENTER_CRITICAL(); 	//进入临界区
								 
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
			
	//创建 500ms 任务
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
  vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL(); 								//退出临界区		
}


//每500ms执行一次任务函数
void Task_500ms(void *pvParameters)
{
	for(;;)
	{	
		led_green_toggle();
		vTaskDelay(TIME_STAMP_200MS);				//500ms
    //代码部分		
		//Send_to_Teammate();
		
	}
}

//系统复位任务函数
void Task_SystemReset(void *pvParameters)
{
	RC_ctrl_time_t   RTask_500ms_RC_time;	
	while(1)
	{
		
		//鼠标右键检测
		if (IF_MOUSE_PRESSED_RIGH)
		{
			if (RTask_500ms_RC_time.mouse_right_time < SYSTEM_RESET_RC_time)
			{
				RTask_500ms_RC_time.mouse_right_time++;
				if(RTask_500ms_RC_time.mouse_right_time == SYSTEM_RESET_RC_time)
				{
	    		__set_FAULTMASK(1);   //关中断
	    		NVIC_SystemReset(); 	//复位  	
					RTask_500ms_RC_time.mouse_right_time = 0;
				}
			}
		}
		else
		{
			RTask_500ms_RC_time.mouse_right_time = 0;
		}
		
		//相对延时
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}


