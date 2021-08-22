
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "spi.h"
#include "delay.h"
#include "flash.h"
#include "pwm.h"

#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#include "system.h"


#include "calibrate_task.h"
#include "start_task.h"
#include "Gimbal_Task.h"
#include "Timer_Send_Task.h"
#include "Info_Update.h"


int main(void)
{
	//系统硬件初始化
	System_Init();
	
	//创建软件定时器数据收发任务，can1、can2，不要放太多函数
	Timer_Send_Create();
	
  //创建状态更新任务
  App_Info_Create();
	
	//创建开始任务
	App_Task_Create();
	
	//开启任务调度
	vTaskStartScheduler();
   
}




