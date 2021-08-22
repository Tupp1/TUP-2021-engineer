
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
	//ϵͳӲ����ʼ��
	System_Init();
	
	//���������ʱ�������շ�����can1��can2����Ҫ��̫�ຯ��
	Timer_Send_Create();
	
  //����״̬��������
  App_Info_Create();
	
	//������ʼ����
	App_Task_Create();
	
	//�����������
	vTaskStartScheduler();
   
}




