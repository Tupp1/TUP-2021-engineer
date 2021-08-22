#include "Info_Update.h"
#include "Info_Transmitter.h"
#include "CAN_Receive.h"
#include "remote.h"
#include "control.h"
#include "rc.h"
#include "led.h"
#include "iwdg.h"

#define APP_INFO_TASK_PRIO  1  				//�������ȼ�
TaskHandle_t AppInfoTask_Handler; 			//������

#define APP_INFO_TRANSMIT_PRIO  6  			//�������ȼ�
TaskHandle_t AppInfoTransmit_Handler; 		//������

#define APP_INFO_JUDGEREAD_PRIO  3  		//�������ȼ�
TaskHandle_t AppInfoJudge_Handler; 			//������

/**
  * @brief  ����״̬����������
  * @param  void
  * @retval void
  * @attention ����ϵͳ����״̬
  */
void App_Info_Create(void)
{
	taskENTER_CRITICAL(); 	//�����ٽ���
	
	/* ״̬��ֵ��ȡ */
	xTaskCreate((TaskFunction_t )Info_Update_Task, 		//������
				(const char* )"Info_Update_Task", 		//��������
				(uint16_t )STK_SIZE_128, 				//�����ջ��С
				(void* )NULL, 							//���ݸ��������Ĳ���
				(UBaseType_t )APP_INFO_TASK_PRIO, 		//�������ȼ�
				(TaskHandle_t* )&AppInfoTask_Handler); 	//������
	
//	/* ���ж�ȡ */
//	xTaskCreate((TaskFunction_t )Info_JudgeRead_Task,	 //������
//				(const char* )"Info_Update_Task", 		//��������
//				(uint16_t )STK_SIZE_128, 				//�����ջ��С
//				(void* )NULL, 							//���ݸ��������Ĳ���
//				(UBaseType_t )APP_INFO_TRANSMIT_PRIO, 	//�������ȼ�
//				(TaskHandle_t* )&AppInfoTransmit_Handler); 	//������

   /* �Ӿ������з��� */
	xTaskCreate((TaskFunction_t )Info_Transmitter_Task, //������
				(const char* )"Info_Update_Task", 		//��������
				(uint16_t )STK_SIZE_128, 				//�����ջ��С
				(void* )NULL, 							//���ݸ��������Ĳ���
				(UBaseType_t )APP_INFO_JUDGEREAD_PRIO, 	//�������ȼ�
				(TaskHandle_t* )&AppInfoJudge_Handler); 	//������
				
	taskEXIT_CRITICAL(); 								//�˳��ٽ���
}

/**
  * @brief  ״̬��������
  * @param  void
  * @retval void
  * @attention ����ϵͳ����״̬,1ms�����ʱ
  */
void Info_Update_Task(void *pvParameters)
{
	static portTickType currentTime;
	
	for(;;)
	{
		vTaskDelay(TIME_STAMP_1MS);				//1ms
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��	
		/* ���ݳ����������� */
		if( REMOTE_IfDataError() == TRUE)
		{
			SYSTEM_OutCtrlProtect();
		}
		else
		{
			IWDG_Feed();//ι��
		}
		
		if(currentTime >= REMOTE_ulGetLostTime( ))//ң��ʧ��ʱ��̫��
		{
			SYSTEM_Reset();//ϵͳ�ָ�������״̬
		}
		else
		{
			SYSTEM_UpdateSystemState();//����ϵͳ״̬,����������״̬
		}
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//������ʱ
	}
}

