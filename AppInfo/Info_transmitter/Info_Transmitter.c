#include "Info_Transmitter.h"
#include "dataset.h"
#include "judge.h"
#include "usart.h"
#include "remote.h"

int32_t count = 0;
void Info_Transmitter_Task(void *pvParameters)
{
	for(;;)
	{
		vTaskDelay(TIME_STAMP_150MS);			//100ms
		count ++;
		DatatransferTask();
		if(count < 5)
		{
			Judge_Engineer_UI();
			UI_Init();
		}
		else
		{
			Judge_Show1_Data();
		}
	}
}

void Info_JudgeRead_Task(void *pvParameters)
{
	for(;;)
	{
		vTaskDelay(TIME_STAMP_2MS);			//2ms
		
		Judge_Read_Data(Judge_Buffer);		//读取裁判系统数据	
	}
}
