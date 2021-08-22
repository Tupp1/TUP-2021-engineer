#ifndef _TIMER_SEND_TASK_H
#define _TIMER_SEND_TASK_H

#include "system.h"	

void Timer_Send_Create(void);

void CAN1_Timer_Callback( TimerHandle_t xTimer );
void CAN2_Timer_Callback( TimerHandle_t xTimer );


/* ���� */
extern QueueHandle_t	CAN1_Queue;					//CAN1��Ϣ���о��
extern QueueHandle_t	CAN2_Queue;					//CAN2��Ϣ���о��
#endif
