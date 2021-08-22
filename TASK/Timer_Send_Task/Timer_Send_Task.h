#ifndef _TIMER_SEND_TASK_H
#define _TIMER_SEND_TASK_H

#include "system.h"	

void Timer_Send_Create(void);

void CAN1_Timer_Callback( TimerHandle_t xTimer );
void CAN2_Timer_Callback( TimerHandle_t xTimer );


/* 队列 */
extern QueueHandle_t	CAN1_Queue;					//CAN1消息队列句柄
extern QueueHandle_t	CAN2_Queue;					//CAN2消息队列句柄
#endif
