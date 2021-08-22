#ifndef TIMER_H
#define TIMER_H
#include "main.h"



#define Get_Time_Micros() (TIM5->CNT)
#define INERLOOPLENGTH 11


extern void TIM6_Init(uint16_t arr, uint16_t psc);
extern void TIM5_Init(void);
extern uint32_t GetInnerLoop(int loop);
extern void InnerLoopInit(void);
extern void TIM3_Init(uint16_t arr, uint16_t psc);
	

#endif
