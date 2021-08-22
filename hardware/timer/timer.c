#include "timer.h"
#include "stm32f4xx.h"
//f = (system clock/prescaler)/Period

void TIM6_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, DISABLE);

    TIM_DeInit(TIM6);
    TIM_TimeBaseInitStructure.TIM_Period = arr - 1; 
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1; 
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM6_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM6, ENABLE); 
}




//TIM5用来测试任务代码执行时间
/**
  * @brief TIM5初始化
  * @param None
  * @retval None
  * @details 	TIM5是一个32位的寄存器，可以用来纯粹的计时，记录从系统开始到现在所
	*						经过的微秒数
  */
void TIM5_Init(void)										
{
   TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 90 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM5, ENABLE);	
    TIM_TimeBaseInit(TIM5, &tim);
    TIM_Cmd(TIM5,ENABLE);	
}


/**
  * @brief TIM5的溢出中断
  * @param None
  * @retval None
  * @details 执行清除中断位的操作
  */
void TIM5_IRQHandler(void)										
{
	  if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
		{
			  TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
        TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		}
} 


/**
  * @brief 初始化GetInnerLoop功能
  * @param None
  * @retval None
  */
void InnerLoopInit(void)
{
	int i=0;
	for (i=0;i<INERLOOPLENGTH;i++)
	{
		GetInnerLoop(i);
	}
}	

/**
  * @brief 得到某函数的精准的调用周期
  * @param 计时序号，在头文件中可以查到
  * @retval 执行周期
  */
uint32_t GetInnerLoop(int loop)								//用于获得精确的函数调用的周期
{
	static uint32_t Time[2][20]={0};//Time[0] is the last time, Time[1] is the new time;
	Time[0][loop] = Time[1][loop];
	Time[1][loop] = Get_Time_Micros();
	return Time[1][loop]-Time[0][loop];
}

void TIM3_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM3_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /* TIM3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}

