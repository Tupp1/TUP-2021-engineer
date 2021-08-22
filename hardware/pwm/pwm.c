#include "pwm.h"
#include "stm32f4xx.h"

void fric_PWM_configuration(void) //pwm
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 90 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);

    TIM_Cmd(TIM4, ENABLE);

}

void TIM_PWM_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2); 	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);	

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 	
	TIM_TimeBaseStructure.TIM_Period = (22140);      				   	//当定时器从0计数到 TIM_Period+1 ，为一个定时周期
	TIM_TimeBaseStructure.TIM_Prescaler = (64);	                //设置预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	    //设置时钟分频系数：不分频(这里用不到)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //向上计数模式 	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);		
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	            //配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//使能输出
	TIM_OCInitStructure.TIM_Pulse = 0;	  												//设置初始PWM脉冲宽度为0	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      //当定时器计数值小于CCR1_Val时为低电平 LED灯亮 
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);	                    //使能通道 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);		                    	//使能TIM重载寄存器ARR
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	TIM_Cmd(TIM2, ENABLE);	
			
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);	
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);		
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
		
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
	
  TIM_SetCompare1(TIM2,2050);   //PA0补给1关2050 
  TIM_SetCompare2(TIM2,2450);	  //PA1补给2关2450 
	TIM_SetCompare3(TIM2,2850);	  //PA2图传中
}

void fric_off(void)
{
    TIM_SetCompare1(TIM4, Fric_OFF);
    TIM_SetCompare2(TIM4, Fric_OFF);
}
void fric1_on(uint16_t cmd)
{
    TIM_SetCompare1(TIM4, cmd);
}
void fric2_on(uint16_t cmd)
{
    TIM_SetCompare2(TIM4, cmd);
}

