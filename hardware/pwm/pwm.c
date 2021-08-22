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
	TIM_TimeBaseStructure.TIM_Period = (22140);      				   	//����ʱ����0������ TIM_Period+1 ��Ϊһ����ʱ����
	TIM_TimeBaseStructure.TIM_Prescaler = (64);	                //����Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	    //����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //���ϼ���ģʽ 	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);		
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	            //����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.TIM_Pulse = 0;	  												//���ó�ʼPWM������Ϊ0	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      //����ʱ������ֵС��CCR1_ValʱΪ�͵�ƽ LED���� 
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);	                    //ʹ��ͨ�� 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);		                    	//ʹ��TIM���ؼĴ���ARR
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
	
  TIM_SetCompare1(TIM2,2050);   //PA0����1��2050 
  TIM_SetCompare2(TIM2,2450);	  //PA1����2��2450 
	TIM_SetCompare3(TIM2,2850);	  //PA2ͼ����
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

