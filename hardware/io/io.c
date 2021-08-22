#include "io.h"
#include "stm32f4xx.h"
#include "system.h"

void io_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOI, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//电磁锁
	GPIO_SetBits(GPIOI,GPIO_Pin_0);     //电磁锁关闭	
	//四个气缸		
	GPIO_ResetBits(GPIOI,GPIO_Pin_2);   //夹取气缸初始打开状态 板子没有上电也是打开状态
	GPIO_ResetBits(GPIOI,GPIO_Pin_7);   //夹取伸缩气缸
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);  //救援气缸 
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);  //除障气缸
			
}

