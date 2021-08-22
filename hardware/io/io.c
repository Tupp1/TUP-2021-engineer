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
	
	//�����
	GPIO_SetBits(GPIOI,GPIO_Pin_0);     //������ر�	
	//�ĸ�����		
	GPIO_ResetBits(GPIOI,GPIO_Pin_2);   //��ȡ���׳�ʼ��״̬ ����û���ϵ�Ҳ�Ǵ�״̬
	GPIO_ResetBits(GPIOI,GPIO_Pin_7);   //��ȡ��������
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);  //��Ԯ���� 
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);  //��������
			
}

