#include "usart.h"
#include "stm32f4xx.h"

#include "dataset.h"


//裁判系统发过来的数据暂存在这里
uint8_t Judge_Buffer[JUDGE_BUFFER_LEN] = {0};

uint8_t  Com6_Vision_Buffer[ VisionBufferLength_SH ] = {0};
//Vision transform
//RX PG9  TX PG14
void USART6_Vision_Config(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef	NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|RCC_APB2Periph_USART6,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14  ;  
		GPIO_Init(GPIOG, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
		GPIO_Init(GPIOG, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
		
		USART_InitStructure.USART_BaudRate              =   115200;
		USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode                  =   USART_Mode_Rx|USART_Mode_Tx;
		USART_InitStructure.USART_Parity                =   USART_Parity_No;
		USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
		USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
		
		USART_Init(USART6, &USART_InitStructure);
	  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	 
	  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART6, ENABLE);

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		DMA_DeInit(DMA2_Stream1);
		while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE)  {
		}
		//DMA2  USART6  VISION  通道5  数据流1
		DMA_InitStructure.DMA_Channel           =   DMA_Channel_5 ;
		DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&USART6->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(Com6_Vision_Buffer);
		DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize        =   VisionBufferLength_SH;
		DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_Mode              =   DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority          =   DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Enable;
		DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
		DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream1, &DMA_InitStructure);
		DMA_Cmd(DMA2_Stream1, ENABLE);
		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

	  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
		USART_Cmd(USART6, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;			   
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;	   
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;             
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	             
		NVIC_Init(&NVIC_InitStructure);	

}




void USART3_Judge_Config(void)
{
	
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  //USART3 PD8 TX PD9 RX
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	
	
	USART_InitTypeDef   USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//Dbus Judge USART3_RX DMA1 Channel4 Stream1 
	USART_InitStructure.USART_BaudRate              =   115200;
	USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                  =   USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_Parity                =   USART_Parity_No;
	USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
	USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART3, ENABLE);
	
	
	NVIC_InitTypeDef	NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//Dudge数据读取
	NVIC_InitStructure.NVIC_IRQChannel						=	USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
  NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

  DMA_InitTypeDef     DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Stream1);
  while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE)  {
  }
	//Dbus Judge USART3_RX DMA1 Channel4 Stream1 
	DMA_InitStructure.DMA_Channel           =   DMA_Channel_4 ;
	DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(Judge_Buffer);
	DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize        =   100;
	DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority          =   DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream1, ENABLE);
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

}





/******************串口8变量*********************/
//数传通信
u8 Rx_8_Buf[256];	
u8 Tx8Buffer[256];
u8 Tx8Counter=0;
u8 count8=0; 
u8 Tx8DMABuffer[256]={0};
/******************串口8变量*********************/
/**
  * @brief 串口8初始化 
  * @param BaudRate
  * @retval None
  * @details	PE0	Rx  DMA1 S6 C5
	*						PE1	Tx  DMA1 S0 C5 
	*						BaudRate	500000
	*						使能DMA发送，RXNE中断
  */
void UART8_Datatransfer_Config(u32 br_num)
{ 
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_UART8);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_UART8);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = br_num;     
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 
	USART_InitStructure.USART_Parity = USART_Parity_No; 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable; 
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge; 
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;
	
	USART_Init(UART8, &USART_InitStructure);
	USART_ClockInit(UART8, &USART_ClockInitStruct);

	USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART8, ENABLE); 
	USART_DMACmd(UART8,USART_DMAReq_Tx,ENABLE);

	DMA_DeInit(DMA1_Stream0);
	
	while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE){}
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART8->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx8DMABuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	
}

/**
  * @brief 串口8中断
  * @param None
  * @retval None
  * @details RXNE中断，由此进入Usart3通讯协议解析
  */
void UART8_IRQHandler(void)
{
	u8 com_data;
	
	if(UART8->SR & USART_SR_ORE)
	{
		com_data = UART8->DR;
	}
	if( USART_GetITStatus(UART8,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART8,USART_IT_RXNE);

		com_data = UART8->DR;
		UART8_DataPrepare(com_data);
	}
}

/**
  * @brief 串口8的DMA发送函数，发送一组数据
  * @param DataToSend 要发送数据的数组的指针
	* @param data_num 要发送的数据的个数
  * @retval None
  */
void UART8_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	static uint16_t num=0;
	static u8 len=0;
	
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);//清除DMA1_Steam0传输完成标志
	num = DMA_GetCurrDataCounter(DMA1_Stream0);
	for(i=0;i<data_num;i++)
	{
		Tx8Buffer[count8++] = *(DataToSend+i);
	}
	for (i=0;i<(u8)num;i++)
	{
		Tx8DMABuffer[i]=Tx8Buffer[((u8)(len-num+i))];
	}
	for (;i<(u8)(num+data_num);i++)
	{
		Tx8DMABuffer[i]=*(DataToSend+i-num);
	}
	len=count8;
	while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE){}	//确保DMA可以被设置  
	DMA1_Stream0->NDTR = (uint16_t)(num+data_num);          //数据传输量  
	DMA_Cmd(DMA1_Stream0, ENABLE);       

}




