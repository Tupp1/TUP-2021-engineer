#ifndef USART_H
#define USART_H
#include "main.h"
#include "stm32f4xx.h"

#define VisionBufferLength_SH       150
#define    JUDGE_BUFFER_LEN           200






void USART6_Vision_Config(void);

void USART3_Judge_Config(void);
void UART8_Datatransfer_Config(u32 br_num);
void UART8_Send(unsigned char *DataToSend ,u8 data_num);


extern uint8_t VisionDataBuffer_SH[VisionBufferLength_SH];
extern uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ];


#endif
