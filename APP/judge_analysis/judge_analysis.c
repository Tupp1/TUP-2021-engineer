
#define __DRIVER_GLOBALS
#include "judge_analysis.h"
#include "string.h"
#include "crc_check.h"





void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength_SH])//裁判系统解算
{	
	static uint16_t start_pos=0,next_start_pos=0;
	while(1)
	{
		memcpy(&JudgementData.frameHeader, &JudgeDataBuffer[start_pos],FrameHeader_Len);
		/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
		if((JudgementData.frameHeader.SOF==(uint16_t)JudgeFrameHeader) \
    &&(1==Verify_CRC8_Check_Sum(&JudgeDataBuffer[start_pos],FrameHeader_Len)) \
    &&(1==Verify_CRC16_Check_Sum(&JudgeDataBuffer[start_pos], JudgementData.frameHeader.DataLenth+FrameHeader_Len+4)))//数据位长度+帧头长度+命令码长度+校验码长度
		{
			memcpy(&JudgementData.rxCmdId, (&JudgeDataBuffer[start_pos]+5), sizeof(JudgementData.rxCmdId));
			JudgeDataBuffer[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
			next_start_pos=start_pos+9+JudgementData.frameHeader.DataLenth;//9为 5位帧头 2位数据长度 2校验位
			switch(JudgementData.rxCmdId)
			{
				case Robot_Status_ID://读取机器人等级血量等
				{
					memcpy(&JudgementData.game_robot_state_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case power_heat_data_ID://读取机器人枪口热量
				{
					memcpy(&JudgementData.power_heat_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}	
				break;
				case shoot_data_ID://读取机器人射频射速
				{
					memcpy(&JudgementData.shoot_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case robot_hurt_ID://读取机器人射频射速
				{
					memcpy(&JudgementData.robot_hurt_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case student_interactive_header_ID:
        {
					memcpy(&JudgementData.userinfo,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
			}
			start_pos=next_start_pos;
		}
		else
		{
			start_pos=0;
			break;				
		}		
		/**如果头指针越界了退出循环**/
		if(start_pos>JudgeBufferLength_SH)
		{
			start_pos=0;
			break;
		}
	}
}



uint8_t JudgeUARTtemp;
//Judge
void USART3_IRQHandler(void)
{	
    JudgeUARTtemp = USART3->DR;
    JudgeUARTtemp = USART3->SR;
    
    DMA_Cmd(DMA1_Stream1, DISABLE);
			
	  judgeCalculate(JudgeDataBuffer_SH);
		
    //重启DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream1, JudgeBufferLength_SH);
    DMA_Cmd(DMA1_Stream1, ENABLE);
}

void USART3_SendBuff (uint8_t *buf,uint16_t len)
{
  int i;
	for(i=0; i<len;i++)
	{
    USART_SendData(USART3,*((u8*)&buf+i));
	}
while( USART_GetFlagStatus(USART3 ,USART_FLAG_TC) == RESET );
        
}




tFrame Frame = {.FrameHeader.SOF = 0xA5};
uint8_t tx_buf[200];
void Send_FrameData(uint16_t cmdid, uint8_t * pchMessage,uint16_t dwLength)
{	
	uint16_t frame_length = HEADER_LEN + CMD_LEN + dwLength + CRC_LEN;
	
	FrameHeader_t *p_header = (FrameHeader_t*)tx_buf;
  
  p_header->SOF          = 0xA5;
  p_header->DataLength   = dwLength;
  p_header->Seq          = 0;
	 
	memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmdid, CMD_LEN); 
  Append_CRC8_Check_Sum(tx_buf, HEADER_LEN);  
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], pchMessage, dwLength);
  Append_CRC16_Check_Sum(tx_buf, frame_length);
	
  USART3_SendBuff((uint8_t *)&tx_buf,frame_length);               //帧发送	
	
} 


//extern uint8_t client_custom_Send[28];
//extern uint8_t  robot_custom_Send[28];
void client_send(uint8_t * data)
{
		JudgementData.userinfo.id.send_id=JudgementData.game_robot_state_t.robot_id;
	
		if(JudgementData.game_robot_state_t.robot_id>10)
			JudgementData.userinfo.id.receive_id=0x0106+JudgementData.userinfo.id.send_id;
		else
			JudgementData.userinfo.id.receive_id=0x0100+JudgementData.userinfo.id.send_id;


		JudgementData.userinfo.data3=8;

		memcpy(data,&JudgementData.userinfo,28);
		Append_CRC16_Check_Sum(data,28);
		USART3_SendBuff(data,28);
}

void robot_send(uint8_t * data)
{
		JudgementData.robot_data_t.id.send_id=JudgementData.game_robot_state_t.robot_id;
		if(JudgementData.game_robot_state_t.robot_id<10)
			JudgementData.robot_data_t.id.receive_id=3;
		else 
			JudgementData.robot_data_t.id.receive_id=17;		
		memcpy(data,&JudgementData.robot_data_t,28);
		Append_CRC16_Check_Sum(data,28);
		USART3_SendBuff(data,28);
}

void client_init(uint8_t * data1,uint8_t * data2)
{
		memcpy(&JudgementData.userinfo,data1,28);
		memcpy(&JudgementData.robot_data_t,data2,28);	
}

