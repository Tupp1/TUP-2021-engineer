#include "judge.h"
#include "string.h"
#include "crc_check.h"
#include "usart.h"
#include "Revolver_task.h"
#include "remote.h"
#include "UDTrans_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"

/*-------------------------2018--------------------------------*/
#if JUDGE_VERSION == JUDGE_18
/*****************ϵͳ���ݶ���**********************/
extGameRobotState_t       RobotState;		//0x0001
extRobotHurt_t            HurtData;			//0x0002
extShootData_t            ShootData;		//0x0003
extPowerHeatData_t        PowerHeatData;	//0x0004
extRfidDetect_t           RfidDetect;		//0x0005
extGameResult_t			  GameResultData;	//0x0006
extGetBuff_t			  GetBuffData;		//0x0007
extGameRobotPos_t		  GameRobotPosData;	//0x0008


xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
xShowData                 ShowData;
/****************************************************/

bool Judge_Data_TF = FALSE;//���������Ƿ����,������������

/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������


//��ǰ�ȼ���Ӧ����������,18���
#define HEAT_LEVEL1 120         //240
#define HEAT_LEVEL2 240         //360
#define HEAT_LEVEL3 480         //480

//��ǰ�ȼ���Ӧ��ǹ����ȴ
#define COLD_LEVEL1 120         //40
#define COLD_LEVEL2 240         //60
#define COLD_LEVEL3 480         //80

portTickType shoot_time;//������ʱ����

portTickType shoot_ping;//����������շ����ӳ�

/*********************************����ϵͳ���ݶ�ȡ**************************************/

uint8_t JudgeUARTtemp;
//Judge
void USART3_IRQHandler(void)
{	
    JudgeUARTtemp = USART3->DR;
    JudgeUARTtemp = USART3->SR;
    
    DMA_Cmd(DMA1_Stream1, DISABLE);
			
	  Judge_Read_Data(Judge_Buffer);
		
    //����DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream1, JUDGE_BUFFER_LEN);
    DMA_Cmd(DMA1_Stream1, ENABLE);
}



/**
  * @brief  ��ȡ��������,loop��ѭ�����ô˺�������ȡ����
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д������
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_STATE:         //0x0001
						memcpy(&RobotState, (ReadFromUsart + DATA), LEN_STATE);
					break;
					
					case ID_HURT:          //0x0002
						memcpy(&HurtData, (ReadFromUsart + DATA), LEN_HURT);
						if(HurtData.hurtType == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
						
					break;
					
					case ID_SHOOT:         //0x0003
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_SHOOT);
						JUDGE_ShootNumCount();//������ͳ��,��������˫ǹ��,��׼
					break;
					
					case ID_POWER_HEAT:    //0x0004
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_POWER_HEAT);
					break;
					
					case ID_RFID:          //0x0005
						memcpy(&RfidDetect, (ReadFromUsart + DATA), LEN_RFID);
					break;
					
					case ID_GAME_RESULT:   //0x0006
						memcpy(&GameResultData, (ReadFromUsart + DATA), LEN_GAME_RESULT);
					break;
					
					case ID_BUFF_GET:      //0x0007
						memcpy(&GetBuffData, (ReadFromUsart + DATA), LEN_BUFF_GET);
					break;
					
					case ID_POSITION:      //0x0008
						memcpy(&GameRobotPosData, (ReadFromUsart + DATA), LEN_POSITION);
					break;
				}
				
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}
	
	return retval_tf;//����������������
}


/**************************�û��Զ��������ϴ����ͻ���******************************/

/**
  * @brief  �ϴ��Զ�������
  * @param  void
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void JUDGE_Show_Data(void)
{
	//��ʵ����22�ͺ�,������ռ�Ҳ����ν
	uint8_t Show_Pack[50] = {0};//���ȴ���ͷ5+ָ��2+����13+β2����,�����u8��Ϊ�˺ü����ֽ���
	int i;//ѭ�����ʹ���
	
	//֡ͷЭ��
	FrameHeader.SOF        = JUDGE_FRAME_HEADER;
	FrameHeader.Seq        = 0;		//Ϊʲô��0,�ٷ�����û��˵
	FrameHeader.DataLength = LEN_SHOW;
	
	//д��֡ͷ
	memcpy( Show_Pack, &FrameHeader, LEN_HEADER );
	
	//֡ͷCRC8У��Э��
	Append_CRC8_Check_Sum( Show_Pack, LEN_HEADER );
	
	//д��������
	ShowData.CmdID = ID_SHOW;
	
	//д������
	//ShowData.data1 = Capvoltage_Percent();//��ʾ����ʣ�����      //(float)ShootNum;//��һ����ʾ���Ƿ�����
	ShowData.data2 = 66.66;//Fric_GetHeatInc();//�ڶ�����ʾ��ǰ����(Ŀ������)
	ShowData.data3 = 88.88;
	ShowData.mask  = 0x00;//��ȫ��,��6λ��Ч
	
//	if(VISION_IfAutoRed() == TRUE)
//	{
//		ShowData.mask &= 0x1f;//��6λ��0
//	}
//	else if(VISION_IfAutoRed() == FALSE)
//	{
//		ShowData.mask |= 0x20;//��6λ��1
//	}
//	
	
	memcpy( Show_Pack + LEN_HEADER, &ShowData, (LEN_CMDID + LEN_SHOW) );
	
	//֡βCRC16У��Э��
	Append_CRC16_Check_Sum( Show_Pack, (LEN_HEADER + LEN_CMDID + LEN_SHOW + LEN_TAIL) );
	
	//������õ�����ͨ��������λ���͵�����ϵͳ
	for (i = 0; i < (LEN_HEADER + LEN_CMDID + LEN_SHOW + LEN_TAIL); i++)
	{
		//UART5_SendChar( Show_Pack[i] );
	}
}


/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassisPower);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
float JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassisPowerBuffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	RobotState.robotLevel;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooterHeat0;
}

/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  
  */
void JUDGE_ShootNumCount(void)
{
	ShootNum++;
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping = shoot_time - REVOL_uiGetRevolTime();//�����ӳ�
}

/**
  * @brief  ��ȡ������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
  */
uint16_t JUDGE_usGetShootNum(void)
{
	return ShootNum;
}

/**
  * @brief  ����������
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear(void)
{
	ShootNum = 0;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ���������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	if (RobotState.robotLevel == 1)//1��
	{
		return HEAT_LEVEL1;
	}
	else if (RobotState.robotLevel == 2)//2��
	{
		return HEAT_LEVEL2;
	}
	else if (RobotState.robotLevel == 3)//3��
	{
		return HEAT_LEVEL3;
	}
	else//��ֹ����������,ǿ���޵���С
	{
		return HEAT_LEVEL1;
	}
}

/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ���ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	if (RobotState.robotLevel == 1)//1��
	{
		return COLD_LEVEL1;
	}
	else if (RobotState.robotLevel == 2)//2��
	{
		return COLD_LEVEL2;
	}
	else if (RobotState.robotLevel == 3)//3��
	{
		return COLD_LEVEL3;
	}
	else//��ֹ����������,ǿ���޵���С
	{
		return COLD_LEVEL1;
	}
}

/****************�����Զ������ж���*******************/
/**
  * @brief  װ�װ��˺������Ƿ����
  * @param  void
  * @retval TRUE�Ѹ���   FALSEû����
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//Ĭ��װ�װ崦������״̬

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//װ�װ����ݸ���
	{
		Hurt_Data_Update = FALSE;//��֤���жϵ��´�װ�װ��˺�����
		ulDelay = ulCurrent + TIME_STAMP_200MS;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}



/************************��������Ԥ���ú���****************************/

/**
  * @brief  ����ʣ�๦��
  * @param  void
  * @retval ʣ�๦��W
  * @attention  ����ʼ�ʵʱ����,�������ݱ���
  */
float JUDGE_fGetChassisResiduePower(void)
{
	return (80 - PowerHeatData.chassisPower);
}

/**
  * @brief  ���㹩���������ݵĵ���ֵ
  * @param  void
  * @retval ��������ֵ
  * @attention  �������ݱ���
  */
float JUDGE_fGetSuper_Cap_Ele(void)
{
	return ((80 - PowerHeatData.chassisPower) / PowerHeatData.chassisCurrent);
}


/*-------------------------2019--------------------------------*/
#elif JUDGE_VERSION == JUDGE_19

/*****************ϵͳ���ݶ���**********************/
ext_game_state_t       				GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_survivors_t          GameRobotSurvivors;			//0x0003
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_supply_projectile_booking_t		SupplyProjectileBooking;	//0x0103
ext_game_robot_state_t			  	GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_musk_t						BuffMusk;					//0x0204
aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207

xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
ext_SendClientData_t      ShowData;			//�ͻ�����Ϣ
ext_CommunatianData_t     CommuData;		//����ͨ����Ϣ
/****************************************************/

bool Judge_Data_TF = FALSE;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID


/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������
#define BLUE  0
#define RED   1

/**
  * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_survivors:       //0x0003
						memcpy(&GameRobotSurvivors, (ReadFromUsart + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_supply_projectile_booking:  //0x0103
						memcpy(&SupplyProjectileBooking, (ReadFromUsart + DATA), LEN_supply_projectile_booking);
					break;
					
					case ID_game_robot_state:      		//0x0201
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						JUDGE_ShootNumCount();//������ͳ
					break;
				}
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
//				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
//				{
//					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
//					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
//				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}
	
	return retval_tf;//����������������
}

/**
  * @brief  �ϴ��Զ�������
  * @param  void
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
void JUDGE_Show_Data(void)
{
	static u8 datalength,i;
	uint8_t judge_led = 0xff;//��ʼ��ledΪȫ��
	static uint8_t auto_led_time = 0;
	static uint8_t buff_led_time = 0;
	
	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
	
	ShowData.txFrameHeader.SOF = 0xA5;
	ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
	ShowData.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//д��֡ͷCRC8У����
	
	ShowData.CmdID = 0x0301;
	
	ShowData.dataFrameHeader.data_cmd_id = 0xD180;//�����ͻ��˵�cmd,�ٷ��̶�
	//ID�Ѿ����Զ���ȡ����
	ShowData.dataFrameHeader.send_ID 	 = Judge_Self_ID;//�����ߵ�ID
	ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	
	/*- �Զ������� -*/
	ShowData.clientData.data1 = (float)Capvoltage_Percent();//����ʣ�����
	ShowData.clientData.data2 = (float)Base_Angle_Measure();//����ǶȲ�
	ShowData.clientData.data3 = GIMBAL_PITCH_Judge_Angle();//��̨̧ͷ�Ƕ�
	
	/***************Ť��ָʾ****************/
	if(Chassis_IfCORGI() == TRUE)//Ťƨ��ģʽ,��һ�ŵ�����
	{
		judge_led &= 0xfe;//��1λ��0,���
	}
	else
	{
		judge_led |= 0x01;//��1λ��1
	}
	
	/****************45��ģʽ****************/
	if(Chassis_IfPISA() == TRUE)//45��ģʽ���ڶ��ŵ�����
	{
		judge_led &= 0xfd;//��2λ��0,���
	}
	else
	{
		judge_led |= 0x02;//��2λ��1
	}
	
	/***************����ָʾ****************/
	if(VISION_isColor() == ATTACK_RED)//�����
	{
		judge_led &= 0xfb;//��3λ��0,���
	}
	else if(VISION_isColor() == ATTACK_BLUE)//������ɫ
	{
		judge_led |= 0x04;//��3λ��1
	}
	else//������
	{
		auto_led_time++;
		if(auto_led_time > 3)
		{
			(judge_led)^=(1<<2);//��3λȡ��,��˸
			auto_led_time = 0;
		}
	}
	
	/****************���ָʾ***************/
	if(VISION_BuffType() == VISION_RBUFF_CLOCKWISE	//˳ʱ��
			|| VISION_BuffType() == VISION_BBUFF_CLOCKWISE)
	{
		judge_led &= 0xf7;//��4λ��0,���
	}
	else if(VISION_BuffType() == VISION_RBUFF_ANTI	//��ʱ��
			|| VISION_BuffType() == VISION_BBUFF_ANTI)
	{
		judge_led |= 0x08;//��4λ��1
	}
	else//�����
	{
		buff_led_time++;
		if(buff_led_time > 3)
		{
			(judge_led)^=(1<<3);//��4λȡ��,��˸
			buff_led_time = 0;
		}
	}
	/*--------------*/
	ShowData.clientData.masks = judge_led;//0~5λ0���,1�̵�
	
	//���д�����ݶ�
	memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&ShowData.CmdID, 
			(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
		  );			
			
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ShowData));//д�����ݶ�CRC16У����	

	datalength = sizeof(ShowData); 
	for(i = 0;i < datalength;i++)
	{
		USART_SendData(UART5,(uint16_t)CliendTxBuffer[i]);
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
	}	 
}

/**
  * @brief  �������ݸ�����
  * @param  void
  * @retval void
  * @attention  
  */
#define Teammate_max_len     200
unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
bool First_Time_Send_Commu = FALSE;
uint16_t send_time = 0;
void Send_to_Teammate(void)
{
	static u8 datalength,i;
	
	Send_Color = is_red_or_blue();//�жϷ��͸��ڱ�����ɫ,17�ڱ�(��),7�ڱ�(��)��
	
	memset(TeammateTxBuffer,0,200);
	
	CommuData.txFrameHeader.SOF = 0xA5;
	CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
	CommuData.txFrameHeader.Seq = 0;
	memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
	Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));	
	
	CommuData.CmdID = 0x0301;
	
	   
	CommuData.dataFrameHeader.send_ID = Judge_Self_ID;//�����ߵ�ID
	
	
	if( Senty_Run() == TRUE)
	{
		Senty_Run_Clean_Flag();
		First_Time_Send_Commu = TRUE;
	}
	
	if( First_Time_Send_Commu == TRUE )
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//��0x0200-0x02ff֮��ѡ��
		send_time++;
		if(send_time >= 20)
		{
			First_Time_Send_Commu = FALSE;
		}
		if(Send_Color == BLUE)//�Լ��������������ڱ�
		{
			CommuData.dataFrameHeader.receiver_ID = 17;//������ID
		}
		else if(Send_Color == RED)//�Լ��Ǻ죬�������ڱ�
		{
			CommuData.dataFrameHeader.receiver_ID = 7;//������ID
		}
	}
	else
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0255;
		send_time = 0;
		CommuData.dataFrameHeader.receiver_ID = 88;//������ID��������
	}
	
	CommuData.interactData.data[0] = 0;//���͵����� //��С��Ҫ���������ı�������   
	
	memcpy(TeammateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
	Append_CRC16_Check_Sum(TeammateTxBuffer,sizeof(CommuData));
	
	datalength = sizeof(CommuData); 
	if( First_Time_Send_Commu == TRUE )
	{
		for(i = 0;i < datalength;i++)
		{
			USART_SendData(UART5,(uint16_t)TeammateTxBuffer[i]);
			while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
		}	 
	}
}

/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//��ȡ��ǰ������ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}
}

/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_heat0;
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time;//������ʱ����
portTickType shoot_ping;//����������շ����ӳ�
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	Shoot_Speed_Now = ShootData.bullet_speed;
	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum++;
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping = shoot_time - REVOL_uiGetRevolTime();//�����ӳ�
}

/**
  * @brief  ��ȡ������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
  */
uint16_t JUDGE_usGetShootNum(void)
{
	return ShootNum;
}

/**
  * @brief  ����������
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear(void)
{
	ShootNum = 0;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return GameRobotStat.shooter_heat0_cooling_limit;
}

/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	return GameRobotStat.shooter_heat0_cooling_rate;
}

/****************�����Զ������ж���*******************/
/**
  * @brief  װ�װ��˺������Ƿ����
  * @param  void
  * @retval TRUE�Ѹ���   FALSEû����
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//Ĭ��װ�װ崦������״̬

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//װ�װ����ݸ���
	{
		Hurt_Data_Update = FALSE;//��֤���жϵ��´�װ�װ��˺�����
		ulDelay = ulCurrent + TIME_STAMP_200MS;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}

bool Judge_If_Death(void)
{
	if(GameRobotStat.remain_HP == 0 && JUDGE_sGetDataState() == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*-------------------------2021--------------------------------*/
#elif JUDGE_VERSION == JUDGE_21
/*****************ϵͳ���ݶ���**********************/
ext_game_status_t                   Game_status;
ext_game_result_t                   Game_result;
ext_game_robot_HP_t                 Robot_HP;
ext_dart_status_t                   Dart_status;
ext_ICRA_buff_debuff_zone_status_t  ICRA_buff_debuff_zone_status;
ext_event_data_t                    Event_data;
ext_supply_projectile_action_t      Supply_projectile_action;
ext_referee_warning_t               Referee_warning;
ext_dart_remaining_time_t           Dart_remaining_time;
ext_game_robot_status_t             Robot_status;
ext_power_heat_data_t               Power_heat_data;
ext_game_robot_pos_t                robot_position;
ext_buff_t                          buff;
aerial_robot_energy_t         		Aerial_robot_energy;
ext_robot_hurt_t                    Robot_hurt;
ext_shoot_data_t                    Shoot_data;
ext_bullet_remaining_t              Bullet_remaining;
ext_rfid_status_t                   Rfid_status;
ext_dart_client_cmd_t               Dart_client_cmd;


ext_robot_command_t					Minimap_robot_command;	
ext_client_map_command_t			Client_map_command_t;
ext_robot_command_t_change			Transfer_image_robot_command;


xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
ext_SendClientData_t      ShowData;			//�ͻ�����Ϣ
ext_CommunatianData_t     CommuData;		//����ͨ����Ϣ
ext_SendClientGraph_t     ClientGraph;  //ͼ��ͨ��
ext_SendClientGraphDelete_t ClientGraphDelete; //ͼ��ɾ��

extern Gimbal_Control_t gimbal_control;
extern UDTrans_Control_t UDTrans_Control;
extern chassis_move_t chassis_move;
extern uint8_t supply1_flag;    	//����1��־λ    0Ϊ�أ�1Ϊ��
extern uint8_t supply2_flag;	 	//����2��־λ    0Ϊ�أ�1Ϊ��
extern uint8_t help_flag;       	//��Ԯ��־λ     0Ϊ�أ�1Ϊ��
extern uint8_t snag_flag;	      	//���ϱ�־λ     0Ϊ�أ�1Ϊ��
extern uint8_t clamp_flag;    		//��ȡ��־λ     0Ϊ�أ�1Ϊ��
extern uint8_t stretch_flag;	  	//������־λ     0Ϊ�أ�1Ϊ��
extern uint8_t lock_flag;
extern uint8_t UI_relative_angle[10];
/****************************************************/
bool Judge_Data_TF = FALSE;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID

/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum_17mm;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
uint16_t ShootNum_42mm;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������

#define BLUE  0
#define RED   1

uint8_t JudgeUARTtemp;
//Judge
void USART3_IRQHandler(void)
{	
    JudgeUARTtemp = USART3->DR;
    JudgeUARTtemp = USART3->SR;
    
    DMA_Cmd(DMA1_Stream1, DISABLE);
			
	  Judge_Read_Data(Judge_Buffer);
		
    //����DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream1, JUDGE_BUFFER_LEN);
    DMA_Cmd(DMA1_Stream1, ENABLE);
}

/**
  * @brief  ��ȡ��������,loop��ѭ�����ô˺�������ȡ����
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д������
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������	

	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
			
				switch(CmdID)
				{
					case ID_game_status:         //0x0001
						memcpy(&Game_status, (ReadFromUsart + DATA), LEN_game_status);
					break;
					
					case ID_game_result:          //0x0002
						memcpy(&Game_result, (ReadFromUsart + DATA), LEN_game_result);						
					break;
					
					case ID_robot_HP:             //0x0003
						memcpy(&Robot_HP, (ReadFromUsart + DATA), LEN_robot_HP);
					break;
					
					case ID_dart_status:          //0x0004
						memcpy(&Dart_status, (ReadFromUsart + DATA), LEN_dart_status);
					break;
					
					case ID_ICRA_buff_debuff_zone_status:          //0x0005
						memcpy(&ICRA_buff_debuff_zone_status, (ReadFromUsart + DATA), LEN_ICRA_buff_debuff_zone_status);
					break;
					
					case ID_event_data:   //0x0101
						memcpy(&Event_data, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:      //0x0102
						memcpy(&Supply_projectile_action, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_referee_warning:      //0x0104
						memcpy(&Referee_warning, (ReadFromUsart + DATA), LEN_referee_warning);
					break;
					
					case ID_dart_remaining_time:      //0x0105
						memcpy(&Dart_remaining_time, (ReadFromUsart + DATA), LEN_dart_remaining_time);
					break;
					
					case ID_robot_status:      //0x0201
						memcpy(&Robot_status, (ReadFromUsart + DATA), LEN_robot_status);
					break;
					
					case ID_heat_data:      //0x0202
						memcpy(&Power_heat_data, (ReadFromUsart + DATA), LEN_heat_data);
					break;
					
					case ID_robot_pos:      //0x0203
						memcpy(&robot_position, (ReadFromUsart + DATA), LEN_robot_pos);
					break;
					
					case ID_buff:      //0x0204
						memcpy(&buff, (ReadFromUsart + DATA), LEN_buff);
					break;
					
					case ID_aerial_robot_energy:      //0x0205
						memcpy(&Aerial_robot_energy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      //0x0206
						memcpy(&Robot_hurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					  if(Robot_hurt.hurt_type == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      //0x0207
						memcpy(&Shoot_data, (ReadFromUsart + DATA), LEN_shoot_data);
					    //JUDGE_ShootNumCount_17mm();//������ͳ��,��������˫ǹ��,��׼
					    JUDGE_ShootNumCount_42mm();//������ͳ��,��������˫ǹ��,��׼
					break;
					
					case ID_bullet_remaining:      //0x0208
						memcpy(&Bullet_remaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;
					
					case ID_rfid_status:      //0x0209
						memcpy(&Rfid_status, (ReadFromUsart + DATA), LEN_rfid_status);
					break;
					
					case ID_dart_client_cmd:      //0x020A
						memcpy(&Dart_client_cmd, (ReadFromUsart + DATA), LEN_dart_client_cmd);
					break;
					
					case ID_minimap_interactive_data:  //0x0303
						memcpy(&Minimap_robot_command, (ReadFromUsart + DATA), LEN_minimap_interactive_data);
					break;

					case ID_keybord_and_mouse_massage:  //0x0304
						memcpy(&Transfer_image_robot_command, (ReadFromUsart + DATA), LEN_keybord_and_mouse_massage);
					break;
					
					case ID_client_map_command:  //0x0304
						memcpy(&Transfer_image_robot_command, (ReadFromUsart + DATA), LEN_client_map_command);
					break;
					
				}
				
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}					
		}		
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}

	return retval_tf;//����������������	
	
}



uint8_t * strcpy1(uint8_t *dst,const uint8_t *src)   
{
    if((dst==NULL)||(src==NULL))
         
           return NULL; 
 
    uint8_t *ret = dst; //[1]
 
    while ((*dst++=*src++)!='\0'); //[2]
 
    return ret;//[3]
}


uint8_t * strcpy2(uint8_t *dst,const char *src)   
{
    if((dst==NULL)||(src==NULL))
         
           return NULL; 
 
    uint8_t *ret = dst; //[1]
 
    while ((*dst++=*src++)!='\0'); //[2]
 
    return ret;//[3]
}



/**
 * @brief  �ϴ��Զ�������
 * @param  void
 * @retval void
 * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
 */

ext_SendClientString_t	CliendStr;
ext_SendClientGraphDelete_t	DataChange;
uint16_t send_max_len = 200;
void Judge_Engineer_UI(void)
{
	unsigned char CliendGraphTxBuffer1[send_max_len];
	unsigned char CliendGraphTxBuffer2[send_max_len];
	u8 datalength1, datalength2, i;
	//�߿�
	uint32_t wide = 2;
	int32_t	a = 12;
	uint32_t color = 4;
	//��̬�ַ�x����
	uint32_t x1 = 300;
	uint32_t x2 = 1470;
	//��̬UI�ַ�����
//	uint32_t x1_1 = 400;
//	uint32_t x2_2 = 1530;
	//��̬�ַ���
	uint8_t str1[30] = "UDT_Mode:";
	uint8_t str2[30] = "Gim_Mode:";
	uint8_t str3[30] = "Cs_Mode:";
	uint8_t str4[30] = "Css_Mode:";
//	uint8_t str5[30] = "Supply1:";
//	uint8_t str6[30] = "Supply2:";
	uint8_t str7[30] = "Help:";
	uint8_t str8[30] = "Snag:";
	uint8_t str9[30] = "Clamp:";
	uint8_t str10[30] = "Stretch:";
	uint8_t str11[30] = "Re_ANGLE:";
	uint8_t str12[30] = "Ele_lock:";
	vTaskDelay(TIME_STAMP_1000MS);

	int sendID;      	
	int receiveID;		
	//�Զ��жϻ�����ID
	sendID=determine_robot_ID();
	switch(sendID)
	{
		//�췽
		case 1: 
				receiveID=0x101;		break;//Ӣ��
		case 2:
				receiveID=0x102;		break;//����
		case 3:
				receiveID=0x103;		break;//����3
		case 4:
				receiveID=0x104;		break;//����4
		case 5:
				receiveID=0x105;		break;//����5		
		case 6:
				receiveID=0x106;		break;//����		
		//����
		case 101: 
				receiveID=0x165;		break;
		case 102:
				receiveID=0x166;		break;
		case 103:
				receiveID=0x167;		break;
		case 104:
				receiveID=0x168;		break;
		case 105:
				receiveID=0x169;		break;		
		case 106:
				receiveID=0x16A;		break;		
				
		default:              			break;				
	}


/************************************************��̬UI*******************************************************/
	//����
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 0;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 9;				//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 820;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;	//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str1);	
	
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
	
	//��̨
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 1;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 12;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 770;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str2);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	//����
	for(i = 0; i < datalength1; i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}
	for(i = 0; i < datalength2; i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}

	//����
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 2;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 12;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 720;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str3);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 

	//������
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 3;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 17;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 670;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str4);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	//���ݷ���
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}

//	//����1
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 4;		//ͼ������
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
//	CliendStr.customStringData.grapic_data_struct.end_angle = 7;			//�ַ�����
//	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
//	CliendStr.customStringData.grapic_data_struct.start_y = 780;				//y����
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
//	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, str5);

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
//	//ID�Ѿ����Զ���ȡ����
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	memcpy(	
//			CliendGraphTxBuffer1 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
//	datalength1 = sizeof(CliendStr); 

//	//����2
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 5;		//ͼ������
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
//	CliendStr.customStringData.grapic_data_struct.end_angle = 7;			//�ַ�����
//	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
//	CliendStr.customStringData.grapic_data_struct.start_y = 730;				//y����
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
//	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, str6);	

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
//	//ID�Ѿ����Զ���ȡ����
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	memcpy(	
//			CliendGraphTxBuffer2 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
//	datalength2 = sizeof(CliendStr); 
//	
//	//���ݷ���
//	for(i = 0;i < datalength1;i++)
//	{
//		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer1[i]);
//		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
//	}
//	for(i = 0;i < datalength2;i++)
//	{
//		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer2[i]);
//		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
//	}

	//��Ԯ
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 6;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 680;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str7);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 

	//����
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 7;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 630;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str8);	

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	//���ݷ���
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}

	//��ȡ
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 8;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 5;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 580;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str9);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 

	//����
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 9;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 7;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 530;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str10);	

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����
    CliendStr.CmdID = 0x0301;
	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);						
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	
	//���ݷ���
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}

	//��ԽǶ�
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 10;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 17;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 450;			//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str11);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
	//�����
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 11;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 17;			//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 480;			//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str12);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	//���ݷ���
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART,USART_FLAG_TC)==RESET);
	}
}


void UI_Init(void)
{
	unsigned char CliendGraphTxBuffer1[send_max_len];
	unsigned char CliendGraphTxBuffer2[send_max_len];
	u8 datalength1, datalength2, i;
	//�߿�
	uint32_t wide = 2;
	int32_t	a = 12;
	//��ɫ
	uint32_t color = 4;
	//��̬UI�ַ�����
	uint32_t x1_1 = 450;//400
	uint32_t x2_2 = 1580;//1530

	//��̬��ʾ�ַ���
	uint8_t UDTrans_Mode[30] = "init";
	uint8_t Gimbal_Mode[30] = "init";
	uint8_t Chassis_Mode[30] = "init";
	uint8_t Chassis_Sub_Mode[30] = "init";
	uint8_t Re_angle[30] = "INIT";
	uint8_t ELE_Lock[30] = "INIT";
//	uint8_t	supply1[30] = "init";
//	uint8_t supply2[30] = "init";
	uint8_t help[30] = "init";
	uint8_t snag[30] = "init";
	uint8_t clamp[30] = "init";
	uint8_t stretch[30] = "init";
	
	vTaskDelay(TIME_STAMP_1000MS);

	int sendID;      	
	int receiveID;		
	//�Զ��жϻ�����ID
	sendID=determine_robot_ID();
	switch(sendID)
	{
		//�췽
		case 1: 
				receiveID=0x101;		break;//Ӣ��
		case 2:
				receiveID=0x102;		break;//����
		case 3:
				receiveID=0x103;		break;//����3
		case 4:
				receiveID=0x104;		break;//����4
		case 5:
				receiveID=0x105;		break;//����5		
		case 6:
				receiveID=0x106;		break;//����		
		//����
		case 101: 
				receiveID=0x165;		break;
		case 102:
				receiveID=0x166;		break;
		case 103:
				receiveID=0x167;		break;
		case 104:
				receiveID=0x168;		break;
		case 105:
				receiveID=0x169;		break;		
		case 106:
				receiveID=0x16A;		break;		
				
		default:              			break;				
	}

	//����ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 100;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 820;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, UDTrans_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//��̨ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 101;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 770;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Gimbal_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}

	//����ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 102;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 720;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//������ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 103;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 670;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Sub_Mode);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}

//	//����ģʽ1UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 104;		//ͼ������
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
//	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//�ַ�����
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x����
//	CliendStr.customStringData.grapic_data_struct.start_y = 780;				//y����
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
//	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply1);

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
//	//ID�Ѿ����Զ���ȡ����
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	memcpy(	
//			CliendGraphTxBuffer1 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
//	datalength1 = sizeof(CliendStr); 
//			
//	//����ģʽ2UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 105;				//ͼ������
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//����(���)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
//	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//�ַ�����
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
//	CliendStr.customStringData.grapic_data_struct.start_y = 730;						//y����
//	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
//	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply2);

//	//���ݷ���
//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
//	//ID�Ѿ����Զ���ȡ����
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	memcpy(	
//			CliendGraphTxBuffer2 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
//	datalength2 = sizeof(CliendStr); 
//	for(i = 0;i < datalength1;i++)
//	{
//		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
//		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
//	}
//	for(i = 0;i < datalength2;i++)
//	{
//		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
//		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
//	}

	//��Ԯģʽ1UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 106;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 680;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, help);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//����ģʽ2UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 107;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 630;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, snag);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	
	//��ȡģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 108;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 580;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, clamp);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//����ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 109;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 530;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, stretch);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}

	//��ԽǶ�	
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 110;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;					//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 450;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Re_angle);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 		
			
	//�����
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 111;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 480;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, ELE_Lock);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	
}



void Judge_Show1_Data(void)
{
	unsigned char CliendGraphTxBuffer1[send_max_len];
	unsigned char CliendGraphTxBuffer2[send_max_len];
	u8 datalength1, datalength2, i;
	//�߿�
	uint32_t wide = 2;
	//�����С
	int32_t	a = 12;
	//��ɫ
	uint32_t color = 4;
	//��̬UI�ַ�����
	uint32_t x1_1 = 440;
	uint32_t x2_2 = 1600;
	//�ַ�����
	uint32_t UDTrans_length;
	uint32_t Gimbal_length;
	uint32_t Chassis_length;
	uint32_t Chassis_sub_length;
//	uint32_t supply1_length;
//	uint32_t supply2_length;
	uint32_t help_length;
	uint32_t snag_length;
	uint32_t clamp_length;
	uint32_t stretch_length;
	uint32_t ELE_Lock_length;
//	uint32_t angle_length;
	//��̬��ʾ�ַ���
	uint8_t UDTrans_Mode[30];
	uint8_t Gimbal_Mode[30];
	uint8_t Chassis_Mode[30];
	uint8_t Chassis_Sub_Mode[30];
//	uint8_t	supply1[30];
//	uint8_t supply2[30];
	uint8_t help[30];
	uint8_t snag[30];
	uint8_t clamp[30];
	uint8_t stretch[30];
	uint8_t ELE_Lock[30];
//	uint8_t relative_angle[30];
	
	vTaskDelay(TIME_STAMP_1000MS);

	int sendID;      	
	int receiveID;		
	//�Զ��жϻ�����ID
	sendID=determine_robot_ID();
	switch(sendID)
	{
		//�췽
		case 1: 
				receiveID=0x101;		break;//Ӣ��
		case 2:
				receiveID=0x102;		break;//����
		case 3:
				receiveID=0x103;		break;//����3
		case 4:
				receiveID=0x104;		break;//����4
		case 5:
				receiveID=0x105;		break;//����5		
		case 6:
				receiveID=0x106;		break;//����		
		//����
		case 101: 
				receiveID=0x165;		break;
		case 102:
				receiveID=0x166;		break;
		case 103:
				receiveID=0x167;		break;
		case 104:
				receiveID=0x168;		break;
		case 105:
				receiveID=0x169;		break;		
		case 106:
				receiveID=0x16A;		break;		
				
		default:              			break;				
	}


/********************************************************��̬UI*************************************************************/

	//����ģʽ
	switch(UDTrans_Control.uptrans_behaviour)
	{
		case 0:
			strcpy2(UDTrans_Mode, "INIT");
			UDTrans_length = 4;
		break;

		case 1:
			strcpy2(UDTrans_Mode, "MANUAL");
			UDTrans_length = 6;
		break;

		case 2:
			strcpy2(UDTrans_Mode, "TINY");
			UDTrans_length = 4;
		break;

		case 3:
			strcpy2(UDTrans_Mode, "HUGE");
			UDTrans_length = 4;
		break;

		case 4:
			strcpy2(UDTrans_Mode, "CONVERSION");
			UDTrans_length = 10;
		break;

		case 5:
			strcpy2(UDTrans_Mode, "SNAG");
			UDTrans_length = 4;
		break;

		case 6:
			strcpy2(UDTrans_Mode, "SUPPLY");
			UDTrans_length = 6;
		break;
		
		case 7:
			strcpy2(UDTrans_Mode, "STOP");
			UDTrans_length = 4;
		break;

		case 8:
			strcpy2(UDTrans_Mode, "RECOVER");
			UDTrans_length = 7;
		break;

		case 9:
			strcpy2(UDTrans_Mode, "NOFORCE");
			UDTrans_length = 7;
		break;

		case 10:
			strcpy2(UDTrans_Mode, "HELP");
			UDTrans_length = 4;
		break;
		
		default:
		break;
	}

	//��̨ģʽ
	switch(gimbal_control.gimbal_motor_mode)
	{
		case 0:
			strcpy2(Gimbal_Mode, "Z_F");
			Gimbal_length = 10;
		break;

		case 1:
			strcpy2(Gimbal_Mode, "INIT");
			Gimbal_length = 4;
		break;

		case 2:
			strcpy2(Gimbal_Mode, "LOCK");
			Gimbal_length = 4;
			break;
			
		case 3:
			strcpy2(Gimbal_Mode, "A_A");
			Gimbal_length = 14;
		break;

		case 4:
			strcpy2(Gimbal_Mode, "R_A");
			Gimbal_length = 14;
		break;

		case 5:
			strcpy2(Gimbal_Mode, "MOTIONLESS");
			Gimbal_length = 10;
		break;

		case 6:
			strcpy2(Gimbal_Mode, "SPIN");
			Gimbal_length = 4;
		break;

		default:
		break;
	}
	//����ģʽ
	switch(chassis_move.chassis_mode)
	{
		case 0:
			strcpy2(Chassis_Mode, "Z_F");
			Chassis_length = 10;
		break;

		case 1:
			strcpy2(Chassis_Mode, "N_M");
			Chassis_length = 7;
		break;

		case 2:
			strcpy2(Chassis_Mode, "F_G_Y");
			Chassis_length = 17;
		break;

		case 3:
			strcpy2(Chassis_Mode, "SPIN");
			Chassis_length = 4;
		break;

		case 4:
			strcpy2(Chassis_Mode, "N_F_Y");
			Chassis_length = 13;
		break;

		default:
		break;
	}
	//������ģʽ
	switch(chassis_move.chassis_sub_mode)
	{
		case 0:
			strcpy2(Chassis_Sub_Mode, "NOM");
			Chassis_sub_length = 6;
		break;

		case 1:
			strcpy2(Chassis_Sub_Mode, "SLOW");
			Chassis_sub_length = 4;
		break;

		default:
		break;
	}
	//����1
//	if(supply1_flag == 1)
//	{
//		strcpy2(supply1, "ON");
//		supply1_length = 2;
//	}
//	else
//	{
//		strcpy2(supply1, "OFF");
//		supply1_length = 3;
//	}
//	//����2
//	if(supply2_flag == 1)
//	{
//		strcpy2(supply2, "ON");
//		supply2_length = 2;
//	}
//	else
//	{
//		strcpy2(supply2, "OFF");
//		supply2_length = 3;
//	}
	//��Ԯ
	if(help_flag == 1)
	{
		strcpy2(help, "ON");
		help_length = 2;
	}
	else
	{
		strcpy2(help, "OFF");
		help_length = 3;
	}
	//����
	if(snag_flag == 1)
	{
		strcpy2(snag, "ON");
		snag_length = 2;
	}
	else
	{
		strcpy2(snag, "OFF");
		snag_length = 3;
	}
	//��ȡ
	if(clamp_flag == 1)
	{
		strcpy2(clamp, "ON");
		clamp_length = 2;
	}
	else
	{
		strcpy2(clamp, "OFF");
		clamp_length = 3;
	}
	//����
	if(stretch_flag == 1)
	{
		strcpy2(stretch, "ON");
		stretch_length = 2;
	}
	else
	{
		strcpy2(stretch, "OFF");
		stretch_length = 3;
	}
	//�����
	if(lock_flag == 1)
	{
		strcpy2(ELE_Lock, "ON");
		ELE_Lock_length = 2;
	}
	else
	{
		strcpy2(ELE_Lock, "OFF");
		ELE_Lock_length = 3;
	}

	
	//����ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 100;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = UDTrans_length;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 820;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, UDTrans_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//��̨ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 101;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = Gimbal_length;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 770;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Gimbal_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}

	//����ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 102;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = Chassis_length;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 720;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//������ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 103;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = Chassis_sub_length;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 670;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Sub_Mode);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}

//	//����ģʽ1UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 104;		//ͼ������
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//����(���)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
//	CliendStr.customStringData.grapic_data_struct.end_angle = supply1_length;//�ַ�����
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x����
//	CliendStr.customStringData.grapic_data_struct.start_y = 780;				//y����
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
//	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply1);

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
//	//ID�Ѿ����Զ���ȡ����
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	memcpy(	
//			CliendGraphTxBuffer1 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
//	datalength1 = sizeof(CliendStr); 
//			
//	//����ģʽ2UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 105;				//ͼ������
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//����(���)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
//	CliendStr.customStringData.grapic_data_struct.end_angle = supply2_length;	//�ַ�����
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
//	CliendStr.customStringData.grapic_data_struct.start_y = 730;						//y����
//	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
//	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply2);

//	//���ݷ���
//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
//	//ID�Ѿ����Զ���ȡ����
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	memcpy(	
//			CliendGraphTxBuffer2 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
//	datalength2 = sizeof(CliendStr); 
//	for(i = 0;i < datalength1;i++)
//	{
//		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
//		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
//	}
//	for(i = 0;i < datalength2;i++)
//	{
//		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
//		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
//	}

	//��Ԯģʽ1UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 106;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = help_length;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 680;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, help);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//����ģʽ2UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 107;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = snag_length;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 630;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, snag);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	
	//��ȡģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 108;		//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = clamp_length;//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 580;				//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;				//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, clamp);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
			
	//����ģʽUI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 109;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = stretch_length;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 530;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, stretch);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	} 

	//�����
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 111;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = ELE_Lock_length;	//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 480;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, ELE_Lock);

	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength1 = sizeof(CliendStr); 
	//��ԽǶ�
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 110;				//ͼ������
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//ͼ������(�ַ�)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//����(���)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//�����С
	CliendStr.customStringData.grapic_data_struct.end_angle = 2;						//�ַ�����
	CliendStr.customStringData.grapic_data_struct.start_x = 1615 ;						//x����
	CliendStr.customStringData.grapic_data_struct.start_y = 450;						//y����
	CliendStr.customStringData.grapic_data_struct.width = wide;						//�߿�
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//ͼ��
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, UI_relative_angle);
	//���ݷ���
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //д��֡ͷCRC8У����

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110Ϊ�ַ���104�����߸�ͼ������
	//ID�Ѿ����Զ���ȡ����
	CliendStr.dataFrameHeader.sender_ID = sendID;								//�����ߵ�ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//д�����ݶ�CRC16У����	
	datalength2 = sizeof(CliendStr); 
			
	for(i = 0;i < datalength1;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer1[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	for(i = 0;i < datalength2;i++)
	{
		USART_SendData(USART3,(uint16_t)CliendGraphTxBuffer2[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	}
	
}




				
/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = Robot_status.robot_id;//��ȡ��ǰ������ID
	
	if(Robot_status.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}
}


/********************�������ݸ����жϺ���***************************/

///**
//  * @brief  �����Ƿ����
//  * @param  void
//  * @retval  TRUE����   FALSE������
//  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
//  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

///**
//  * @brief  ��ȡ˲ʱ����
//  * @param  void
//  * @retval ʵʱ����ֵ
//  * @attention  
//  */
float JUDGE_fGetChassisPower(void)
{
	return (Power_heat_data.chassis_power);
}



///**
//  * @brief  ��ȡʣ�ཹ������
//  * @param  void
//  * @retval ʣ�໺�役������(���60)
//  * @attention  
//  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (Power_heat_data.chassis_power_buffer);
}



/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	Robot_status.robot_level;
}


/**
  * @brief  ��ȡid1_17mmǹ������
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void)
{
	return Power_heat_data.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  ��ȡid2_17mmǹ������
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id2_17mm(void)
{
	return Power_heat_data.shooter_id2_17mm_cooling_heat;
}

/**
  * @brief  ��ȡid1_42mmǹ������
  * @param  void
  * @retval 42mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void)
{
	return Power_heat_data.shooter_id1_42mm_cooling_heat;
}


/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
float JUDGE_usGetSpeedHeat(void)
{
	return Shoot_data.bullet_speed;
}

/**
  * @brief  ͳ��17mm������
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time_17mm;//������ʱ����
portTickType shoot_ping_17mm;//����������շ����ӳ�

float Shoot_Speed_Now_17mm = 0;
float Shoot_Speed_Last_17mm = 0;

void JUDGE_ShootNumCount_17mm(void)
{
	//��⵽��С����  bullet_type==1
	if(Shoot_data.bullet_type == 1)
	{
	Shoot_Speed_Now_17mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_17mm != Shoot_Speed_Now_17mm)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum_17mm++;
		Shoot_Speed_Last_17mm = Shoot_Speed_Now_17mm;
	}
	shoot_time_17mm = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping_17mm = shoot_time_17mm - REVOL_uiGetRevolTime();//�����ӳ�
	}
}

///**
//  * @brief  ͳ��42mm������
//  * @param  void
//  * @retval void
//  * @attention  
//  */
portTickType shoot_time_42mm;//������ʱ����
portTickType shoot_ping_42mm;//����������շ����ӳ�

float Shoot_Speed_Now_42mm = 0;
float Shoot_Speed_Last_42mm = 0;

void JUDGE_ShootNumCount_42mm(void)
{
	//��⵽�Ǵ���  bullet_type==2
	if(Shoot_data.bullet_type == 2)
	{
	Shoot_Speed_Now_42mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_42mm != Shoot_Speed_Now_42mm)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum_42mm++;
		Shoot_Speed_Last_42mm = Shoot_Speed_Now_42mm;
	}
	shoot_time_42mm = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping_42mm = shoot_time_42mm - REVOL_uiGetRevolTime();//�����ӳ�
	}
}


///**
//  * @brief  ��ȡ17mm������
//  * @param  void
//  * @retval ������
//  * @attention ��������˫ǹ��
//  */
uint16_t JUDGE_usGetShootNum_17mm(void)
{
	return ShootNum_17mm;
}

///**
//  * @brief  ��ȡ42mm������
//  * @param  void
//  * @retval ������
//  * @attention ��������˫ǹ��
//  */
uint16_t JUDGE_usGetShootNum_42mm(void)
{
	return ShootNum_42mm;
}
///**
//  * @brief  17mm����������
//  * @param  void
//  * @retval void
//  * @attention 
//  */
void JUDGE_ShootNum_Clear_17mm(void)
{
	ShootNum_17mm = 0;
}
///**
//  * @brief  42mm����������
//  * @param  void
//  * @retval void
//  * @attention 
//  */
void JUDGE_ShootNum_Clear_42mm(void)
{
	ShootNum_42mm = 0;
}

/**
  * @brief  ��ȡid1_17mmǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id1_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_limit;
}

/**
  * @brief  ��ȡid2_17mmǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id2_17mm(void)
{
	return Robot_status.shooter_id2_17mm_cooling_limit;
}

/**
  * @brief  ��ȡid1_42mmǹ������
  * @param  void
  * @retval ��ǰ�ȼ�42mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id1_42mm(void)
{
	return Robot_status.shooter_id1_42mm_cooling_limit;
}
///**
//  * @brief  ��ȡ���̹������ֵ
//  * @param  void
//  * @retval ��ǰ�ȼ����̹�������
//  * @attention  
//  */
uint16_t JUDGE_usGetChassisPowerLimit(void)
{
	return Robot_status.chassis_power_limit;
}


///**
//  * @brief  ��ȡ�����˵ȼ�
//  * @param  void
//  * @retval 
//  * @attention  
//  */
uint8_t JUDGE_ucRobotLevel(void)
{
	return Robot_status.robot_level;
}


/**
  * @brief  ��ǰ�ȼ�id1_17mm��Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold_id1_17mm(void)
{
	return Robot_status. shooter_id1_17mm_cooling_rate;
}
/**
  * @brief  ��ǰ�ȼ�id2_17mm��Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold_id2_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_rate;
}

/**
  * @brief  ��ǰ�ȼ�id1_42mm��Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold_id1_42mm(void)
{
	return Robot_status. shooter_id1_42mm_cooling_rate;
}

/**
  * @brief  ����վ�� ID��
  * @param  void
  * @retval ��ǰ����վ�� ID��
  * @attention  Ϊ�ַ��ͱ���
	1�� 1 �Ų�����
	2�� 2 �Ų�����
  */
uint8_t JUDGE_usGetSupply_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}

/**
  * @brief  ���������� ID��
  * @param  void
  * @retval ��ǰ�貹�������� ID��
  * @attention  Ϊ�ַ��ͱ���
	���������� ID�� 0 Ϊ��ǰ�޻����˲����� 1 Ϊ�췽Ӣ�ۻ����˲����� 2 Ϊ�췽���̻�
	���˲����� 3/4/5 Ϊ�췽���������˲����� 101 Ϊ����Ӣ�ۻ����˲����� 102 Ϊ������
	�̻����˲����� 103/104/105 Ϊ�������������˲���
  */
uint8_t JUDGE_usGetSupply_Robo_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}


/**
  * @brief  �����ڿ���״̬ 
  * @param  void
  * @retval �����ڿ���״̬ 
  * @attention  Ϊ�ַ��ͱ���
	0 Ϊ�رգ�
	1 Ϊ�ӵ�׼���У�
	2 Ϊ�ӵ�����
  */
uint8_t JUDGE_usGetSupply_Mode(void)
{
	return Supply_projectile_action.supply_projectile_step ;
}


/**
  * @brief  ��ǰ��������
  * @param  void
  * @retval ��ǰ��������
  * @attention Ϊ�ַ��ͱ���
	50�� 50 ���ӵ���
	100�� 100 ���ӵ���
	150�� 150 ���ӵ���
	200�� 200 ���ӵ�
*/ 

uint8_t JUDGE_usGetSupply_Num(void)
{
	return Supply_projectile_action.supply_projectile_num;
}

/**
  * @brief  С��ͼ������ϢĿ������� ID(�״�վ)
  * @param  void
  * @retval ��ǰ��������
  * @attention 

*/ 
uint16_t JUDGE_usGetRadar_Station_to_robo_ID(void)
{
	return Client_map_command_t.target_robot_ID;
}

/**
  * @brief  С��ͼ������ϢĿ������� x λ�����꣬��λ m �� x,y ��������ʱ����ʾ
			(�״�վ)
  * @param  void
  * @retval ��ǰ��������
  * @attention 

*/ 
float JUDGE_usGetRadar_Station_to_robo_posX(void)
{
	return	Client_map_command_t.target_position_x;
}

/**
  * @brief  С��ͼ������ϢĿ������� y λ�����꣬��λ m �� x,y ��������ʱ����ʾ
			(�״�վ)
  * @param  void
  * @retval ��ǰ��������
  * @attention 

*/ 
float JUDGE_usGetRadar_Station_to_robo_posy(void)
{
	return	Client_map_command_t.target_position_y;
}


/****************�����Զ������ж���*******************/
/**
  * @brief  װ�װ��˺������Ƿ����
  * @param  void
  * @retval TRUE�Ѹ���   FALSEû����
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//Ĭ��װ�װ崦������״̬

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//װ�װ����ݸ���
	{
		Hurt_Data_Update = FALSE;//��֤���жϵ��´�װ�װ��˺�����
		ulDelay = ulCurrent + TIME_STAMP_200MS;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}

bool Judge_If_Death(void)
{
	if(Robot_status.remain_HP == 0 && JUDGE_sGetDataState() == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  װ�װ��ܴ��ID
  * @param  void
  * @retval   
  * @attention  
		bit 0-3�� ��Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ��������
		�����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0��

  */
uint8_t Judge_armor_id(void)
{
	return Robot_hurt.armor_id;
}

/**
  * @brief  װ�װ�����ģʽ
  * @param  void
  * @retval   
  * @attention  
		bit 4-7�� Ѫ���仯����
			0x0 װ���˺���Ѫ��
			0x1 ģ����߿�Ѫ��
			0x2 �����ٿ�Ѫ��
			0x3 ��ǹ��������Ѫ��
			0x4 �����̹��ʿ�Ѫ��
			0x5 װ��ײ����Ѫ
  */
uint8_t Judge_hurt_mode(void)
{
	return Robot_hurt.hurt_type;
}				

uint8_t determine_robot_ID(void)
{
	return Robot_status.robot_id;
}	
#endif
