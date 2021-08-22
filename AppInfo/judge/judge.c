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
/*****************系统数据定义**********************/
extGameRobotState_t       RobotState;		//0x0001
extRobotHurt_t            HurtData;			//0x0002
extShootData_t            ShootData;		//0x0003
extPowerHeatData_t        PowerHeatData;	//0x0004
extRfidDetect_t           RfidDetect;		//0x0005
extGameResult_t			  GameResultData;	//0x0006
extGetBuff_t			  GetBuffData;		//0x0007
extGameRobotPos_t		  GameRobotPosData;	//0x0008


xFrameHeader              FrameHeader;		//发送帧头信息
xShowData                 ShowData;
/****************************************************/

bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用

/**************裁判系统数据辅助****************/
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用


//当前等级对应的热量上限,18年版
#define HEAT_LEVEL1 120         //240
#define HEAT_LEVEL2 240         //360
#define HEAT_LEVEL3 480         //480

//当前等级对应的枪口冷却
#define COLD_LEVEL1 120         //40
#define COLD_LEVEL2 240         //60
#define COLD_LEVEL3 480         //80

portTickType shoot_time;//发射延时测试

portTickType shoot_ping;//计算出的最终发弹延迟

/*********************************裁判系统数据读取**************************************/

uint8_t JudgeUARTtemp;
//Judge
void USART3_IRQHandler(void)
{	
    JudgeUARTtemp = USART3->DR;
    JudgeUARTtemp = USART3->SR;
    
    DMA_Cmd(DMA1_Stream1, DISABLE);
			
	  Judge_Read_Data(Judge_Buffer);
		
    //重启DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream1, JUDGE_BUFFER_LEN);
    DMA_Cmd(DMA1_Stream1, ENABLE);
}



/**
  * @brief  读取裁判数据,loop中循环调用此函数来读取数据
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				switch(CmdID)
				{
					case ID_STATE:         //0x0001
						memcpy(&RobotState, (ReadFromUsart + DATA), LEN_STATE);
					break;
					
					case ID_HURT:          //0x0002
						memcpy(&HurtData, (ReadFromUsart + DATA), LEN_HURT);
						if(HurtData.hurtType == 0)//非装甲板离线造成伤害
						{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
						
					break;
					
					case ID_SHOOT:         //0x0003
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_SHOOT);
						JUDGE_ShootNumCount();//发弹量统计,不适用于双枪管,不准
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
				
				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}
	
	return retval_tf;//对数据正误做处理
}


/**************************用户自定义数据上传到客户端******************************/

/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void JUDGE_Show_Data(void)
{
	//其实大于22就好,分配多点空间也无所谓
	uint8_t Show_Pack[50] = {0};//长度大于头5+指令2+数据13+尾2就行,定义成u8是为了好计算字节数
	int i;//循环发送次数
	
	//帧头协议
	FrameHeader.SOF        = JUDGE_FRAME_HEADER;
	FrameHeader.Seq        = 0;		//为什么是0,官方好像没有说
	FrameHeader.DataLength = LEN_SHOW;
	
	//写入帧头
	memcpy( Show_Pack, &FrameHeader, LEN_HEADER );
	
	//帧头CRC8校验协议
	Append_CRC8_Check_Sum( Show_Pack, LEN_HEADER );
	
	//写入命令码
	ShowData.CmdID = ID_SHOW;
	
	//写入数据
	//ShowData.data1 = Capvoltage_Percent();//显示电容剩余电量      //(float)ShootNum;//第一个显示的是发弹量
	ShowData.data2 = 66.66;//Fric_GetHeatInc();//第二个显示当前射速(目标射速)
	ShowData.data3 = 88.88;
	ShowData.mask  = 0x00;//灯全关,低6位有效
	
//	if(VISION_IfAutoRed() == TRUE)
//	{
//		ShowData.mask &= 0x1f;//第6位置0
//	}
//	else if(VISION_IfAutoRed() == FALSE)
//	{
//		ShowData.mask |= 0x20;//第6位置1
//	}
//	
	
	memcpy( Show_Pack + LEN_HEADER, &ShowData, (LEN_CMDID + LEN_SHOW) );
	
	//帧尾CRC16校验协议
	Append_CRC16_Check_Sum( Show_Pack, (LEN_HEADER + LEN_CMDID + LEN_SHOW + LEN_TAIL) );
	
	//将打包好的数据通过串口移位发送到裁判系统
	for (i = 0; i < (LEN_HEADER + LEN_CMDID + LEN_SHOW + LEN_TAIL); i++)
	{
		//UART5_SendChar( Show_Pack[i] );
	}
}


/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassisPower);
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention  
  */
float JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassisPowerBuffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	RobotState.robotLevel;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooterHeat0;
}

/**
  * @brief  统计发弹量
  * @param  void
  * @retval void
  * @attention  
  */
void JUDGE_ShootNumCount(void)
{
	ShootNum++;
	shoot_time = xTaskGetTickCount();//获取弹丸发射时的系统时间
	shoot_ping = shoot_time - REVOL_uiGetRevolTime();//计算延迟
}

/**
  * @brief  读取发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
  */
uint16_t JUDGE_usGetShootNum(void)
{
	return ShootNum;
}

/**
  * @brief  发弹量清零
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear(void)
{
	ShootNum = 0;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 当前等级热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	if (RobotState.robotLevel == 1)//1级
	{
		return HEAT_LEVEL1;
	}
	else if (RobotState.robotLevel == 2)//2级
	{
		return HEAT_LEVEL2;
	}
	else if (RobotState.robotLevel == 3)//3级
	{
		return HEAT_LEVEL3;
	}
	else//防止读不到数据,强制限到最小
	{
		return HEAT_LEVEL1;
	}
}

/**
  * @brief  当前等级对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级冷却速度
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	if (RobotState.robotLevel == 1)//1级
	{
		return COLD_LEVEL1;
	}
	else if (RobotState.robotLevel == 2)//2级
	{
		return COLD_LEVEL2;
	}
	else if (RobotState.robotLevel == 3)//3级
	{
		return COLD_LEVEL3;
	}
	else//防止读不到数据,强制限到最小
	{
		return COLD_LEVEL1;
	}
}

/****************底盘自动闪避判断用*******************/
/**
  * @brief  装甲板伤害数据是否更新
  * @param  void
  * @retval TRUE已更新   FALSE没更新
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//默认装甲板处于离线状态

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//装甲板数据更新
	{
		Hurt_Data_Update = FALSE;//保证能判断到下次装甲板伤害更新
		ulDelay = ulCurrent + TIME_STAMP_200MS;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}



/************************超级电容预留用函数****************************/

/**
  * @brief  计算剩余功率
  * @param  void
  * @retval 剩余功率W
  * @attention  最大功率减实时功率,超级电容备用
  */
float JUDGE_fGetChassisResiduePower(void)
{
	return (80 - PowerHeatData.chassisPower);
}

/**
  * @brief  计算供给超级电容的电流值
  * @param  void
  * @retval 供给电流值
  * @attention  超级电容备用
  */
float JUDGE_fGetSuper_Cap_Ele(void)
{
	return ((80 - PowerHeatData.chassisPower) / PowerHeatData.chassisCurrent);
}


/*-------------------------2019--------------------------------*/
#elif JUDGE_VERSION == JUDGE_19

/*****************系统数据定义**********************/
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

xFrameHeader              FrameHeader;		//发送帧头信息
ext_SendClientData_t      ShowData;			//客户端信息
ext_CommunatianData_t     CommuData;		//队友通信信息
/****************************************************/

bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID


/**************裁判系统数据辅助****************/
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
#define BLUE  0
#define RED   1

/**
  * @brief  读取裁判数据,中断中读取保证速度
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
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
						if(RobotHurt.hurt_type == 0)//非装甲板离线造成伤害
						{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						JUDGE_ShootNumCount();//发弹量统
					break;
				}
				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
//				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
//				{
//					//如果一个数据包出现了多帧数据,则再次读取
//					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
//				}
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}
	
	return retval_tf;//对数据正误做处理
}

/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
void JUDGE_Show_Data(void)
{
	static u8 datalength,i;
	uint8_t judge_led = 0xff;//初始化led为全绿
	static uint8_t auto_led_time = 0;
	static uint8_t buff_led_time = 0;
	
	determine_ID();//判断发送者ID和其对应的客户端ID
	
	ShowData.txFrameHeader.SOF = 0xA5;
	ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
	ShowData.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	
	ShowData.CmdID = 0x0301;
	
	ShowData.dataFrameHeader.data_cmd_id = 0xD180;//发给客户端的cmd,官方固定
	//ID已经是自动读取的了
	ShowData.dataFrameHeader.send_ID 	 = Judge_Self_ID;//发送者的ID
	ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//客户端的ID，只能为发送者机器人对应的客户端
	
	/*- 自定义内容 -*/
	ShowData.clientData.data1 = (float)Capvoltage_Percent();//电容剩余电量
	ShowData.clientData.data2 = (float)Base_Angle_Measure();//吊射角度测
	ShowData.clientData.data3 = GIMBAL_PITCH_Judge_Angle();//云台抬头角度
	
	/***************扭腰指示****************/
	if(Chassis_IfCORGI() == TRUE)//扭屁股模式,第一颗灯亮红
	{
		judge_led &= 0xfe;//第1位置0,变红
	}
	else
	{
		judge_led |= 0x01;//第1位置1
	}
	
	/****************45°模式****************/
	if(Chassis_IfPISA() == TRUE)//45°模式，第二颗灯亮红
	{
		judge_led &= 0xfd;//第2位置0,变红
	}
	else
	{
		judge_led |= 0x02;//第2位置1
	}
	
	/***************自瞄指示****************/
	if(VISION_isColor() == ATTACK_RED)//自瞄红
	{
		judge_led &= 0xfb;//第3位置0,变红
	}
	else if(VISION_isColor() == ATTACK_BLUE)//自瞄蓝色
	{
		judge_led |= 0x04;//第3位置1
	}
	else//无自瞄
	{
		auto_led_time++;
		if(auto_led_time > 3)
		{
			(judge_led)^=(1<<2);//第3位取反,闪烁
			auto_led_time = 0;
		}
	}
	
	/****************打符指示***************/
	if(VISION_BuffType() == VISION_RBUFF_CLOCKWISE	//顺时针
			|| VISION_BuffType() == VISION_BBUFF_CLOCKWISE)
	{
		judge_led &= 0xf7;//第4位置0,变红
	}
	else if(VISION_BuffType() == VISION_RBUFF_ANTI	//逆时针
			|| VISION_BuffType() == VISION_BBUFF_ANTI)
	{
		judge_led |= 0x08;//第4位置1
	}
	else//不打符
	{
		buff_led_time++;
		if(buff_led_time > 3)
		{
			(judge_led)^=(1<<3);//第4位取反,闪烁
			buff_led_time = 0;
		}
	}
	/*--------------*/
	ShowData.clientData.masks = judge_led;//0~5位0红灯,1绿灯
	
	//打包写入数据段
	memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&ShowData.CmdID, 
			(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
		  );			
			
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ShowData));//写入数据段CRC16校验码	

	datalength = sizeof(ShowData); 
	for(i = 0;i < datalength;i++)
	{
		USART_SendData(UART5,(uint16_t)CliendTxBuffer[i]);
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
	}	 
}

/**
  * @brief  发送数据给队友
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
	
	Send_Color = is_red_or_blue();//判断发送给哨兵的颜色,17哨兵(蓝),7哨兵(红)；
	
	memset(TeammateTxBuffer,0,200);
	
	CommuData.txFrameHeader.SOF = 0xA5;
	CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
	CommuData.txFrameHeader.Seq = 0;
	memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
	Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));	
	
	CommuData.CmdID = 0x0301;
	
	   
	CommuData.dataFrameHeader.send_ID = Judge_Self_ID;//发送者的ID
	
	
	if( Senty_Run() == TRUE)
	{
		Senty_Run_Clean_Flag();
		First_Time_Send_Commu = TRUE;
	}
	
	if( First_Time_Send_Commu == TRUE )
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//在0x0200-0x02ff之间选择
		send_time++;
		if(send_time >= 20)
		{
			First_Time_Send_Commu = FALSE;
		}
		if(Send_Color == BLUE)//自己是蓝，发给蓝哨兵
		{
			CommuData.dataFrameHeader.receiver_ID = 17;//接收者ID
		}
		else if(Send_Color == RED)//自己是红，发给红哨兵
		{
			CommuData.dataFrameHeader.receiver_ID = 7;//接收者ID
		}
	}
	else
	{
		CommuData.dataFrameHeader.data_cmd_id = 0x0255;
		send_time = 0;
		CommuData.dataFrameHeader.receiver_ID = 88;//随便给个ID，不发送
	}
	
	CommuData.interactData.data[0] = 0;//发送的内容 //大小不要超过变量的变量类型   
	
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
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//读取当前机器人ID
	
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
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//计算客户端ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
	}
}

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_heat0;
}

/**
  * @brief  读取射速
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  统计发弹量
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time;//发射延时测试
portTickType shoot_ping;//计算出的最终发弹延迟
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	Shoot_Speed_Now = ShootData.bullet_speed;
	if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum++;
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//获取弹丸发射时的系统时间
	shoot_ping = shoot_time - REVOL_uiGetRevolTime();//计算延迟
}

/**
  * @brief  读取发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
  */
uint16_t JUDGE_usGetShootNum(void)
{
	return ShootNum;
}

/**
  * @brief  发弹量清零
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear(void)
{
	ShootNum = 0;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return GameRobotStat.shooter_heat0_cooling_limit;
}

/**
  * @brief  当前等级对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	return GameRobotStat.shooter_heat0_cooling_rate;
}

/****************底盘自动闪避判断用*******************/
/**
  * @brief  装甲板伤害数据是否更新
  * @param  void
  * @retval TRUE已更新   FALSE没更新
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//默认装甲板处于离线状态

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//装甲板数据更新
	{
		Hurt_Data_Update = FALSE;//保证能判断到下次装甲板伤害更新
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
/*****************系统数据定义**********************/
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


xFrameHeader              FrameHeader;		//发送帧头信息
ext_SendClientData_t      ShowData;			//客户端信息
ext_CommunatianData_t     CommuData;		//队友通信信息
ext_SendClientGraph_t     ClientGraph;  //图形通信
ext_SendClientGraphDelete_t ClientGraphDelete; //图形删除

extern Gimbal_Control_t gimbal_control;
extern UDTrans_Control_t UDTrans_Control;
extern chassis_move_t chassis_move;
extern uint8_t supply1_flag;    	//补给1标志位    0为关，1为开
extern uint8_t supply2_flag;	 	//补给2标志位    0为关，1为开
extern uint8_t help_flag;       	//救援标志位     0为关，1为开
extern uint8_t snag_flag;	      	//除障标志位     0为关，1为开
extern uint8_t clamp_flag;    		//夹取标志位     0为关，1为开
extern uint8_t stretch_flag;	  	//伸缩标志位     0为关，1为开
extern uint8_t lock_flag;
extern uint8_t UI_relative_angle[10];
/****************************************************/
bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

/**************裁判系统数据辅助****************/
uint16_t ShootNum_17mm;//统计发弹量,0x0003触发一次则认为发射了一颗
uint16_t ShootNum_42mm;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用

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
		
    //重启DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream1, JUDGE_BUFFER_LEN);
    DMA_Cmd(DMA1_Stream1, ENABLE);
}

/**
  * @brief  读取裁判数据,loop中循环调用此函数来读取数据
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析	

	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
			
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
					  if(Robot_hurt.hurt_type == 0)//非装甲板离线造成伤害
						{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
					break;
					
					case ID_shoot_data:      //0x0207
						memcpy(&Shoot_data, (ReadFromUsart + DATA), LEN_shoot_data);
					    //JUDGE_ShootNumCount_17mm();//发弹量统计,不适用于双枪管,不准
					    JUDGE_ShootNumCount_42mm();//发弹量统计,不适用于双枪管,不准
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
				
				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}					
		}		
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}

	return retval_tf;//对数据正误做处理	
	
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
 * @brief  上传自定义数据
 * @param  void
 * @retval void
 * @attention  数据打包,打包完成后通过串口发送到裁判系统
 */

ext_SendClientString_t	CliendStr;
ext_SendClientGraphDelete_t	DataChange;
uint16_t send_max_len = 200;
void Judge_Engineer_UI(void)
{
	unsigned char CliendGraphTxBuffer1[send_max_len];
	unsigned char CliendGraphTxBuffer2[send_max_len];
	u8 datalength1, datalength2, i;
	//线宽
	uint32_t wide = 2;
	int32_t	a = 12;
	uint32_t color = 4;
	//静态字符x坐标
	uint32_t x1 = 300;
	uint32_t x2 = 1470;
	//动态UI字符坐标
//	uint32_t x1_1 = 400;
//	uint32_t x2_2 = 1530;
	//静态字符串
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
	//自动判断机器人ID
	sendID=determine_robot_ID();
	switch(sendID)
	{
		//红方
		case 1: 
				receiveID=0x101;		break;//英雄
		case 2:
				receiveID=0x102;		break;//工程
		case 3:
				receiveID=0x103;		break;//步兵3
		case 4:
				receiveID=0x104;		break;//步兵4
		case 5:
				receiveID=0x105;		break;//步兵5		
		case 6:
				receiveID=0x106;		break;//空中		
		//蓝方
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


/************************************************静态UI*******************************************************/
	//工程
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 0;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 9;				//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 820;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;	//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str1);	
	
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
	
	//云台
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 1;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 12;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 770;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str2);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength2 = sizeof(CliendStr); 
	//发送
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

	//底盘
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 2;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 12;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 720;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str3);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 

	//底盘子
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 3;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 17;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 670;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str4);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength2 = sizeof(CliendStr); 
	//数据发送
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

//	//补给1
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 4;		//图形名称
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
//	CliendStr.customStringData.grapic_data_struct.end_angle = 7;			//字符长度
//	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
//	CliendStr.customStringData.grapic_data_struct.start_y = 780;				//y坐标
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
//	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, str5);

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
//	//ID已经是自动读取的了
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
//	memcpy(	
//			CliendGraphTxBuffer1 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
//	datalength1 = sizeof(CliendStr); 

//	//补给2
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 5;		//图形名称
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
//	CliendStr.customStringData.grapic_data_struct.end_angle = 7;			//字符长度
//	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
//	CliendStr.customStringData.grapic_data_struct.start_y = 730;				//y坐标
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
//	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, str6);	

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
//	//ID已经是自动读取的了
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
//	memcpy(	
//			CliendGraphTxBuffer2 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
//	datalength2 = sizeof(CliendStr); 
//	
//	//数据发送
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

	//救援
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 6;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 680;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str7);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 

	//除障
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 7;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 630;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str8);	

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength2 = sizeof(CliendStr); 
	//数据发送
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

	//夹取
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 8;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 5;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 580;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str9);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 

	//伸缩
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 9;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 7;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 530;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str10);	

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码
    CliendStr.CmdID = 0x0301;
	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);						
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength2 = sizeof(CliendStr); 
	
	//数据发送
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

	//相对角度
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 10;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 17;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 450;			//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str11);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
	//电磁锁
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 11;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 17;			//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 480;			//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 0;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, str12);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength2 = sizeof(CliendStr); 
	//数据发送
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
	//线宽
	uint32_t wide = 2;
	int32_t	a = 12;
	//颜色
	uint32_t color = 4;
	//动态UI字符坐标
	uint32_t x1_1 = 450;//400
	uint32_t x2_2 = 1580;//1530

	//动态显示字符串
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
	//自动判断机器人ID
	sendID=determine_robot_ID();
	switch(sendID)
	{
		//红方
		case 1: 
				receiveID=0x101;		break;//英雄
		case 2:
				receiveID=0x102;		break;//工程
		case 3:
				receiveID=0x103;		break;//步兵3
		case 4:
				receiveID=0x104;		break;//步兵4
		case 5:
				receiveID=0x105;		break;//步兵5		
		case 6:
				receiveID=0x106;		break;//空中		
		//蓝方
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

	//工程模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 100;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 820;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, UDTrans_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//云台模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 101;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 770;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Gimbal_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

	//底盘模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 102;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 720;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//底盘子模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 103;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 670;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Sub_Mode);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

//	//补给模式1UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 104;		//图形名称
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
//	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//字符长度
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x坐标
//	CliendStr.customStringData.grapic_data_struct.start_y = 780;				//y坐标
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
//	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply1);

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
//	//ID已经是自动读取的了
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
//	memcpy(	
//			CliendGraphTxBuffer1 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
//	datalength1 = sizeof(CliendStr); 
//			
//	//补给模式2UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 105;				//图形名称
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//操作(添加)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
//	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//字符长度
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
//	CliendStr.customStringData.grapic_data_struct.start_y = 730;						//y坐标
//	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
//	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply2);

//	//数据发送
//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
//	//ID已经是自动读取的了
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
//	memcpy(	
//			CliendGraphTxBuffer2 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

	//救援模式1UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 106;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 680;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, help);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//除障模式2UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 107;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 630;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, snag);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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
	
	//夹取模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 108;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 580;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, clamp);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//伸缩模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 109;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 530;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, stretch);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

	//相对角度	
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 110;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;					//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 450;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Re_angle);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 		
			
	//电磁锁
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 111;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 1;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 4;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 480;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, ELE_Lock);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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
	//线宽
	uint32_t wide = 2;
	//字体大小
	int32_t	a = 12;
	//颜色
	uint32_t color = 4;
	//动态UI字符坐标
	uint32_t x1_1 = 440;
	uint32_t x2_2 = 1600;
	//字符长度
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
	//动态显示字符串
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
	//自动判断机器人ID
	sendID=determine_robot_ID();
	switch(sendID)
	{
		//红方
		case 1: 
				receiveID=0x101;		break;//英雄
		case 2:
				receiveID=0x102;		break;//工程
		case 3:
				receiveID=0x103;		break;//步兵3
		case 4:
				receiveID=0x104;		break;//步兵4
		case 5:
				receiveID=0x105;		break;//步兵5		
		case 6:
				receiveID=0x106;		break;//空中		
		//蓝方
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


/********************************************************动态UI*************************************************************/

	//工程模式
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

	//云台模式
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
	//底盘模式
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
	//底盘子模式
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
	//补给1
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
//	//补给2
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
	//救援
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
	//除障
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
	//夹取
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
	//伸缩
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
	//电磁锁
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

	
	//工程模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 100;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = UDTrans_length;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 820;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
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
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//云台模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 101;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = Gimbal_length;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 770;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Gimbal_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

	//底盘模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 102;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = Chassis_length;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 720;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Mode);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//底盘子模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 103;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = Chassis_sub_length;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x1_1;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 670;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, Chassis_Sub_Mode);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

//	//补给模式1UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 104;		//图形名称
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//操作(添加)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
//	CliendStr.customStringData.grapic_data_struct.end_angle = supply1_length;//字符长度
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x坐标
//	CliendStr.customStringData.grapic_data_struct.start_y = 780;				//y坐标
//	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
//	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply1);

//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
//	//ID已经是自动读取的了
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
//	memcpy(	
//			CliendGraphTxBuffer1 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
//	datalength1 = sizeof(CliendStr); 
//			
//	//补给模式2UI
//	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 105;				//图形名称
//	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
//	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//操作(添加)
//	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
//	CliendStr.customStringData.grapic_data_struct.end_angle = supply2_length;	//字符长度
//	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
//	CliendStr.customStringData.grapic_data_struct.start_y = 730;						//y坐标
//	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
//	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
//	CliendStr.customStringData.grapic_data_struct.color = color;
//	for(i=0; i<30; i++)
//	{
//		CliendStr.customStringData.data[i] = '\0';
//	}
//	strcpy1(CliendStr.customStringData.data, supply2);

//	//数据发送
//	CliendStr.txFrameHeader.SOF = 0xA5;
//	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
//	CliendStr.txFrameHeader.Seq = 0;
//	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
//	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

//    CliendStr.CmdID = 0x0301;

//	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
//	//ID已经是自动读取的了
//	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
//	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
//	memcpy(	
//			CliendGraphTxBuffer2 + 5, 
//			(uint8_t*)&CliendStr.CmdID, 
//			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
//		);		
//				
//	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

	//救援模式1UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 106;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = help_length;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 680;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, help);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//除障模式2UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 107;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = snag_length;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 630;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, snag);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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
	
	//夹取模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 108;		//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;			//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;			//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;			//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = clamp_length;//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;				//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 580;				//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;				//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;					//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, clamp);

	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
			
	//伸缩模式UI
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 109;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = stretch_length;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 530;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, stretch);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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

	//电磁锁
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 111;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = ELE_Lock_length;	//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = x2_2;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 480;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, ELE_Lock);

	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer1, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer1, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer1 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer1,sizeof(CliendStr));				//写入数据段CRC16校验码	
	datalength1 = sizeof(CliendStr); 
	//相对角度
	CliendStr.customStringData.grapic_data_struct.graphic_name[0] = 110;				//图形名称
	CliendStr.customStringData.grapic_data_struct.graphic_tpye = 7;					//图形类型(字符)
	CliendStr.customStringData.grapic_data_struct.operate_tpye = 2;					//操作(添加)
	CliendStr.customStringData.grapic_data_struct.start_angle = a;					//字体大小
	CliendStr.customStringData.grapic_data_struct.end_angle = 2;						//字符长度
	CliendStr.customStringData.grapic_data_struct.start_x = 1615 ;						//x坐标
	CliendStr.customStringData.grapic_data_struct.start_y = 450;						//y坐标
	CliendStr.customStringData.grapic_data_struct.width = wide;						//线宽
	CliendStr.customStringData.grapic_data_struct.layer = 1;							//图层
	CliendStr.customStringData.grapic_data_struct.color = color;
	for(i=0; i<30; i++)
	{
		CliendStr.customStringData.data[i] = '\0';
	}
	strcpy1(CliendStr.customStringData.data, UI_relative_angle);
	//数据发送
	CliendStr.txFrameHeader.SOF = 0xA5;
	CliendStr.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	CliendStr.txFrameHeader.Seq = 0;
	memcpy(CliendGraphTxBuffer2, &CliendStr.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	Append_CRC8_Check_Sum(CliendGraphTxBuffer2, sizeof(xFrameHeader));			  //写入帧头CRC8校验码

    CliendStr.CmdID = 0x0301;

	CliendStr.dataFrameHeader.data_cmd_id = 0x110;								//110为字符，104发送七个图形数据
	//ID已经是自动读取的了
	CliendStr.dataFrameHeader.sender_ID = sendID;								//发送者的ID
	CliendStr.dataFrameHeader.receiver_ID = receiveID;							//客户端的ID，只能为发送者机器人对应的客户端
	memcpy(	
			CliendGraphTxBuffer2 + 5, 
			(uint8_t*)&CliendStr.CmdID, 
			(sizeof(CliendStr.CmdID) + sizeof(CliendStr.dataFrameHeader) + sizeof(CliendStr.customStringData))
		);		
				
	Append_CRC16_Check_Sum(CliendGraphTxBuffer2,sizeof(CliendStr));				//写入数据段CRC16校验码	
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
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = Robot_status.robot_id;//读取当前机器人ID
	
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
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//计算客户端ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
	}
}


/********************裁判数据辅助判断函数***************************/

///**
//  * @brief  数据是否可用
//  * @param  void
//  * @retval  TRUE可用   FALSE不可用
//  * @attention  在裁判读取函数中实时改变返回值
//  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

///**
//  * @brief  读取瞬时功率
//  * @param  void
//  * @retval 实时功率值
//  * @attention  
//  */
float JUDGE_fGetChassisPower(void)
{
	return (Power_heat_data.chassis_power);
}



///**
//  * @brief  读取剩余焦耳能量
//  * @param  void
//  * @retval 剩余缓冲焦耳能量(最大60)
//  * @attention  
//  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (Power_heat_data.chassis_power_buffer);
}



/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	Robot_status.robot_level;
}


/**
  * @brief  读取id1_17mm枪口热量
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void)
{
	return Power_heat_data.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  读取id2_17mm枪口热量
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id2_17mm(void)
{
	return Power_heat_data.shooter_id2_17mm_cooling_heat;
}

/**
  * @brief  读取id1_42mm枪口热量
  * @param  void
  * @retval 42mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void)
{
	return Power_heat_data.shooter_id1_42mm_cooling_heat;
}


/**
  * @brief  读取射速
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
float JUDGE_usGetSpeedHeat(void)
{
	return Shoot_data.bullet_speed;
}

/**
  * @brief  统计17mm发弹量
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time_17mm;//发射延时测试
portTickType shoot_ping_17mm;//计算出的最终发弹延迟

float Shoot_Speed_Now_17mm = 0;
float Shoot_Speed_Last_17mm = 0;

void JUDGE_ShootNumCount_17mm(void)
{
	//检测到是小弹丸  bullet_type==1
	if(Shoot_data.bullet_type == 1)
	{
	Shoot_Speed_Now_17mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_17mm != Shoot_Speed_Now_17mm)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum_17mm++;
		Shoot_Speed_Last_17mm = Shoot_Speed_Now_17mm;
	}
	shoot_time_17mm = xTaskGetTickCount();//获取弹丸发射时的系统时间
	shoot_ping_17mm = shoot_time_17mm - REVOL_uiGetRevolTime();//计算延迟
	}
}

///**
//  * @brief  统计42mm发弹量
//  * @param  void
//  * @retval void
//  * @attention  
//  */
portTickType shoot_time_42mm;//发射延时测试
portTickType shoot_ping_42mm;//计算出的最终发弹延迟

float Shoot_Speed_Now_42mm = 0;
float Shoot_Speed_Last_42mm = 0;

void JUDGE_ShootNumCount_42mm(void)
{
	//检测到是大弹丸  bullet_type==2
	if(Shoot_data.bullet_type == 2)
	{
	Shoot_Speed_Now_42mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_42mm != Shoot_Speed_Now_42mm)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum_42mm++;
		Shoot_Speed_Last_42mm = Shoot_Speed_Now_42mm;
	}
	shoot_time_42mm = xTaskGetTickCount();//获取弹丸发射时的系统时间
	shoot_ping_42mm = shoot_time_42mm - REVOL_uiGetRevolTime();//计算延迟
	}
}


///**
//  * @brief  读取17mm发弹量
//  * @param  void
//  * @retval 发弹量
//  * @attention 不适用于双枪管
//  */
uint16_t JUDGE_usGetShootNum_17mm(void)
{
	return ShootNum_17mm;
}

///**
//  * @brief  读取42mm发弹量
//  * @param  void
//  * @retval 发弹量
//  * @attention 不适用于双枪管
//  */
uint16_t JUDGE_usGetShootNum_42mm(void)
{
	return ShootNum_42mm;
}
///**
//  * @brief  17mm发弹量清零
//  * @param  void
//  * @retval void
//  * @attention 
//  */
void JUDGE_ShootNum_Clear_17mm(void)
{
	ShootNum_17mm = 0;
}
///**
//  * @brief  42mm发弹量清零
//  * @param  void
//  * @retval void
//  * @attention 
//  */
void JUDGE_ShootNum_Clear_42mm(void)
{
	ShootNum_42mm = 0;
}

/**
  * @brief  读取id1_17mm枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id1_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_limit;
}

/**
  * @brief  读取id2_17mm枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id2_17mm(void)
{
	return Robot_status.shooter_id2_17mm_cooling_limit;
}

/**
  * @brief  读取id1_42mm枪口热量
  * @param  void
  * @retval 当前等级42mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id1_42mm(void)
{
	return Robot_status.shooter_id1_42mm_cooling_limit;
}
///**
//  * @brief  读取底盘功率最大值
//  * @param  void
//  * @retval 当前等级底盘功率上限
//  * @attention  
//  */
uint16_t JUDGE_usGetChassisPowerLimit(void)
{
	return Robot_status.chassis_power_limit;
}


///**
//  * @brief  读取机器人等级
//  * @param  void
//  * @retval 
//  * @attention  
//  */
uint8_t JUDGE_ucRobotLevel(void)
{
	return Robot_status.robot_level;
}


/**
  * @brief  当前等级id1_17mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t JUDGE_usGetShootCold_id1_17mm(void)
{
	return Robot_status. shooter_id1_17mm_cooling_rate;
}
/**
  * @brief  当前等级id2_17mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t JUDGE_usGetShootCold_id2_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_rate;
}

/**
  * @brief  当前等级id1_42mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t JUDGE_usGetShootCold_id1_42mm(void)
{
	return Robot_status. shooter_id1_42mm_cooling_rate;
}

/**
  * @brief  补给站口 ID：
  * @param  void
  * @retval 当前补给站口 ID：
  * @attention  为字符型变量
	1： 1 号补给口
	2： 2 号补给口
  */
uint8_t JUDGE_usGetSupply_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}

/**
  * @brief  补给机器人 ID：
  * @param  void
  * @retval 当前需补给机器人 ID：
  * @attention  为字符型变量
	补弹机器人 ID： 0 为当前无机器人补弹， 1 为红方英雄机器人补弹， 2 为红方工程机
	器人补弹， 3/4/5 为红方步兵机器人补弹， 101 为蓝方英雄机器人补弹， 102 为蓝方工
	程机器人补弹， 103/104/105 为蓝方步兵机器人补弹
  */
uint8_t JUDGE_usGetSupply_Robo_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}


/**
  * @brief  出弹口开闭状态 
  * @param  void
  * @retval 出弹口开闭状态 
  * @attention  为字符型变量
	0 为关闭，
	1 为子弹准备中，
	2 为子弹下落
  */
uint8_t JUDGE_usGetSupply_Mode(void)
{
	return Supply_projectile_action.supply_projectile_step ;
}


/**
  * @brief  当前补弹数量
  * @param  void
  * @retval 当前补弹数量
  * @attention 为字符型变量
	50： 50 颗子弹；
	100： 100 颗子弹；
	150： 150 颗子弹；
	200： 200 颗子弹
*/ 

uint8_t JUDGE_usGetSupply_Num(void)
{
	return Supply_projectile_action.supply_projectile_num;
}

/**
  * @brief  小地图接收信息目标机器人 ID(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
uint16_t JUDGE_usGetRadar_Station_to_robo_ID(void)
{
	return Client_map_command_t.target_robot_ID;
}

/**
  * @brief  小地图接收信息目标机器人 x 位置坐标，单位 m 当 x,y 超出界限时则不显示
			(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
float JUDGE_usGetRadar_Station_to_robo_posX(void)
{
	return	Client_map_command_t.target_position_x;
}

/**
  * @brief  小地图接收信息目标机器人 y 位置坐标，单位 m 当 x,y 超出界限时则不显示
			(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
float JUDGE_usGetRadar_Station_to_robo_posy(void)
{
	return	Client_map_command_t.target_position_y;
}


/****************底盘自动闪避判断用*******************/
/**
  * @brief  装甲板伤害数据是否更新
  * @param  void
  * @retval TRUE已更新   FALSE没更新
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//默认装甲板处于离线状态

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//装甲板数据更新
	{
		Hurt_Data_Update = FALSE;//保证能判断到下次装甲板伤害更新
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
  * @brief  装甲板受打击ID
  * @param  void
  * @retval   
  * @attention  
		bit 0-3： 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人
		的五个装甲片，其他血量变化类型，该变量数值为 0。

  */
uint8_t Judge_armor_id(void)
{
	return Robot_hurt.armor_id;
}

/**
  * @brief  装甲板受伤模式
  * @param  void
  * @retval   
  * @attention  
		bit 4-7： 血量变化类型
			0x0 装甲伤害扣血；
			0x1 模块掉线扣血；
			0x2 超射速扣血；
			0x3 超枪口热量扣血；
			0x4 超底盘功率扣血；
			0x5 装甲撞击扣血
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
