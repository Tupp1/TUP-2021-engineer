#ifndef _JUDGE_H
#define _JUDGE_H


#include "system.h"

//裁判系统版本预编译
#define		JUDGE_18		18
#define		JUDGE_19		19
#define		JUDGE_21		21

#define		JUDGE_VERSION	JUDGE_21

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1
/*-------------------------2018--------------------------------*/
#if JUDGE_VERSION == JUDGE_18

//2018裁判系统官方接口协议


//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16


/* RFID卡类型 */
#define    CARD_ATTACK        ((uint8_t)0x00)
#define    CARD_PROTECT       ((uint8_t)0x01)
#define    CARD_BLOOD_RED     ((uint8_t)0x02)
#define    CARD_BLOOD_BLUE    ((uint8_t)0x03)
#define    CARD_HEAL_RED      ((uint8_t)0x04)
#define    CARD_HEAL_BLUE     ((uint8_t)0x05)
#define    CARD_COLD_RED      ((uint8_t)0x06)
#define    CARD_COLD_BLUE     ((uint8_t)0x07)
#define    CARD_FORT          ((uint8_t)0x08)



typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;


//frame header格式

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)


//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;



/***************命令码ID********************/

/* 

   ID: 0x0001  Byte:  8    比赛机器人状态       发送频率 10Hz      
   ID: 0x0002  Byte:  1    伤害数据             受到伤害时发送      
   ID: 0x0003  Byte:  6    实时射击数据         发射弹丸时发送  
   ID: 0x0004  Byte: 20    实时功率和热量数据   发送频率 50Hz   20MS发送一次
   ID: 0x0005  Byte:  2    实时场地交互数据     检测到RFID卡时 发送频率 10Hz 
   ID: 0X0006  Byte:  1    比赛胜负数据         比赛结束时发送一次 
   ID: 0X0007  Byte:  2    Buff 获取数据        激活机关后发送一次
   ID: 0X0008  Byte: 16    机器人位置朝向信息   发送频率 50Hz        
   ID: 0x0100  Byte: 13    自定义数据           限制频率 10Hz
*/


//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_STATE       = 0x0001,
	ID_HURT 	   = 0x0002,
	ID_SHOOT       = 0x0003,
	ID_POWER_HEAT  = 0x0004,
	ID_RFID        = 0x0005,
	ID_GAME_RESULT = 0x0006,
	ID_BUFF_GET    = 0x0007,
	ID_POSITION    = 0x0008,
	ID_SHOW        = 0x0100,

} CmdID;


//命令码数据段长,根据官方协议来定义长度
typedef enum
{
	LEN_STATE       =  8,	//0x0001
	LEN_HURT        =  1,	//0x0002
	LEN_SHOOT       =  6,	//0x0003
	LEN_POWER_HEAT  = 20,	//0x0004
	LEN_RFID        =  2,	//0x0005
	LEN_GAME_RESULT =  1,	//0x0006
	LEN_BUFF_GET    =  2,	//0x0007
	LEN_POSITION    = 16,	//0x0008
	LEN_SHOW        = 13,	//0x0100
	
} JudgeDataLength;



/* ID: 0x0001  Byte:  8 */
typedef __packed struct
{
	uint16_t stageRemainTime;
	uint8_t  gameProcess;
	uint8_t  robotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	
} extGameRobotState_t;


/* ID: 0x0002  Byte:  1 */
typedef __packed struct
{
	uint8_t armorType :4;
	uint8_t hurtType  :4;

} extRobotHurt_t;	


/* ID: 0x0003  Byte:  6 */
typedef __packed struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float   bulletSpeed;
	
} extShootData_t;


/* ID: 0x0004  Byte: 20 */
typedef __packed struct
{
	float chassisVolt;
	float chassisCurrent;
	float chassisPower;//瞬时功率
	float chassisPowerBuffer;//60焦耳缓冲能量
	uint16_t shooterHeat0;//17mm
	uint16_t shooterHeat1;//42mm
	
} extPowerHeatData_t;


/* ID: 0x0005  Byte:  2 */
typedef __packed struct
{
	uint8_t cardType;
	uint8_t cardIdx;
	
} extRfidDetect_t;


/* ID: 0X0006  Byte:  1 */
typedef __packed struct
{
	uint8_t winner;
	
} extGameResult_t;


/* ID: 0X0007  Byte:  2 */
typedef __packed struct
{
	uint8_t buffType;
	uint8_t buffAddition;
	
} extGetBuff_t;


/* ID: 0X0008  Byte: 16 */
typedef __packed struct
{
	float  x;
	float  y;
	float  z;
	float  yaw;
	
} extGameRobotPos_t;


/* ID: 0x0100  Byte: 13 */
typedef __packed struct
{
	uint16_t CmdID;//命令码 0x0100
	float data1;//浮点显示具体数值
	float data2;
	float data3;
	uint8_t mask;//低6位对应6个指示灯
	
} xShowData;


typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;


/**********裁判系统数据读取*************/
bool Judge_Read_Data(uint8_t *ReadFormUsart);//主循环4ms调用一次

/****用户自定义数据上传到客户端****/
void JUDGE_Show_Data(void);

/*****裁判数据辅助判断函数********/
bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
float JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat17(void);

void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);

uint16_t JUDGE_usGetHeatLimit(void);
uint16_t JUDGE_usGetShootCold(void);

/*******底盘自动闪避判断用******/
bool JUDGE_IfArmorHurt(void);

/*****超级电容预留用函数******/
float JUDGE_fGetChassisResiduePower(void);
float JUDGE_fGetSuper_Cap_Ele(void);



extern extPowerHeatData_t  PowerHeatData;	//0x0004

/*-------------------------2019--------------------------------*/
#elif JUDGE_VERSION == JUDGE_19

#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16


//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  2    比赛机器人存活数据   		1Hz发送  
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/


//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       			= 0x0001,//比赛状态数据
	ID_game_result 	   				= 0x0002,//比赛结果数据
	ID_game_robot_survivors       	= 0x0003,//比赛机器人存活数据
	ID_event_data  					= 0x0101,//场地事件数据 
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_supply_projectile_booking 	= 0x0103,//场地补给站预约子弹数据
	ID_game_robot_state    			= 0x0201,//机器人状态数据
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据
	ID_buff_musk					= 0x0204,//机器人增益数据
	ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					= 0x0206,//伤害状态数据
	ID_shoot_data					= 0x0207,//实时射击数据

} CmdID;


//命令码数据段长,根据官方协议来定义长度
typedef enum
{
	LEN_game_state       				=  3,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_survivors       		=  2,	//0x0003
	LEN_event_data  					=  4,	//0x0101
	LEN_supply_projectile_action        =  3,	//0x0102
	LEN_supply_projectile_booking		=  2,	//0x0103
	LEN_game_robot_state    			= 15,	//0x0201
	LEN_power_heat_data   				= 14,	//0x0202
	LEN_game_robot_pos        			= 16,	//0x0203
	LEN_buff_musk        				=  1,	//0x0204
	LEN_aerial_robot_energy        		=  3,	//0x0205
	LEN_robot_hurt        				=  1,	//0x0206
	LEN_shoot_data       				=  6,	//0x0207
	
} JudgeDataLength;

/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    比赛机器人存活数据 */
typedef __packed struct 
{ 
	uint16_t robot_legion;
} ext_game_robot_survivors_t; 


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  2    场地补给站预约子弹数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/* ID: 0X0201  Byte: 15    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;   //机器人ID，可用来校验发送
	uint8_t robot_level;  //1一级，2二级，3三级
	uint16_t remain_HP;  //机器人剩余血量
	uint16_t max_HP; //机器人满血量
	uint16_t shooter_heat0_cooling_rate;  //机器人 17mm 子弹热量冷却速度 单位 /s
	uint16_t shooter_heat0_cooling_limit;   // 机器人 17mm 子弹热量上限
	uint16_t shooter_heat1_cooling_rate;   
	uint16_t shooter_heat1_cooling_limit;   
	uint8_t mains_power_gimbal_output : 1;  
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   
	uint16_t chassis_current;    
	float chassis_power;   //瞬时功率 
	uint16_t chassis_power_buffer;//60焦耳缓冲能量
	uint16_t shooter_heat0;//17mm
	uint16_t shooter_heat1;  
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;   
	float bullet_speed;  
} ext_shoot_data_t; 


/* 
	
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。 
*/
/* 交互数据接收信息：0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz


	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //数据段,n需要小于113
} robot_interactive_data_t;

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	client_custom_data_t  					clientData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;


//机器人交互信息
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;

bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
void determine_ID(void);

bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat17(void);
float JUDGE_usGetSpeedHeat17(void);
void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint16_t JUDGE_usGetShootCold(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);


/*-------------------------2021--------------------------------*/
#elif JUDGE_VERSION == JUDGE_21
//2021裁判系统官方接口协议
//通信方式是串口，配置为波特率 115200， 8 位数据位， 1 位停止位，无硬件流控，无校验位


//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
//#define    LEN_DATA      32        //数据长度
#define    LEN_TAIL      2	      //帧尾CRC16

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)
//通讯串口
#define		USART		USART3

//通信协议偏移位置
typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;


//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
}FrameHeaderOffset;


typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;


/***************命令码ID********************/

/* 
   ID: 0x0001  Byte:  11    比赛状态数据，                  1Hz 周期发送      
   ID: 0x0002  Byte:  1     比赛结果数据，                  比赛结束后发送      
   ID: 0x0003  Byte:  28    比赛机器人血量数据，            1Hz 周期发送  
   ID: 0x0004  Byte:  3     飞镖发射状态，                  飞镖发射后发送
   ID: 0x0005  Byte:  11    人工智能挑战赛加成与惩罚区状态   1Hz 周期发送 
   ID: 0X0101  Byte:  4    场地事件数据，                  1Hz 周期发送 
   ID: 0X0102  Byte:  3    场地补给站动作标识数据，        动作发生后发送
//  ID: 0x0103  Byte:  2    	请求补给站补弹数据	          由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）   
   ID: 0X0104  Byte:  2    裁判警告数据，                  警告发生后发送        
   ID: 0x0105  Byte:  1    飞镖发射口倒计时，              1Hz 周期发送
   ID: 0x0201  Byte:  27 15   机器人状态数据，                10Hz 周期发送
   ***官方写的15但实际上是27***
   ID: 0x0202  Byte:  14   实时功率热量数据，              50Hz 周期发送
   ID: 0x0203  Byte:  16   机器人位置数据，                10Hz 发送
   ID: 0x0204  Byte:  1    机器人增益数据，                1Hz 周期发送
   ID: 0x0205  Byte:  3    空中机器人能量状态数据，        10Hz 周期发送，只有空中机器人主控发送
   ID: 0x0206  Byte:  1    伤害状态数据，                  伤害发生后发送
   ID: 0x0207  Byte:  6    实时射击数据，                  子弹发射后发送
	 ID: 0x0208  Byte:  2    弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送  1Hz 周期发送
	 ID: 0x0209  Byte:  4    机器人 RFID 状态，              1Hz 周期发送
	 ID: 0x020A  Byte:  12   飞镖机器人客户端指令数据，      10Hz 周期发送
	 ID: 0x0301  Byte:  n    机器人间交互数据，              发送方触发发送，
	 ID: 0x0302  Byte:  n    自定义控制器交互数据接口,      通过客户端触发发送，上限 30Hz
	 ID: 0x0303  Byte:  15   客户端小地图交互数据,  		   触发发送
	 ID: 0x0304  Byte:  12	 键盘、鼠标信息,					通过图传串口发送
	 ID: 0X0305  Byte:  10	 客户端小地图接收信息		最大接收频率： 10Hz 雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到。	

*/


//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_status                 = 0x0001,//比赛状态数据,1Hz周期发送
	ID_game_result 	               = 0x0002,//比赛结果数据，比赛结束后发送
	ID_robot_HP                    = 0x0003,//比赛机器人血量数据,1Hz周期发送
	ID_dart_status                 = 0x0004,//飞镖发射状态，飞镖发射时发送
	ID_ICRA_buff_debuff_zone_status= 0x0005,
	ID_event_data                  = 0x0101,//场地事件数据,1Hz 周期发送 
	ID_supply_projectile_action    = 0x0102,//场地补给站动作标识数据,动作发生后发送
	ID_referee_warning             = 0x0104,//裁判警告数据，警告发生后发送 
	ID_dart_remaining_time         = 0x0105,//飞镖发射口倒计时，1Hz周期发送
    ID_robot_status                = 0x0201,//机器人状态数据,10Hz周期发送 
	ID_heat_data                   = 0x0202,//实时功率热量数据,50Hz周期发送 
	ID_robot_pos                   = 0x0203,//机器人位置数据,10Hz发送
	ID_buff                        = 0x0204,//机器人增益数据,1Hz 周期发送 
	ID_aerial_robot_energy         = 0x0205,//空中机器人能量状态数据,10Hz周期发送,只有空中机器人主控发送
	ID_robot_hurt                  = 0x0206,//伤害状态数据,，伤害发生后发送
	ID_shoot_data                  = 0x0207,//实时射击数据,子弹发射后发送 
	ID_bullet_remaining            = 0x0208,//弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送，1Hz 周期发送
	ID_rfid_status                 = 0x0209,//机器人 RFID 状态，1Hz 周期发送 
	ID_dart_client_cmd             = 0x020A,//飞镖机器人客户端指令数据，10Hz 周期发送 
	ID_robot_interaction_data      = 0x0301, //机器人间交互数据，发送方触发发送
	ID_customize_controller_interaction_data_interface     = 0x0302, //自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
    ID_minimap_interactive_data    = 0x0303, //客户端小地图交互数据，触发发送
	ID_keybord_and_mouse_massage   = 0x0304, //键盘、鼠标信息，通过图传串口发送
	ID_client_map_command     	   = 0x0305, //雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到

} CmdID;

/*未写
	  0x301
	  0x302
*/
//命令码数据段长,根据官方协议来定义长度
typedef enum
{
	LEN_game_status                  =  11,	//0x0001
	LEN_game_result                  =  1,	//0x0002
	LEN_robot_HP                     =  28,	//0x0003
	LEN_dart_status                  =  3,	//0x0004
	LEN_ICRA_buff_debuff_zone_status =  11, //0x0005
	LEN_event_data                   =  4,	//0x0101
	LEN_supply_projectile_action     =  3,	//0x0102
	LEN_referee_warning              =  2,	//0x0104
	LEN_dart_remaining_time          =  1,	//0x0105
	LEN_robot_status                 = 27,	//0x0201
	LEN_heat_data                    = 14,	//0x0202
	LEN_robot_pos                    = 16,	//0x0203
	LEN_buff                         =  1,	//0x0204
	LEN_aerial_robot_energy          =  3,	//0x0205    
	LEN_robot_hurt                   =  1,	//0x0206  
	LEN_shoot_data                   =  6,	//0x0207  
	LEN_bullet_remaining             =  2,	//0x0208  
	LEN_rfid_status                  =  4,	//0x0209 
	LEN_dart_client_cmd              = 12,	//0x020A 
	LEN_minimap_interactive_data 	 = 15,   //0x0303
	LEN_keybord_and_mouse_massage 	 = 12,   //0x0304
	LEN_client_map_command    		 = 10,  //0x0305
	
} JudgeDataLength;



//比赛状态数据： 0x0001      发送范围：所有机器人。
typedef __packed struct
{
//语法：在自定义的变量后面加上：数字 表明该变量所占的位数
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
	
uint64_t SyncTimeStamp;	
} ext_game_status_t;
// 0-3 bit： 比赛类型
// 1： RoboMaster 机甲大师赛；
// 2： RoboMaster 机甲大师单项赛；
// 3： ICRA RoboMaster 人工智能挑战赛
// 4： RoboMaster 联盟赛 3V3
// 5： RoboMaster 联盟赛 1V1
//4-7 bit： 当前比赛阶段
// 0： 未开始比赛；
// 1： 准备阶段；
// 2： 自检阶段；
// 3： 5s 倒计时；
//4： 对战中；
// 5： 比赛结算中


//比赛结果数据： 0x0002     发送范围：所有机器人。
typedef __packed struct
{
uint8_t winner;
} ext_game_result_t;

//机器人血量数据 0x0003     发送范围：所有机器人。
typedef __packed struct
{
uint16_t red_1_robot_HP;
uint16_t red_2_robot_HP;
uint16_t red_3_robot_HP;
uint16_t red_4_robot_HP;
uint16_t red_5_robot_HP;
uint16_t red_7_robot_HP;
uint16_t red_outpost_HP;
uint16_t red_base_HP;
uint16_t blue_1_robot_HP;
uint16_t blue_2_robot_HP;
uint16_t blue_3_robot_HP;
uint16_t blue_4_robot_HP;
uint16_t blue_5_robot_HP;
uint16_t blue_7_robot_HP;
uint16_t blue_outpost_HP;
uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//飞镖发射状态： 0x0004          发送范围：所有机器人。
typedef __packed struct
{
uint8_t dart_belong;
uint16_t stage_remaining_time;
} ext_dart_status_t;


//人工智能挑战赛加成与惩罚区状态： 0x0005   发送范围：所有机器人。
typedef __packed struct
{
uint8_t F1_zone_status:1;
uint8_t F1_zone_buff_debuff_status:3;
uint8_t F2_zone_status:1;
uint8_t F2_zone_buff_debuff_status:3;
uint8_t F3_zone_status:1;
uint8_t F3_zone_buff_debuff_status:3;
uint8_t F4_zone_status:1;
uint8_t F4_zone_buff_debuff_status:3;
uint8_t F5_zone_status:1;
uint8_t F5_zone_buff_debuff_status:3;
uint8_t F6_zone_status:1;
uint8_t F6_zone_buff_debuff_status:3;
uint16_t red1_bullet_left;
uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

//场地事件数据： 0x0101      发送范围： 己方机器人
typedef __packed struct
{
uint32_t event_type;
} ext_event_data_t;

//补给站动作标识： 0x0102                发送范围：己方机器人
typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//裁判警告信息： cmd_id (0x0104)          发送范围：己方机器人
typedef __packed struct
{
uint8_t level;
uint8_t foul_robot_id;
} ext_referee_warning_t;

//飞镖发射口倒计时： cmd_id (0x0105)      发送范围：己方机器
typedef __packed struct
{
uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//比赛机器人状态： 0x0201                  发送范围：单一机器人
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_id1_17mm_cooling_rate;
uint16_t shooter_id1_17mm_cooling_limit;
uint16_t shooter_id1_17mm_speed_limit;
uint16_t shooter_id2_17mm_cooling_rate;
uint16_t shooter_id2_17mm_cooling_limit;
uint16_t shooter_id2_17mm_speed_limit;
uint16_t shooter_id1_42mm_cooling_rate;
uint16_t shooter_id1_42mm_cooling_limit;
uint16_t shooter_id1_42mm_speed_limit;
uint16_t chassis_power_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

//实时功率热量数据： 0x0202                发送范围：单一机器人
typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_id1_17mm_cooling_heat;
uint16_t shooter_id2_17mm_cooling_heat;
uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

//机器人位置： 0x0203                     发送范围：单一机器人。
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

//机器人增益： 0x0204                      发送范围：单一机器人。
typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_t;

//空中机器人能量状态： 0x0205              发送范围：单一机器人
typedef __packed struct
{
uint8_t attack_time;
} aerial_robot_energy_t;

//伤害状态： 0x0206。                      发送范围：单一机器人。
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//实时射击信息： 0x0207                    发送范围：单一机器人。
typedef __packed struct
{
uint8_t bullet_type;
uint8_t shooter_id;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;

//子弹剩余发射数： 0x0208                  发送范围：单一机器人。空中机器人，哨兵机器人以及 ICRA 机器人,主控发送
typedef __packed struct
{
uint16_t bullet_remaining_num_17mm;
uint16_t bullet_remaining_num_42mm;
uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


//机器人 RFID 状态： 0x0209                发送范围：单一机器人
typedef __packed struct
{
uint32_t rfid_status;
} ext_rfid_status_t;

//飞镖机器人客户端指令数据： 0x020A        发送范围：单一机器人
typedef __packed struct
{
uint8_t dart_launch_opening_status;
uint8_t dart_attack_target;
uint16_t target_change_time;
uint8_t first_dart_speed;
uint8_t second_dart_speed;
uint8_t third_dart_speed;
uint8_t fourth_dart_speed;
uint16_t last_dart_launch_time;
uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;



/*机器人间交互数据*/

/* 
	
	交互数据包括一个统一的数据段头结构。
  数据段包含了内容ID，发送者以及接收者的ID和内容数据段，
  整个交互数据的包总共长最大为 128 个字节，
  减去 frame_header,cmd_id 和 frame_tail 共 9 个字节
  以及数据段头结构的 6 个字节，故而发送的内容数据段最大为 113
	机器人 ID：                                                                                                                                                                                                                                                                                                                                             
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
  9，雷达站（红）
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)：
  109，雷达站（蓝）	
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，操作手客户端步兵(蓝)；
	0x016A，空中操作手客户端(蓝)。 
*/


/* 交互数据接收信息：0x0301  */
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t sender_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;



///* 
//	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180F
//	发送频率：上限 10Hz


/*	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		发送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/

typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;



/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			      备注 
	0 		     	2 		数据的内容 ID 	0x0200~0x02FF 可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 		    	2 		发送者的 ID 	  需要校验发送者的 ID 正确性， 
	
	4 			    2 		接收者的 ID 	  需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID 
	
	6 			    n 		数据段 		     	n 需要小于 113 

*/
typedef __packed struct
{
 uint8_t data[50];
} robot_interactive_data_t;


//客户端删除图形 机器人间通信： 0x0301
typedef __packed struct
{
uint8_t operate_tpye;
uint8_t layer;
} ext_client_custom_graphic_delete_t;

//图形数据
typedef __packed struct
{
uint8_t graphic_name[3]; 		 //图形名
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
fp32	Float;
} graphic_data_struct_t;



//客户端绘制一个图形 机器人间通信：0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;


// 客户端绘制二个图形 机器人间通信： 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;


//客户端绘制五个图形 机器人间通信： 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;


 //客户端绘制字符 机器人间通信： 0x0301
 typedef __packed struct
 {
 graphic_data_struct_t grapic_data_struct;
 uint8_t data[30];
 }ext_client_custom_character_t;


//客户端绘制七个图形 机器人间通信： 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;


/*
	自定义控制器数据包括一个统一的数据段头结构。数据段为内容数据段，整个交互数据的包总共长最大为
	39 个字节，减去 frame_header,cmd_id 和 frame_tail 共 9 个字节，故而发送的内容数据段最大为 30 字
	节。整个交互数据 0x0302 的包下行行频率为 30Hz。
*/

////交互数据接收信息： 0x0302。 发送频率：上限 30Hz
//typedef __packed struct
//{
//uint8_t data[];
//} robot_interactive_data_t;


//小地图交互信息标识： 0x0303。 发送频率：触发时发送。
//客户端下发信息
//机器人 ID： 机器人 ID： 1，英雄(红)； 2，工程(红)； 3/4/5，步兵(红)； 6，空中(红)； 7，哨兵(红)； 9，雷
//达站（红）； 10，前哨站（红）； 11，基地（红）； 101，英雄(蓝)； 102，工程(蓝)； 103/104/105，步兵
//(蓝)； 106，空中(蓝)； 107，哨兵(蓝)； 109，雷达站（蓝）； 110，前哨站（蓝）； 111，基地（蓝） 。

typedef __packed struct
{
float target_position_x;
float target_position_y;
float target_position_z;
uint8_t commd_keyboard;
uint16_t target_robot_ID;
} ext_robot_command_t;


//小地图接收信息标识： 0x0305。 最大接收频率： 10Hz。
//雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到

typedef __packed struct
{ 
uint16_t target_robot_ID;
float target_position_x;
float target_position_y;
} ext_client_map_command_t;


//图传遥控信息标识： 0x0304。发送频率： 30Hz。
//客户端下发信息
//字节偏移量      大小          		说明
//	  0			   2			鼠标 X 轴信息
//	  2 		   2			鼠标 Y 轴信息
//	  4			   2			鼠标滚轮信息
//	  6			   1			鼠标左键
//	  7			   1     		鼠标右键按下
//	  8			   2			键盘信息
//bit 0：键盘 W 是否按下
//bit 1：键盘 S 是否按下
//bit 2：键盘 A 是否按下
//bit 3：键盘 D 是否按下
//bit 4：键盘 SHIFT 是否按下
//bit 5：键盘 CTRL 是否按下
//bit 6：键盘 Q 是否按下
//bit 7：键盘 E 是否按下
//bit 8：键盘 R 是否按下
//bit 9：键盘 F 是否按下
//bit 10：键盘 G 是否按下
//bit 11：键盘 Z 是否按下
//bit 12：键盘 X 是否按下
//bit 13：键盘 C 是否按下
//bit 14：键盘 V 是否按下
//bit 15：键盘 B 是否按下

//自己更改过结构体名称	原   ext_robot_command_t;
typedef __packed struct
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_robot_command_t_change;

/*
	以下
*/


//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//帧头(5个字节)
	uint16_t		 				 	                 	 CmdID;//命令码(2个字节)
	ext_student_interactive_header_data_t    dataFrameHeader;//数据段头结构(6个字节)
	client_custom_data_t  					         clientData;//数据段(14个字节)
	uint16_t		 						                 FrameTail;//帧尾(2个字节)
}ext_SendClientData_t;

//机器人交互信息
typedef __packed struct
{
	xFrameHeader   					txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;

//客户端第一个图形添加
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//帧头(5个字节)
	uint16_t		 				 	                 	 CmdID;//命令码(2个字节)
	ext_student_interactive_header_data_t    dataFrameHeader;//数据段头结构(6个字节)
	ext_client_custom_graphic_single_t  		 customGraphicData;//数据段(15个字节)
	uint16_t		 						                 FrameTail;//帧尾(2个字节)
}ext_SendClientGraph_t;


//客户端字符添加
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//帧头(5个字节)
	uint16_t		 				 	                 	 CmdID;//命令码(2个字节)
	ext_student_interactive_header_data_t    dataFrameHeader;//数据段头结构(6个字节)
	ext_client_custom_character_t  		 	 customStringData;//数据段(15个字节)
	uint16_t		 						                 FrameTail;//帧尾(2个字节)
}ext_SendClientString_t;

//客户端删除图形
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//帧头(5个字节)
	uint16_t		 				 	                 	 CmdID;//命令码(2个字节)
	ext_student_interactive_header_data_t    dataFrameHeader;//数据段头结构(6个字节)
	ext_client_custom_graphic_delete_t  		 customGraphicDelete;//数据段(2个字节)
	uint16_t		 						                 FrameTail;//帧尾(2个字节)
}ext_SendClientGraphDelete_t;

typedef __packed struct
{
	uint16_t CmdID;//命令码 0x0100
	float data1;//浮点显示具体数值
	float data2;
	float data3;
	uint8_t mask;//低6位对应6个指示灯	
} xShowData;

/*
        以下
*/

//这是什么2018年头文件有但不知道是干什么的,这个似乎是自定义的
extern ext_power_heat_data_t  Power_heat_data;	//0x0202


/**********裁判系统数据读取*************/
bool Judge_Read_Data(uint8_t *ReadFormUsart);//主循环4ms调用一次
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
void determine_ID(void);


/*****裁判数据辅助判断函数********/
bool JUDGE_sGetDataState(void);    //判断是否接受到裁判系统
float JUDGE_fGetChassisPower(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
//各枪口热量读取枪口热量
uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void);
uint16_t JUDGE_usGetRemoteHeat_id2_17mm(void);
uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void);
//读取射速
float JUDGE_usGetSpeedHeat(void);
//统计发弹量
//ATTENTION！！！！！！！！！！！！！双枪管可能会出错！！！！！！！！！！！！！！！
void JUDGE_ShootNumCount_17mm(void);
void JUDGE_ShootNumCount_42mm(void);
//读取发弹量
//ATTENTION！！！！！！！！！！！！！双枪管可能会出错！！！！！！！！！！！！！！！
uint16_t JUDGE_usGetShootNum_17mm(void);
uint16_t JUDGE_usGetShootNum_42mm(void);
//发弹量清零
//ATTENTION！！！！！！！！！！！！！双枪管可能会出错！！！！！！！！！！！！！！！
void JUDGE_ShootNum_Clear_17mm(void);
void JUDGE_ShootNum_Clear_42mm(void);
//各枪口热量读取枪口热量上限
uint16_t JUDGE_usGetHeatLimit_id1_17mm(void);
uint16_t JUDGE_usGetHeatLimit_id2_17mm(void);
uint16_t JUDGE_usGetHeatLimit_id1_42mm(void);
//各枪口热量读取枪口热量每秒冷却值
uint16_t JUDGE_usGetShootCold_id1_17mm(void);
uint16_t JUDGE_usGetShootCold_id2_17mm(void);
uint16_t JUDGE_usGetShootCold_id1_42mm(void);
//补给站口
uint8_t JUDGE_usGetSupply_Id(void);
//补给机器人 ID
uint8_t JUDGE_usGetSupply_Robo_Id(void);
//出弹口开闭状态 
uint8_t JUDGE_usGetSupply_Mode(void);
//返回当前补弹数量（为字符型变量）
uint8_t JUDGE_usGetSupply_Num(void);
//小地图接收信息目标机器人 ID(雷达站)
uint16_t JUDGE_usGetRadar_Station_to_robo_ID(void);
//小地图接收信息目标机器人 x 位置坐标
float JUDGE_usGetRadar_Station_to_robo_posX(void);
//小地图接收信息目标机器人 y 位置坐标
float JUDGE_usGetRadar_Station_to_robo_posy(void);



bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);

//底盘功率函数
uint16_t JUDGE_usGetChassisPowerLimit(void);//获取当前等级底盘功率上限
uint8_t JUDGE_ucRobotLevel(void);//获取机器人等级数据

uint8_t Judge_armor_id(void);//装甲板受打击ID
uint8_t Judge_hurt_mode(void);//伤害类型
void Judge_Engineer_UI(void);
void Judge_Show1_Data(void);
void Judge_Show2_Data(void);
void UI_Init(void);
//自动判断ID
uint8_t determine_robot_ID(void);


#endif //版本号
/*-------------------------------------------------------------*/

#endif //头文件
