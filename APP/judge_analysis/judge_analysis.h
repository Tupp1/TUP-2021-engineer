#ifndef JUDGE_ANALYSIS_H
#define JUDGE_ANALYSIS_H


#include "stm32f4xx.h"
/************************ 2017年的裁判系统协议结构体 Start ***********************/
//#define JudgeBufferLength       150
//#define JudgeFrameLength_1      44
//#define JudgeFrameLength_2      11
//#define JudgeFrameLength_3      24

//#define JudgeFrameHeader        0xA5        //帧头 



#ifdef  __DRIVER_GLOBALS
#define __DRIVER_EXT
#else
#define __DRIVER_EXT extern
#endif


//小符状态枚举
typedef enum
{
    BUFF_TYPE_NONE, //无效
    BUFF_TYPE_ARMOR = 0x01, //防御符
    BUFF_TYPE_SUPPLY = 0x04, //加血符
    BUFF_TYPE_BULLFTS= 0x08, //加弹符
}LBuffType_Enum;


//位置状态结构体
typedef __packed struct
{
    uint8_t flag; //0 无效， 1 有效
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t compass;
}GpsData_Struct;


//比赛进程信息结构体
typedef __packed struct
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    uint8_t runeStatus[4];
    uint8_t bigRune0Status;
    uint8_t bigRune1status;
    uint8_t conveyorBelts0:2;
    uint8_t conveyorBelts1:2;
    uint8_t parkingApron0:1;
    uint8_t parkingApron1:1;
    uint8_t parkingApron2:1;
    uint8_t parkingApron3:1;
    GpsData_Struct gpsData;
}GameInfo_Struct;


//实时血量变化信息结构体
typedef __packed struct
{
    uint8_t weakId:4;
    uint8_t way:4;
    uint16_t value;
}RealBloodChangedData_Struct;


//实时射击信息结构体
typedef __packed struct
{
    float realBulletShootSpeed;
    float realBulletShootFreq;
    float realGolfShootSpeed;
    float realGolfShootFreq;
}RealShootData_Struct;


//裁判系统结构体
typedef struct
{
    float RealVoltage;                  //实时电压
    float RealCurrent;                  //实时电流
    int16_t LastBlood;                  //剩余血量
    uint8_t LastHartID;                 //上次收到伤害的装甲板ID号
    //portTickType LastHartTick;          //上次受伤害时间 
    float LastShotSpeed;                //上次射击速度
    //·portTickType LastShotTick;          //上次射击时间
#if INFANTRY == 7
    uint16_t ShootNum;                  //已发射子弹数
    uint8_t BulletUseUp;                //1 基地子弹射完          0 基地子弹未射完
    uint16_t ShootFail;                 //发射失败时间 
#endif
}InfantryJudge_Struct;

//裁判系统数据缓存
//__DRIVER_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];
//实时电压
//__DRIVER_EXT InfantryJudge_Struct InfantryJudge;
////帧率计数器
//__DRIVER_EXT float JudgeFrameCounter;
////帧率
//__DRIVER_EXT float JudgeFrameRate;

/************************ 2017年的裁判系统协议结构体 End ***********************/









/************************ 2018年的裁判系统协议结构体 Start ***********************/
#define JudgeBufferLength_SH       255

#define JudgeBufferLength_PJ        29 //0X0004
#define JudgeBufferLength_Position  25 //0X0008
#define JudgeFrameLength_PJPosi_SH      54 //之所以是54是因为大疆将04 与 08两段数据安排到一帧里发送，但是校验却是其中一个包的校验
#define JudgeFrameLength_1_SH      54 


#define JudgeFrameLength_2_SH      11
#define JudgeFrameLength_3_SH      24

//比赛机器人状态 (0x0001)
typedef __packed struct
{
	uint16_t stageRemianTime;
	uint8_t gameProgress;
	uint8_t robotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
}extGameRobotState_t;


//实施射击信息 (0x0003)
typedef __packed struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float bulletSpeed;
	int16_t bulletcount;
	uint8_t bulletMAXspeed;
}extShootData_t;


//实时功率热量数据 (0x0004)
typedef __packed struct
{
	float chassisVolt;
	float chassisCurrent;
	float chassisPower;
	float chassisPowerBuffer;
	uint16_t shooterHeat0;
	uint16_t shooterHeat1;
	uint16_t shooterHeat0MAX;
	uint16_t shooterHeat1MAX;
	uint16_t shooterHeatCooling0;
	uint16_t shooterHeatCooling1;
}extPowerHeatData_t;

//机器人位置朝向信息 (0x0008)
typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
}extGameRobotPos_t;


//裁判系统数据缓存
//__DRIVER_EXT uint8_t JudgeDataBuffer_SH[JudgeBufferLength_SH];

//__DRIVER_EXT extGameRobotState_t extGameRobotState;

//__DRIVER_EXT extGameRobotPos_t extGameRobotPos;

//__DRIVER_EXT extPowerHeatData_t extPowerHeatData;

//__DRIVER_EXT extShootData_t extShootData;


/************************ 2018年的裁判系统协议结构体 End ***********************/





///************************ 2019年的裁判系统协议结构体 Start ***********************/
typedef enum
{
   Robot_Status_ID                   =0x0201,//机器人状态  等级
   power_heat_data_ID   	           =0x0202,//枪口热量 底盘功率
	 robot_hurt_ID                     =0x0206,//伤害类型
   shoot_data_ID	      					   =0x0207,//射频射速
   student_interactive_header_ID     =0x0301,
} Judege_Cmd_ID;
//帧头


typedef __packed struct
{
  uint8_t   SOF;//0xA5
  uint16_t  DataLenth;//数据位长度
	uint8_t   Seq;//包序号
  uint8_t   CRC8;//crc8位校验
}tFrameHeader_Judge;



/*伤害类型*/
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;
/*比赛机器人状态*/
typedef __packed struct
{
	uint8_t robot_id;//机器人ID
	uint8_t robot_level;//机器人等级
	uint16_t remain_HP;//机器人剩余HP
	uint16_t max_HP;//机器人上限血量
	uint16_t shooter_heat0_cooling_rate;//机器人17mm枪口每秒冷却值
	uint16_t shooter_heat0_cooling_limit;//枪口热量上限
	uint16_t shooter_heat1_cooling_rate;//42mm枪口每秒冷却值
	uint16_t shooter_heat1_cooling_limit;//枪口热量上限
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;


/*底盘功率及枪口热量*/
typedef __packed struct
{
	uint16_t chassis_volt;//底盘输出电压  单位 毫伏
	uint16_t chassis_current;//底盘输出电流  单位毫安
	float chassis_power;//底盘输出功率  单位 W
	uint16_t chassis_power_buffer;//底盘功率缓冲  单位J
	uint16_t shooter_heat0;//17mm枪口热量
	uint16_t shooter_heat1;//42mm枪口热量
} ext_power_heat_data_t;



/*子弹信息*/
typedef __packed struct
{
	uint8_t bullet_type;//子弹类型
	uint8_t bullet_freq;//子弹射频
	float bullet_speed;//子弹射速
} ext_shoot_data_t;

/*用户自定义部分显示在屏幕上*/
/*0-1 数据内容ID 0xD180
	2-3 发送者的ID
*/


typedef __packed struct
{
  uint8_t   SOF;//0xA5
  uint16_t  DataLenth;
	uint8_t   Seq;
  uint8_t   CRC8;
}tFrameHeader;

typedef __packed struct
{
	uint16_t rxCmdId;
	uint16_t data_id;
	uint16_t send_id;
	uint16_t receive_id;
} id_data_t;


typedef __packed struct
{
	tFrameHeader Header;
	id_data_t    id;
	float data1;
	float data2;
	float data3;
	uint8_t masks;
	uint16_t crc_16;
} client_custom_data_t;


//机器人之间相互通信
typedef __packed struct
{
	uint8_t data[112];
} robot_interactive_data_t;


typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
}ext_student_interactive_header_data_t;


typedef  struct
{
  tFrameHeader          frameHeader;
  uint16_t              rxCmdId;
  ext_power_heat_data_t power_heat_data_t;
  ext_shoot_data_t		  shoot_data_t;
	ext_robot_hurt_t      robot_hurt_t;
	ext_game_robot_state_t game_robot_state_t;
//  robot_interactive_data_t robot_data_t;
	client_custom_data_t robot_data_t;
	client_custom_data_t userinfo;
}JudgementDataTypedef;

/************************ 2019年的裁判系统协议结构体 End ***********************/

#define JudgeFrameHeader        0xA5        //帧头
#define FrameHeader_Len         5 


typedef enum 
{
	GameInfo = 0x0001,     
	RealBloodChangedData,  
	RealShootData,            
	SelfDefinedData =0x0100, 
	Wrong = 0x1301
}tCmdID; 

typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}tSelfDefineInfo;  

typedef  __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}SelfDefineInfo_t;   //学生上传自定义数据 (0x0005)  

 


typedef __packed struct
{
	uint8_t SOF;               //数据起始字节，固定为0xA5          
	uint16_t DataLength;       //数据长度
	uint8_t Seq;               //包序号
	uint8_t CRC8;              //帧头CRC校验
}FrameHeader_t;//帧头



#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes
#define HEADER_LEN   sizeof(FrameHeader_t)  //frame length



typedef  struct
{
  tFrameHeader_Judge          frameHeader;
  uint16_t              rxCmdId;
		__packed union 
	{
  ext_power_heat_data_t power_heat_data_t;
  ext_shoot_data_t		  shoot_data_t;
	ext_robot_hurt_t      robot_hurt_t;
	ext_game_robot_state_t game_robot_state_t;
  robot_interactive_data_t robot_data_t;
	client_custom_data_t userinfo;
		}Data;
	uint16_t        CRC16;
	uint16_t        CRC16_2 ;
}FRAME;



typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tSelfDefineInfo SelfDefineInfo; 
	uint16_t        CRC16;         
}tFrame;  


__DRIVER_EXT uint8_t JudgeDataBuffer_SH[JudgeBufferLength_SH];
__DRIVER_EXT SelfDefineInfo_t SelfDefineInfo;
__DRIVER_EXT JudgementDataTypedef JudgementData;

void Send_FrameData(uint16_t cmdid, uint8_t * pchMessage,uint16_t dwLength);
void client_send(uint8_t * data);
void client_init(uint8_t * data1,uint8_t * data2);
#endif
