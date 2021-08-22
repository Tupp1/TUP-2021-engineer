#ifndef JUDGE_ANALYSIS_H
#define JUDGE_ANALYSIS_H


#include "stm32f4xx.h"
/************************ 2017��Ĳ���ϵͳЭ��ṹ�� Start ***********************/
//#define JudgeBufferLength       150
//#define JudgeFrameLength_1      44
//#define JudgeFrameLength_2      11
//#define JudgeFrameLength_3      24

//#define JudgeFrameHeader        0xA5        //֡ͷ 



#ifdef  __DRIVER_GLOBALS
#define __DRIVER_EXT
#else
#define __DRIVER_EXT extern
#endif


//С��״̬ö��
typedef enum
{
    BUFF_TYPE_NONE, //��Ч
    BUFF_TYPE_ARMOR = 0x01, //������
    BUFF_TYPE_SUPPLY = 0x04, //��Ѫ��
    BUFF_TYPE_BULLFTS= 0x08, //�ӵ���
}LBuffType_Enum;


//λ��״̬�ṹ��
typedef __packed struct
{
    uint8_t flag; //0 ��Ч�� 1 ��Ч
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t compass;
}GpsData_Struct;


//����������Ϣ�ṹ��
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


//ʵʱѪ���仯��Ϣ�ṹ��
typedef __packed struct
{
    uint8_t weakId:4;
    uint8_t way:4;
    uint16_t value;
}RealBloodChangedData_Struct;


//ʵʱ�����Ϣ�ṹ��
typedef __packed struct
{
    float realBulletShootSpeed;
    float realBulletShootFreq;
    float realGolfShootSpeed;
    float realGolfShootFreq;
}RealShootData_Struct;


//����ϵͳ�ṹ��
typedef struct
{
    float RealVoltage;                  //ʵʱ��ѹ
    float RealCurrent;                  //ʵʱ����
    int16_t LastBlood;                  //ʣ��Ѫ��
    uint8_t LastHartID;                 //�ϴ��յ��˺���װ�װ�ID��
    //portTickType LastHartTick;          //�ϴ����˺�ʱ�� 
    float LastShotSpeed;                //�ϴ�����ٶ�
    //��portTickType LastShotTick;          //�ϴ����ʱ��
#if INFANTRY == 7
    uint16_t ShootNum;                  //�ѷ����ӵ���
    uint8_t BulletUseUp;                //1 �����ӵ�����          0 �����ӵ�δ����
    uint16_t ShootFail;                 //����ʧ��ʱ�� 
#endif
}InfantryJudge_Struct;

//����ϵͳ���ݻ���
//__DRIVER_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];
//ʵʱ��ѹ
//__DRIVER_EXT InfantryJudge_Struct InfantryJudge;
////֡�ʼ�����
//__DRIVER_EXT float JudgeFrameCounter;
////֡��
//__DRIVER_EXT float JudgeFrameRate;

/************************ 2017��Ĳ���ϵͳЭ��ṹ�� End ***********************/









/************************ 2018��Ĳ���ϵͳЭ��ṹ�� Start ***********************/
#define JudgeBufferLength_SH       255

#define JudgeBufferLength_PJ        29 //0X0004
#define JudgeBufferLength_Position  25 //0X0008
#define JudgeFrameLength_PJPosi_SH      54 //֮������54����Ϊ�󽮽�04 �� 08�������ݰ��ŵ�һ֡�﷢�ͣ�����У��ȴ������һ������У��
#define JudgeFrameLength_1_SH      54 


#define JudgeFrameLength_2_SH      11
#define JudgeFrameLength_3_SH      24

//����������״̬ (0x0001)
typedef __packed struct
{
	uint16_t stageRemianTime;
	uint8_t gameProgress;
	uint8_t robotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
}extGameRobotState_t;


//ʵʩ�����Ϣ (0x0003)
typedef __packed struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float bulletSpeed;
	int16_t bulletcount;
	uint8_t bulletMAXspeed;
}extShootData_t;


//ʵʱ������������ (0x0004)
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

//������λ�ó�����Ϣ (0x0008)
typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
}extGameRobotPos_t;


//����ϵͳ���ݻ���
//__DRIVER_EXT uint8_t JudgeDataBuffer_SH[JudgeBufferLength_SH];

//__DRIVER_EXT extGameRobotState_t extGameRobotState;

//__DRIVER_EXT extGameRobotPos_t extGameRobotPos;

//__DRIVER_EXT extPowerHeatData_t extPowerHeatData;

//__DRIVER_EXT extShootData_t extShootData;


/************************ 2018��Ĳ���ϵͳЭ��ṹ�� End ***********************/





///************************ 2019��Ĳ���ϵͳЭ��ṹ�� Start ***********************/
typedef enum
{
   Robot_Status_ID                   =0x0201,//������״̬  �ȼ�
   power_heat_data_ID   	           =0x0202,//ǹ������ ���̹���
	 robot_hurt_ID                     =0x0206,//�˺�����
   shoot_data_ID	      					   =0x0207,//��Ƶ����
   student_interactive_header_ID     =0x0301,
} Judege_Cmd_ID;
//֡ͷ


typedef __packed struct
{
  uint8_t   SOF;//0xA5
  uint16_t  DataLenth;//����λ����
	uint8_t   Seq;//�����
  uint8_t   CRC8;//crc8λУ��
}tFrameHeader_Judge;



/*�˺�����*/
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;
/*����������״̬*/
typedef __packed struct
{
	uint8_t robot_id;//������ID
	uint8_t robot_level;//�����˵ȼ�
	uint16_t remain_HP;//������ʣ��HP
	uint16_t max_HP;//����������Ѫ��
	uint16_t shooter_heat0_cooling_rate;//������17mmǹ��ÿ����ȴֵ
	uint16_t shooter_heat0_cooling_limit;//ǹ����������
	uint16_t shooter_heat1_cooling_rate;//42mmǹ��ÿ����ȴֵ
	uint16_t shooter_heat1_cooling_limit;//ǹ����������
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;


/*���̹��ʼ�ǹ������*/
typedef __packed struct
{
	uint16_t chassis_volt;//���������ѹ  ��λ ����
	uint16_t chassis_current;//�����������  ��λ����
	float chassis_power;//�����������  ��λ W
	uint16_t chassis_power_buffer;//���̹��ʻ���  ��λJ
	uint16_t shooter_heat0;//17mmǹ������
	uint16_t shooter_heat1;//42mmǹ������
} ext_power_heat_data_t;



/*�ӵ���Ϣ*/
typedef __packed struct
{
	uint8_t bullet_type;//�ӵ�����
	uint8_t bullet_freq;//�ӵ���Ƶ
	float bullet_speed;//�ӵ�����
} ext_shoot_data_t;

/*�û��Զ��岿����ʾ����Ļ��*/
/*0-1 ��������ID 0xD180
	2-3 �����ߵ�ID
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


//������֮���໥ͨ��
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

/************************ 2019��Ĳ���ϵͳЭ��ṹ�� End ***********************/

#define JudgeFrameHeader        0xA5        //֡ͷ
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
}SelfDefineInfo_t;   //ѧ���ϴ��Զ������� (0x0005)  

 


typedef __packed struct
{
	uint8_t SOF;               //������ʼ�ֽڣ��̶�Ϊ0xA5          
	uint16_t DataLength;       //���ݳ���
	uint8_t Seq;               //�����
	uint8_t CRC8;              //֡ͷCRCУ��
}FrameHeader_t;//֡ͷ



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
