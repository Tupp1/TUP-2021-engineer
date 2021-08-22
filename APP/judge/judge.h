#ifndef __JUDGE_H
#define __JUDGE_H
#include "stm32f4xx.h"

//#define JudgeBufferLength       255
//#define JudgeFrameHeader        0xA5        //帧头 
//#define FrameHeader_Len         5 
//extern uint8_t JudgeDataBuffer[JudgeBufferLength];
//extern uint8_t JudgeDataBuffer_Send[JudgeBufferLength];
///*__packed  c语言关键字，有了它结构体中的数据就不会首地址对齐
//进而说有了它数据会被压缩
//1.通常编译器在生成代码的时候都会进行结构体的填充，保证（结构体内部成员）最高性能的对齐方式
//2.编译器自动分配出来结构体的内存（比如定义为全局变量或局部变量）肯定是对齐的
//*/
//typedef enum
//{
//   Robot_Status_ID                   =0x0201,//机器人状态  等级
//   power_heat_data_ID   	           =0x0202,//枪口热量 底盘功率
//	 robot_hurt_ID                     =0x0206,//伤害类型
//   shoot_data_ID	      					   =0x0207,//射频射速
//   student_interactive_header_ID     =0x0301,
//} Judege_Cmd_ID;
////帧头
//typedef __packed struct
//{
//  uint8_t   SOF;//0xA5
//  uint16_t  DataLenth;//数据位长度
//	uint8_t   Seq;//包序号
//  uint8_t   CRC8;//crc8位校验
//}tFrameHeader;
///*伤害类型*/
//typedef __packed struct
//{
//uint8_t armor_id : 4;
//uint8_t hurt_type : 4;
//} ext_robot_hurt_t;
///*比赛机器人状态*/
//typedef __packed struct
//{
//	uint8_t robot_id;//机器人ID
//	uint8_t robot_level;//机器人等级
//	uint16_t remain_HP;//机器人剩余HP
//	uint16_t max_HP;//机器人上限血量
//	uint16_t shooter_heat0_cooling_rate;//机器人17mm枪口每秒冷却值
//	uint16_t shooter_heat0_cooling_limit;//枪口热量上限
//	uint16_t shooter_heat1_cooling_rate;//42mm枪口每秒冷却值
//	uint16_t shooter_heat1_cooling_limit;//枪口热量上限
//	uint8_t mains_power_gimbal_output : 1;
//	uint8_t mains_power_chassis_output : 1;
//	uint8_t mains_power_shooter_output : 1;
//} ext_game_robot_state_t;


///*底盘功率及枪口热量*/
//typedef __packed struct
//{
//	uint16_t chassis_volt;//底盘输出电压  单位 毫伏
//	uint16_t chassis_current;//底盘输出电流  单位毫安
//	float chassis_power;//底盘输出功率  单位 W
//	uint16_t chassis_power_buffer;//底盘功率缓冲  单位J
//	uint16_t shooter_heat0;//17mm枪口热量
//	uint16_t shooter_heat1;//42mm枪口热量
//} ext_power_heat_data_t;



///*子弹信息*/
//typedef __packed struct
//{
//	uint8_t bullet_type;//子弹类型
//	uint8_t bullet_freq;//子弹射频
//	float bullet_speed;//子弹射速
//} ext_shoot_data_t;

///*用户自定义部分显示在屏幕上*/
///*0-1 数据内容ID 0xD180
//	2-3 发送者的ID
//*/
//typedef __packed struct
//{
//	float data1;//自定义浮点型数据1
//	float data2;//自定义浮点型数据2
//	float data3;//自定义浮点型数据3
//	uint8_t masks;//自定义8位数据4 0-5 控制客户端自定义控制面板上的6个指示灯 1为绿色 0为红色
//} client_custom_data_t;
////机器人之间相互通信
//typedef __packed struct
//{
//	uint8_t data[112];
//} robot_interactive_data_t;


//typedef  struct
//{
//  tFrameHeader          frameHeader;//帧头
//  uint16_t              rxCmdId;//命令码
//  ext_power_heat_data_t power_heat_data_t;//底盘功率枪口热量数据
//  ext_shoot_data_t		  shoot_data_t;//射击信息
//	ext_robot_hurt_t      robot_hurt_t;//伤害类型
//	ext_game_robot_state_t game_robot_state_t;//机器人基本信息
//  robot_interactive_data_t robot_data_t;//机器人之间交互
//	client_custom_data_t userinfo;//用户自定义
//}JudgementDataTypedef;


//extern JudgementDataTypedef JudgementData;




//void judgeCalculate(uint8_t * JudgeDataBuffer);
#endif

