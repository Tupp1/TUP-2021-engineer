#ifndef __JUDGE_H
#define __JUDGE_H
#include "stm32f4xx.h"

//#define JudgeBufferLength       255
//#define JudgeFrameHeader        0xA5        //֡ͷ 
//#define FrameHeader_Len         5 
//extern uint8_t JudgeDataBuffer[JudgeBufferLength];
//extern uint8_t JudgeDataBuffer_Send[JudgeBufferLength];
///*__packed  c���Թؼ��֣��������ṹ���е����ݾͲ����׵�ַ����
//����˵���������ݻᱻѹ��
//1.ͨ�������������ɴ����ʱ�򶼻���нṹ�����䣬��֤���ṹ���ڲ���Ա��������ܵĶ��뷽ʽ
//2.�������Զ���������ṹ����ڴ棨���綨��Ϊȫ�ֱ�����ֲ��������϶��Ƕ����
//*/
//typedef enum
//{
//   Robot_Status_ID                   =0x0201,//������״̬  �ȼ�
//   power_heat_data_ID   	           =0x0202,//ǹ������ ���̹���
//	 robot_hurt_ID                     =0x0206,//�˺�����
//   shoot_data_ID	      					   =0x0207,//��Ƶ����
//   student_interactive_header_ID     =0x0301,
//} Judege_Cmd_ID;
////֡ͷ
//typedef __packed struct
//{
//  uint8_t   SOF;//0xA5
//  uint16_t  DataLenth;//����λ����
//	uint8_t   Seq;//�����
//  uint8_t   CRC8;//crc8λУ��
//}tFrameHeader;
///*�˺�����*/
//typedef __packed struct
//{
//uint8_t armor_id : 4;
//uint8_t hurt_type : 4;
//} ext_robot_hurt_t;
///*����������״̬*/
//typedef __packed struct
//{
//	uint8_t robot_id;//������ID
//	uint8_t robot_level;//�����˵ȼ�
//	uint16_t remain_HP;//������ʣ��HP
//	uint16_t max_HP;//����������Ѫ��
//	uint16_t shooter_heat0_cooling_rate;//������17mmǹ��ÿ����ȴֵ
//	uint16_t shooter_heat0_cooling_limit;//ǹ����������
//	uint16_t shooter_heat1_cooling_rate;//42mmǹ��ÿ����ȴֵ
//	uint16_t shooter_heat1_cooling_limit;//ǹ����������
//	uint8_t mains_power_gimbal_output : 1;
//	uint8_t mains_power_chassis_output : 1;
//	uint8_t mains_power_shooter_output : 1;
//} ext_game_robot_state_t;


///*���̹��ʼ�ǹ������*/
//typedef __packed struct
//{
//	uint16_t chassis_volt;//���������ѹ  ��λ ����
//	uint16_t chassis_current;//�����������  ��λ����
//	float chassis_power;//�����������  ��λ W
//	uint16_t chassis_power_buffer;//���̹��ʻ���  ��λJ
//	uint16_t shooter_heat0;//17mmǹ������
//	uint16_t shooter_heat1;//42mmǹ������
//} ext_power_heat_data_t;



///*�ӵ���Ϣ*/
//typedef __packed struct
//{
//	uint8_t bullet_type;//�ӵ�����
//	uint8_t bullet_freq;//�ӵ���Ƶ
//	float bullet_speed;//�ӵ�����
//} ext_shoot_data_t;

///*�û��Զ��岿����ʾ����Ļ��*/
///*0-1 ��������ID 0xD180
//	2-3 �����ߵ�ID
//*/
//typedef __packed struct
//{
//	float data1;//�Զ��帡��������1
//	float data2;//�Զ��帡��������2
//	float data3;//�Զ��帡��������3
//	uint8_t masks;//�Զ���8λ����4 0-5 ���ƿͻ����Զ����������ϵ�6��ָʾ�� 1Ϊ��ɫ 0Ϊ��ɫ
//} client_custom_data_t;
////������֮���໥ͨ��
//typedef __packed struct
//{
//	uint8_t data[112];
//} robot_interactive_data_t;


//typedef  struct
//{
//  tFrameHeader          frameHeader;//֡ͷ
//  uint16_t              rxCmdId;//������
//  ext_power_heat_data_t power_heat_data_t;//���̹���ǹ����������
//  ext_shoot_data_t		  shoot_data_t;//�����Ϣ
//	ext_robot_hurt_t      robot_hurt_t;//�˺�����
//	ext_game_robot_state_t game_robot_state_t;//�����˻�����Ϣ
//  robot_interactive_data_t robot_data_t;//������֮�佻��
//	client_custom_data_t userinfo;//�û��Զ���
//}JudgementDataTypedef;


//extern JudgementDataTypedef JudgementData;




//void judgeCalculate(uint8_t * JudgeDataBuffer);
#endif

