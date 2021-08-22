#ifndef DATASET_H
#define DATASET_H
#include "main.h"
#include "stm32f4xx.h"

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, u32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_RCData(u16 ch0,u16 ch1,u16 ch2,u16 ch3,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_Check(u8 head, u8 check_sum);
void UART8_DataPrepare(u8 data);
void BasicProtocolAnalysis(u8 const *data_buf,int _len);
static void Data_Send(u8 * _val, u8 _len);
void DatatransferTask(void);
#endif
