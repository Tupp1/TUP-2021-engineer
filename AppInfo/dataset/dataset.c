#include "remote.h"
#include "dataset.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "timer.h"
#include "judge.h"
#include "calibrate_task.h"
#include "CAN_receive.h"
#include "sys_config.h"

#include "UDTrans_task.h"
#define caliMaxTime 100
u8 send_pid1=0,send_pid2=0,send_pid3=0;
u8 checkdata_to_send,checksum_to_send,send_check=0;
u32 SelfCheckErrorFlag=0x0000;

u8 data_to_send[50];
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
extern UDTrans_Control_t UDTrans_Control;


void DatatransferTask(void)
{
 
//		ANO_DT_Send_Status(UDTrans_Control.rotatemotor[0].motor_angle*0.001f,UDTrans_Control.rotatemotor[1].motor_angle*0.001f,5,SelfCheckErrorFlag,0,0);
//	

////		ANO_DT_Send_Senser((vs16)MPU6500_Acc.x,(vs16)MPU6500_Acc.y,(vs16)MPU6500_Acc.z,
////											 (vs16)MPU6500_Gyro.x,(vs16)MPU6500_Gyro.y,(vs16)MPU6500_Gyro.z,
////											 (vs16)MagValue.x,(vs16)MagValue.y,(vs16)MagValue.z);
//		
//	
//	
//		ANO_DT_Send_RCData(UDTrans_Control.rotatemotor[0].motor_angle,
//		                   UDTrans_Control.rotatemotor[0].motor_angle_set,
//											 RC_Ctl.rc.ch[2]+660,
//											 RC_Ctl.rc.ch[3]+660,
//											 RC_Ctl.rc.s[0]*300+1000,
//											 RC_Ctl.rc.s[1]*300+1000,
//											 RC_Ctl.mouse.x+1000,
//											 RC_Ctl.mouse.y+1000,
//											 motor_pit.ecd,    //pitch
//											 motor_yaw.ecd);          //yaw
	

	if (send_check)
	{
		send_check = 0;
		ANO_DT_Send_Check(checkdata_to_send,checksum_to_send);
	}
	
	if (send_pid1)
	{
		send_pid1=0;
		ANO_DT_Send_PID(1,PID_arg[0].kp,PID_arg[0].ki,PID_arg[0].kd,
											PID_arg[1].kp,PID_arg[1].ki,PID_arg[1].kd,
											PID_arg[2].kp,PID_arg[2].ki,PID_arg[2].kd);
		
	}
	else if(send_pid2)
	{
		send_pid2=0;
		ANO_DT_Send_PID(2,PID_arg[3].kp,PID_arg[3].ki,PID_arg[3].kd,
											PID_arg[4].kp,PID_arg[4].ki,PID_arg[4].kd,
											PID_arg[5].kp,PID_arg[5].ki,PID_arg[5].kd);
	}
	else if (send_pid3)
	{
		send_pid3=0;
		ANO_DT_Send_PID(3,PID_arg[6].kp,PID_arg[6].ki,PID_arg[6].kd,
									    PID_arg[7].kp,PID_arg[7].ki,PID_arg[7].kd,
								    	0,0,0);
	}
}


/**
  * @brief 向上位机发送三个欧拉角
  * @param 三个欧拉角，
  * @retval None
  */
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, u32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Data_Send(data_to_send, _cnt);
}

/**
  * @brief 发送遥控器数据
  * @param None
  * @retval None
  */
void ANO_DT_Send_RCData(u16 ch0,u16 ch1,u16 ch2,u16 ch3,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	u8 i=0;
	u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(ch0);
	data_to_send[_cnt++]=BYTE0(ch0);
	data_to_send[_cnt++]=BYTE1(ch1);
	data_to_send[_cnt++]=BYTE0(ch1);
	data_to_send[_cnt++]=BYTE1(ch2);
	data_to_send[_cnt++]=BYTE0(ch2);
	data_to_send[_cnt++]=BYTE1(ch3);
	data_to_send[_cnt++]=BYTE0(ch3);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Data_Send(data_to_send, _cnt);
}


void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
		u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Data_Send(data_to_send, _cnt);
}




/**
  * @brief 发送PID至上位机
  * @param 三组PID的值及其group序号
  * @retval None
  * @details None
  */
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum=0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Data_Send(data_to_send, _cnt);
}
/**
  * @brief 接收地面站的数据
  * @param data 
  * @retval None
  */

void UART8_DataPrepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)						//帧头第一个字节AA
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)			  //帧头第二个字节AF
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)			
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)				
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)		
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)								
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		BasicProtocolAnalysis(RxBuffer,_data_cnt+5);
	}
	else state = 0;
}


/**
  * @brief 发送attack
  * @param head 帧类型
  * @param check_sum 校验位
  * @retval None
  */
void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	

	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	Data_Send(data_to_send, 7);
}

static void Data_Send(u8 * _val, u8 _len)
{
	UART8_Send(_val, _len);
}


/**
  * @brief 基本串口通讯协议解析
  * @param data_buf	包含完整一帧数据的数组的指针
	* @param _len		帧总长
  * @retval None
  * @details 上层硬件发来的信号或是地面站发来的信号的解析函数
  */
void BasicProtocolAnalysis(u8 const *data_buf,int _len)
{
	u8 sum = 0;
	u8 i;
	for(i=0;i<(_len-1);i++)		sum += *(data_buf+i);										//求和校验
	if(!(sum==*(data_buf+_len-1)))		return;						//校验不成功则return
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;	//帧头校验不成功则return
	
	
	/****************↓↓↓↓分析帧类型 start  AA AF ↓↓↓↓*******************/
	/**********************↓↓↓↓ AA AF 01 ↓↓↓↓***********************/
	if(*(data_buf+2)==0X01)													
	{
//		if(*(data_buf+4)==0X01)      IMU_ACCERDataCali(); //AA AF 01 01 ----> ACC校准 （加速度计）
//		else if(*(data_buf+4)==0X02) IMU_GYRODataCali();  //AA AF 01 02 ----> GYRO校准 （陀螺仪）
//		else if(*(data_buf+4)==0X03)                      //AA AF 01 03 ----> ACC GYRO校准
//		{
//			IMU_ACCERDataCali();	
//			IMU_GYRODataCali();	
//		}
//		else if(*(data_buf+4)==0X04) 	IMU_MAGDataCali();  //AA AF 01 04 ----> MAG校准 （罗盘）
		 if(*(data_buf+4)==0X05) 	ParaSavingFlag=1;   //AA AF 01 05 ----> BARO校准  （点击气压计校准才会保持）
	}
	/**********************↑↑↑↑ AA AF 01 ↑↑↑↑***********************/
	/**********************↓↓↓↓ AA AF 02 ↓↓↓↓***********************/
	if(*(data_buf+2)==0X02) 
	{
		if(*(data_buf+4)==0X01) //AA AF 02 01 ----> 读取pid请求
		{
			send_pid1 = 1;
			send_pid2 = 1;
			send_pid3 = 1;
		}
		if(*(data_buf+4)==0XA1)	//AA AF 02 A1 ----> 恢复默认参数
		{
			//ParametersInit();
		}
	}
	/**********************↑↑↑↑ AA AF 02 ↑↑↑↑***********************/
	/**********************↓↓↓↓ AA AF 10 ↓↓↓↓***********************/ 
		if(*(data_buf+2)==0X10)								//AA AF 10 ----> PID1-3
    {
        PID_arg[0].kp = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_arg[0].ki = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_arg[0].kd = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_arg[1].kp = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_arg[1].ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_arg[1].kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        PID_arg[2].kp = ( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_arg[2].ki = 0.01*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_arg[2].kd = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
	/**********************↑↑↑↑ AA AF 10 ↑↑↑↑***********************/ 
  /**********************↓↓↓↓ AA AF 11 ↓↓↓↓***********************/ 		
		if(*(data_buf+2)==0X11)								//AA AF 11 ----> PID4-6
    {
        PID_arg[3].kp = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_arg[3].ki = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_arg[3].kd = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_arg[4].kp = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_arg[4].ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_arg[4].kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
				PID_arg[5].kp = ( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_arg[5].ki = 0.01*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_arg[5].kd = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
	/**********************↑↑↑↑ AA AF 11 ↑↑↑↑***********************/ 
  /**********************↓↓↓↓ AA AF 12 ↓↓↓↓***********************/ 		
    if(*(data_buf+2)==0X12)								//AA AF 12 ----> PID7-8
    {	
			  PID_arg[6].kp = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_arg[6].ki = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_arg[6].kd = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_arg[7].kp = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_arg[7].ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_arg[7].kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       	if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
	/**********************↑↑↑↑ AA AF 12 ↑↑↑↑***********************/ 
  /**********************↓↓↓↓ AA AF 13&14&15 ↓↓↓↓***********************/ 
			if(*(data_buf+2)==0X13)								//PID4
	{
		   	if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
						if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
						if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
	}
	/**********************↑↑↑↑ AA AF 13&14&15 ↑↑↑↑***********************/ 
	/****************↑↑↑↑ 分析帧类型 start  AA AF ↑↑↑↑ *******************/
}

