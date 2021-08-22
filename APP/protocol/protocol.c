#include "protocol.h"
#include "stdlib.h"
#include "string.h"
#include "pid.h"
#include "ist8310.h"
#include "mpu6500.h"
#include "parameters.h"

#define caliMaxTime 100
u8 send_pid1=0,send_pid2=0,send_pid3=0;
u8 checkdata_to_send,checksum_to_send,send_check=0;
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
		if(*(data_buf+4)==0X01)      IMU_ACCERDataCali(); //AA AF 01 01 ----> ACC校准 （加速度计）
		else if(*(data_buf+4)==0X02) IMU_GYRODataCali();  //AA AF 01 02 ----> GYRO校准 （陀螺仪）
		else if(*(data_buf+4)==0X03)                      //AA AF 01 03 ----> ACC GYRO校准
		{
			IMU_ACCERDataCali();	
			IMU_GYRODataCali();	
		}
		else if(*(data_buf+4)==0X04) 	IMU_MAGDataCali();  //AA AF 01 04 ----> MAG校准 （罗盘）
		else if(*(data_buf+4)==0X05) 	ParaSavingFlag=1;   //AA AF 01 05 ----> BARO校准  （点击气压计校准才会保持）
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
			ParametersInit();
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
