#include "vision.h"
#include "math.h"
#include "crc_check.h"
#include "usart.h"
#include "led.h"

extVisionSendHeader_t    VisionSendHeader;  //头
extVisionRecvData_t      VisionRecvData;    //视觉接收结构体
extVisionSendData_t      VisionSendData;    //视觉发送结构体

int Usart6_Clean_IDLE_Flag = 0;
//接收到的视觉数据暂存在这里
extern  uint8_t  Com6_Vision_Buffer[ VisionBufferLength_SH ];
	
//格式转换联合体
typedef union
{
    uint8_t U[4];
    float F;
    int I;
		long L;
	  uint16_t U16_t;
}FormatTrans;

#define NOW  0
#define LAST 1

uint32_t Vision_Time_Test[2] = {0};//前后两次事件
uint16_t Vision_Ping = 0;//测试时间间隔
//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Get_New_Data = FALSE;

//打符是否换装甲了
uint8_t Vision_Armor = FALSE;
void Vision_Read_Data(uint8_t *ReadFromUsart)
{
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[0] == VISION_SOF)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
			{
				//接收数据拷贝
				memcpy( &VisionRecvData, ReadFromUsart, VISION_LEN_PACKED);	
				Vision_Get_New_Data = TRUE;//标记视觉数据更新了
				//帧计算
				Vision_Time_Test[NOW] = xTaskGetTickCount();
				Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
				Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
			}
		}
	}
	if(VisionRecvData.yaw_angle == 99.99f)
	{
		memset(Com6_Vision_Buffer, 0, 100);
	}
}

void USART6_IRQHandler(void)		   
{
  if(USART_GetITStatus(USART6,USART_IT_IDLE)!=RESET)//检测到空闲线路
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		Usart6_Clean_IDLE_Flag = USART6->SR ;
		Usart6_Clean_IDLE_Flag = USART6->DR ;
			
		DMA_Cmd(DMA2_Stream1,DISABLE );
		Usart6_Clean_IDLE_Flag = VisionBufferLength_SH - DMA_GetCurrDataCounter(DMA2_Stream1);
		Vision_Read_Data(Com6_Vision_Buffer);//读取视觉数据	
		memset(Com6_Vision_Buffer, 0, 100);
		DMA_Cmd(DMA2_Stream1,ENABLE);
	}
}

/**
  * @brief  获取yaw误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = (-VisionRecvData.yaw_angle +0) / 360.0f * 6.28f;
//				* 8192.0f / 360.0f / 10.0f;//请根据自己对欧拉角的放大倍数来乘对应倍数
	if(VisionRecvData.yaw_angle == 0)//发零
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
float kvision_mouse_pitch = 0.007;
float mouse_pitch_comps = 0;//距离很远时开放鼠标补偿
float vision_pitch_dist = 2;//固定距离,超过此距离开启距离补偿
float vision_pitch_dist_far = 4.4f;//超过此距离开放鼠标补偿
void Vision_Error_Angle_Pitch(float *error)
{	

	{
		*error = -(VisionRecvData.pitch_angle + 0.0f)/ 360.0f * 6.28f ;
		//深大此处数值为0，但是程序是有对距离进行补偿的代码的。
	}
	
	if(VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}


/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}


/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}


/**
  * @brief  获取距离
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Get_Distance(float *distance)
{
	*distance = VisionRecvData.distance;
	if(VisionRecvData.distance < 0)
	{
		*distance = 0;
	}
}
