#include "vision.h"
#include "math.h"
#include "crc_check.h"
#include "usart.h"
#include "led.h"

extVisionSendHeader_t    VisionSendHeader;  //ͷ
extVisionRecvData_t      VisionRecvData;    //�Ӿ����սṹ��
extVisionSendData_t      VisionSendData;    //�Ӿ����ͽṹ��

int Usart6_Clean_IDLE_Flag = 0;
//���յ����Ӿ������ݴ�������
extern  uint8_t  Com6_Vision_Buffer[ VisionBufferLength_SH ];
	
//��ʽת��������
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

uint32_t Vision_Time_Test[2] = {0};//ǰ�������¼�
uint16_t Vision_Ping = 0;//����ʱ����
//�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
uint8_t Vision_Get_New_Data = FALSE;

//����Ƿ�װ����
uint8_t Vision_Armor = FALSE;
void Vision_Read_Data(uint8_t *ReadFromUsart)
{
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[0] == VISION_SOF)
	{
		//֡ͷCRC8У��
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
		{
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
			{
				//�������ݿ���
				memcpy( &VisionRecvData, ReadFromUsart, VISION_LEN_PACKED);	
				Vision_Get_New_Data = TRUE;//����Ӿ����ݸ�����
				//֡����
				Vision_Time_Test[NOW] = xTaskGetTickCount();
				Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//����ʱ����
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
  if(USART_GetITStatus(USART6,USART_IT_IDLE)!=RESET)//��⵽������·
	{		
		//����ST�ٷ��ֲ�,��һ��SR��DR�Ĵ���,IDLE�����ٴ�ʹ��,�����һֱ�����ж�,�ͻ�����ڽ����ж�û����
		Usart6_Clean_IDLE_Flag = USART6->SR ;
		Usart6_Clean_IDLE_Flag = USART6->DR ;
			
		DMA_Cmd(DMA2_Stream1,DISABLE );
		Usart6_Clean_IDLE_Flag = VisionBufferLength_SH - DMA_GetCurrDataCounter(DMA2_Stream1);
		Vision_Read_Data(Com6_Vision_Buffer);//��ȡ�Ӿ�����	
		memset(Com6_Vision_Buffer, 0, 100);
		DMA_Cmd(DMA2_Stream1,ENABLE);
	}
}

/**
  * @brief  ��ȡyaw���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
	*error = (-VisionRecvData.yaw_angle +0) / 360.0f * 6.28f;
//				* 8192.0f / 360.0f / 10.0f;//������Լ���ŷ���ǵķŴ������˶�Ӧ����
	if(VisionRecvData.yaw_angle == 0)//����
	{
		*error = 0;
	}
}

/**
  * @brief  ��ȡpitch���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  �Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ
  */
float kvision_mouse_pitch = 0.007;
float mouse_pitch_comps = 0;//�����Զʱ������겹��
float vision_pitch_dist = 2;//�̶�����,�����˾��뿪�����벹��
float vision_pitch_dist_far = 4.4f;//�����˾��뿪����겹��
void Vision_Error_Angle_Pitch(float *error)
{	

	{
		*error = -(VisionRecvData.pitch_angle + 0.0f)/ 360.0f * 6.28f ;
		//���˴���ֵΪ0�����ǳ������жԾ�����в����Ĵ���ġ�
	}
	
	if(VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}


/**
  * @brief  �ж��Ӿ����ݸ�������
  * @param  void
  * @retval TRUE������   FALSEû����
  * @attention  Ϊ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Get_New_Data��TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}


/**
  * @brief  �Ӿ����ݸ��±�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}


/**
  * @brief  ��ȡ����
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
