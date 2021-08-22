#include "control.h"
#include "remote.h"
#include "CAN_Receive.h"

/**********************ϵͳ�����жϼ�����******************/

//ϵͳ״̬
eSystemState systemState = SYSTEM_STARTING;

/**
  * @brief  ϵͳ����
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_Reset( void )
{
	systemState = SYSTEM_STARTING;
}

/**
  * @brief  ʧ�ر���
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_OutCtrlProtect(void)
{
  SYSTEM_Reset();          //ϵͳ�ָ�������״̬
	REMOTE_vResetData();     //ң�����ݻָ���Ĭ��״̬
	CAN_CMD_GIMBAL_STOP();
	CAN_CMD_CHASSIS_STOP();
	
}

/**
  * @brief  ����ϵͳ״̬
  * @param  void
  * @retval void
  * @attention 1kHz,��LOOPѭ������
  */
void SYSTEM_UpdateSystemState(void)
{
	static uint32_t  ulInitCnt  =  0;
	if (systemState == SYSTEM_STARTING)
	{
		ulInitCnt++;
		if (ulInitCnt > 1500)//������ʱ,1ms*2k=2s,Ϊ�˸�MPU����ʱ��
		{
			ulInitCnt = 0;
			systemState = SYSTEM_RUNNING;//�������,ת������ͨģʽ
		}
	}
}


//����ϵͳ״̬
eSystemState SYSTEM_GetSystemState( )
{
	return systemState;
}




