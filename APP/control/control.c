#include "control.h"
#include "remote.h"
#include "CAN_Receive.h"

/**********************系统控制判断及保护******************/

//系统状态
eSystemState systemState = SYSTEM_STARTING;

/**
  * @brief  系统重置
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_Reset( void )
{
	systemState = SYSTEM_STARTING;
}

/**
  * @brief  失控保护
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_OutCtrlProtect(void)
{
  SYSTEM_Reset();          //系统恢复至重启状态
	REMOTE_vResetData();     //遥控数据恢复至默认状态
	CAN_CMD_GIMBAL_STOP();
	CAN_CMD_CHASSIS_STOP();
	
}

/**
  * @brief  更新系统状态
  * @param  void
  * @retval void
  * @attention 1kHz,在LOOP循环调用
  */
void SYSTEM_UpdateSystemState(void)
{
	static uint32_t  ulInitCnt  =  0;
	if (systemState == SYSTEM_STARTING)
	{
		ulInitCnt++;
		if (ulInitCnt > 1500)//启动延时,1ms*2k=2s,为了给MPU启动时间
		{
			ulInitCnt = 0;
			systemState = SYSTEM_RUNNING;//启动完成,转换成普通模式
		}
	}
}


//返回系统状态
eSystemState SYSTEM_GetSystemState( )
{
	return systemState;
}




