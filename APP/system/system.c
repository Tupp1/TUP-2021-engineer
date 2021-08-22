#include "system.h"
#include "control.h"
#include "remote.h"
#include "power_ctrl.h"
#include "timer.h"
#include "usart.h"
#include "pwm.h"
#include "led.h"
#include "delay.h"
#include "rc.h"
#include "can.h"
#include "iwdg.h"
#include "io.h"
#include "calibrate_Task.h"

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

//系统初始化
void System_Init(void)
{
	REMOTE_vResetData();//遥控初始化
	SYSTEM_InitPeripheral();//外设初始化
}

//外设初始化
void SYSTEM_InitPeripheral(void)
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(configTICK_RATE_HZ);
  led_configuration();
	io_configuration();
	TIM_PWM_Config();
	
	power_ctrl_configuration();
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

	USART3_Judge_Config();
	//stm32 板载温度传感器初始化
  temperature_ADC_init();
	buzzer_init(30000, 90);
	UART8_Datatransfer_Config(500000);
  remote_control_init();
  SYSTEM_OutCtrlProtect();
	SensorOffsetInit(); //flash写入
	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
	{
			power_ctrl_on(i);
			delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
	}
	IWDG_Init(4,30000);
	
}
