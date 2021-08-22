#include "cap_control.h"
#include "Remote_Control.h"
#include "Cap.h"
#include "led.h"
#include "CAN_Receive.h"
#include "chassis_task.h"
#include "judge_analysis.h"
#include "delay.h"
#include "Remote_Control.h"

/**
  * @brief          超级电容剩余电量反馈 adc
  * @author         SAU
  * @retval         返回空
  */
static void Cap_Value_Get(void);
static u16 Get_Adc_Average(u8 ch,u8 times);
static u16 Get_Adc(u8 ch);  


//超级电容任务控制
void Cap_Control(void)
{
	Cap_Value_Get();
	if(JudgementData.power_heat_data_t.chassis_power<65)                //充电
	
	{
		if(CAP_VALUE < 23)
	{
		CAP_IN(1);flow_led_off(0);   
	  CAP_OUT(0);flow_led_on(1);
	}
		else              //停
	{
		CAP_IN(0);flow_led_off(0);   
	  CAP_OUT(0);flow_led_on(1);
	}
	}	
	if(JudgementData.power_heat_data_t.chassis_power>=65)                //停
	{
	
		
		CAP_IN(0);flow_led_off(0);   
	  CAP_OUT(0);flow_led_on(1);

	}
	
	if(CAP_VALUE >= 20)
	{
		flag_CAP_FULL = 1;
	}
		if(CAP_VALUE < 5)
	{
		flag_CAP_FULL = 0;
	}
	
	if( (RC_CAP & KEY_PRESSED_OFFSET_SHIFT)&&(flag_CAP_FULL == 1))
	{
		CAP_IN(0);flow_led_off(0);   
	  CAP_OUT(1);flow_led_on(1);
	}

}



void Cap_Value_Get(void)
{
	u16 adcx;
  adcx=Get_Adc_Average(ADC_Channel_13,20);//获取通道5的转换值，20次取平均
  CAP_VALUE=(float)adcx*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
//	adcx=CAP_VALUE;
//	CAP_VALUE-=adcx;
  CAP_VALUE=CAP_VALUE/1.65*23.2;
}

//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 

//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

