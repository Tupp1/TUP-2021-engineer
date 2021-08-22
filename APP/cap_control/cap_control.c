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
  * @brief          ��������ʣ��������� adc
  * @author         SAU
  * @retval         ���ؿ�
  */
static void Cap_Value_Get(void);
static u16 Get_Adc_Average(u8 ch,u8 times);
static u16 Get_Adc(u8 ch);  


//���������������
void Cap_Control(void)
{
	Cap_Value_Get();
	if(JudgementData.power_heat_data_t.chassis_power<65)                //���
	
	{
		if(CAP_VALUE < 23)
	{
		CAP_IN(1);flow_led_off(0);   
	  CAP_OUT(0);flow_led_on(1);
	}
		else              //ͣ
	{
		CAP_IN(0);flow_led_off(0);   
	  CAP_OUT(0);flow_led_on(1);
	}
	}	
	if(JudgementData.power_heat_data_t.chassis_power>=65)                //ͣ
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
  adcx=Get_Adc_Average(ADC_Channel_13,20);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
  CAP_VALUE=(float)adcx*(3.3/4096);          //��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������3.1111
//	adcx=CAP_VALUE;
//	CAP_VALUE-=adcx;
  CAP_VALUE=CAP_VALUE/1.65*23.2;
}

//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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

//���ADCֵ
//ch: @ref ADC_channels 
//ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//����ֵ:ת�����
u16 Get_Adc(u8 ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}

