#include "ramp.h"

_ramp_st RcKeyTowardRamp={30,0};
_ramp_st RcKeyLeftRightRamp={30,0};
_ramp_st RcKeyRotateRamp={30,0};


void RampReset(_ramp_st * ramp)
{
	ramp->Num_Current=0;
}

float RampCalculate(_ramp_st * ramp)
{
	if (ramp->Num_Current<ramp->Num_Max)
	{
		ramp->Num_Current++;
	}
	return ramp->Num_Current/ramp->Num_Max;
}






/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}





