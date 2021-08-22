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
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
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





