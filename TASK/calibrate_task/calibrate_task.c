/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      完成校准相关设备的数据，包括云台，陀螺仪，加速度计，磁力计
  *             云台校准主要是中值，最大最小相对角度，陀螺仪主要校准零漂
  *             加速度计和磁力计只是写好接口函数，加速度计目前没有必要校准，
  *             磁力计尚未使用在解算算法中。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */


#include "calibrate_Task.h"
#include "flash.h"
#include "Remote.h"
#include "string.h"
#include "sys_config.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
u8 CALIFLAG=0;
u8 ParamSavedFlag=0;
u8 ParaSavingFlag=0;

IMUSensor_OffSet__ IMUSensor_Offset;
AllDataUnion__ AllDataUnion; 

_PID_arg_st PID_arg[PIDGROUPLEN]={CHASSIS_Rot_PID_OFF,
																	CHASSIS_Vec_PID_OFF,
																	GIMBALP_Pos_PID_OFF,
																	GIMBALP_Vec_PID_OFF,
	
																	GIMBALY_Pos_PID_OFF,
																	GIMBALY_Vec_PID_OFF,
																	SLIBLIN_Pos_PID_OFF,
																	SLIBLIN_Vec_PID_OFF};
_PID_val_st PID_val[PIDGROUPLEN+3];
	


static int8_t temperate;
static int8_t temperate_temp;
static uint32_t calibrate_systemTick;
const RC_ctrl_t *calibrate_RC; //遥控器结构体指针
static imu_cali_t gyro_cali;          //陀螺仪校准数据
char cali_sensor_cmd[5]; //校准设备数组，

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

void calibrate_task(void *pvParameters)
{
	  calibrate_RC = get_remote_ctrl_point_cali();
    while (1)
    {
			//遥控器操作校准步骤
      RC_cmd_to_calibrate();
			if(cali_sensor_cmd[CALI_GYRO])
			{
				//调用校准函数
				if (cali_gyro_hook((uint32_t *)&gyro_cali, CALI_FUNC_CMD_ON))
  			{
						cali_sensor_cmd[CALI_GYRO]=0;
					  ParametersSave();
 				}
			}

      vTaskDelay(CALIBRATE_CONTROL_TIME);
			
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

//遥控器操作校准云台，陀螺仪
static void RC_cmd_to_calibrate(void)
{
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t rc_cmd_time = 0;
    static uint8_t rc_action_flag = 0;
    static uint16_t buzzer_time = 0;
	if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
	{
			//判断遥控器2s内八开始20s校准选择时间，rc_action_falg及rc_cmd_time在下方逻辑判断
			rc_cmd_systemTick = xTaskGetTickCount();
			rc_action_flag = 1;
			rc_cmd_time = 0;
	}
	else if (rc_action_flag == 2 && rc_cmd_time > RC_CMD_LONG_TIME)
	{
			//判断遥控器在20s校准选择时间，上外八使能云台校准，并且保持2s,rc_action_falg及rc_cmd_time在下方逻辑判断
			rc_action_flag = 0;
			rc_cmd_time = 0;
			//cali_sensor[CALI_GIMBAL].cali_cmd = 1;
	}
	else if (rc_action_flag == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
	{
			//判断遥控器在20s校准选择时间，下外八使能陀螺仪校准，并且保持2s，rc_action_falg及rc_cmd_time在下方逻辑判断
			rc_action_flag = 0;
			rc_cmd_time = 0;
			cali_sensor_cmd[CALI_GYRO] = 1;
			//更新MPU6500需要控制的温度
			temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
			if (temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
			{
					temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
			}
	}
		if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE &&  \
			  calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE &&  \
		    calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE &&   \
		    calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE &&  \
		    switch_is_down(calibrate_RC->rc.s[0]) &&         \
		    switch_is_down(calibrate_RC->rc.s[1]) &&         \
		    rc_action_flag == 0)
    {
        //判断遥控器2s内八 计时的时间， 当rc_cmd_time > 2000 为保持2s
        rc_cmd_time++;
    }
    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE &&  \
			       calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE &&  \
		         calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && \
		         calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE &&  \
		         switch_is_down(calibrate_RC->rc.s[0]) &&        \
		         switch_is_down(calibrate_RC->rc.s[1]) &&        \
		         rc_action_flag != 0)
    {
        //判断遥控器2s上外八 计时的时间， 云台使能
        rc_cmd_time++;
        rc_action_flag = 2;
    }

    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE &&  \
			       calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && \
		         calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && \
		         calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && \
		         switch_is_down(calibrate_RC->rc.s[0]) &&        \
		         switch_is_down(calibrate_RC->rc.s[1]) &&        \
		         rc_action_flag != 0)
    {
        //判断遥控器2s下外八 计时的时间， 陀螺仪使能
        rc_cmd_time++;
        rc_action_flag = 3;
    }
    else
    {
        rc_cmd_time = 0;
    }
    calibrate_systemTick = xTaskGetTickCount();
		
		if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        //判断遥控器20s校准选择时间，无操作
        rc_action_flag = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        //判断遥控器10s后校准选择时间切换蜂鸣器高频声音
        //rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        //遥控器10s前 开始蜂鸣器低频声音
        rc_cali_buzzer_start_on();
    }

    if (rc_action_flag != 0)
    {
        buzzer_time++;
    }
    //蜂鸣器断续发声
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
    {
        cali_buzzer_off();
    }
}




//校准陀螺仪设备，主要校准零漂
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            cali_buzzer_off();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); //掉线遥控器以防误操作
            imu_start_buzzer();
            return 0;
        }
    }
    return 0;
}

//初始化校准结构体数组，读取flash值，如果未校准，使能校准命令,同时初始化对应校准数据
/**
  * @brief 传感器校准值初始化
  * @param None
  * @retval None
  */
void SensorOffsetInit(void)
{
	BSP_FLASH_Read(ADDR_FLASH_SECTOR_11, AllDataUnion.data, sizeof(AllDataOffset__));
	
	//-.-！原谅我C语言水平有限。。。。并且此处数据转换非常之混乱。。。忘见谅
	gyro_cali.offset[0]=AllDataUnion.AllData.imu_offset.GYRO_Offset.x;
	gyro_cali.offset[1]=AllDataUnion.AllData.imu_offset.GYRO_Offset.y;
	gyro_cali.offset[2]=AllDataUnion.AllData.imu_offset.GYRO_Offset.z;
	
	cali_gyro_hook((uint32_t *)&gyro_cali, CALI_FUNC_CMD_INIT);
}


//返回mpu6500需要控制到的温度
int8_t get_control_temperate(void)
{
    return temperate_temp = (int8_t)(cali_get_mcu_temperature()) + 10;
}


void ParametersSave(void)
{
	int i=0;
	for (i=0;i<PIDGROUPLEN;i++)
	{
		AllDataUnion.AllData.PID_ARG[i]=PID_arg[i];
	}
	AllDataUnion.AllData.savedflag=1;
	AllDataUnion.AllData.imu_offset=IMUSensor_Offset;
	BSP_FLASH_Write(ADDR_FLASH_SECTOR_11, AllDataUnion.data, sizeof(AllDataOffset__));
}
