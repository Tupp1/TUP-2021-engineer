/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      ���У׼����豸�����ݣ�������̨�������ǣ����ٶȼƣ�������
  *             ��̨У׼��Ҫ����ֵ�������С��ԽǶȣ���������ҪУ׼��Ư
  *             ���ٶȼƺʹ�����ֻ��д�ýӿں��������ٶȼ�Ŀǰû�б�ҪУ׼��
  *             ��������δʹ���ڽ����㷨�С�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
const RC_ctrl_t *calibrate_RC; //ң�����ṹ��ָ��
static imu_cali_t gyro_cali;          //������У׼����
char cali_sensor_cmd[5]; //У׼�豸���飬

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

void calibrate_task(void *pvParameters)
{
	  calibrate_RC = get_remote_ctrl_point_cali();
    while (1)
    {
			//ң��������У׼����
      RC_cmd_to_calibrate();
			if(cali_sensor_cmd[CALI_GYRO])
			{
				//����У׼����
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

//ң��������У׼��̨��������
static void RC_cmd_to_calibrate(void)
{
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t rc_cmd_time = 0;
    static uint8_t rc_action_flag = 0;
    static uint16_t buzzer_time = 0;
	if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
	{
			//�ж�ң����2s�ڰ˿�ʼ20sУ׼ѡ��ʱ�䣬rc_action_falg��rc_cmd_time���·��߼��ж�
			rc_cmd_systemTick = xTaskGetTickCount();
			rc_action_flag = 1;
			rc_cmd_time = 0;
	}
	else if (rc_action_flag == 2 && rc_cmd_time > RC_CMD_LONG_TIME)
	{
			//�ж�ң������20sУ׼ѡ��ʱ�䣬�����ʹ����̨У׼�����ұ���2s,rc_action_falg��rc_cmd_time���·��߼��ж�
			rc_action_flag = 0;
			rc_cmd_time = 0;
			//cali_sensor[CALI_GIMBAL].cali_cmd = 1;
	}
	else if (rc_action_flag == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
	{
			//�ж�ң������20sУ׼ѡ��ʱ�䣬�����ʹ��������У׼�����ұ���2s��rc_action_falg��rc_cmd_time���·��߼��ж�
			rc_action_flag = 0;
			rc_cmd_time = 0;
			cali_sensor_cmd[CALI_GYRO] = 1;
			//����MPU6500��Ҫ���Ƶ��¶�
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
        //�ж�ң����2s�ڰ� ��ʱ��ʱ�䣬 ��rc_cmd_time > 2000 Ϊ����2s
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
        //�ж�ң����2s����� ��ʱ��ʱ�䣬 ��̨ʹ��
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
        //�ж�ң����2s����� ��ʱ��ʱ�䣬 ������ʹ��
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
        //�ж�ң����20sУ׼ѡ��ʱ�䣬�޲���
        rc_action_flag = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        //�ж�ң����10s��У׼ѡ��ʱ���л���������Ƶ����
        //rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        //ң����10sǰ ��ʼ��������Ƶ����
        rc_cali_buzzer_start_on();
    }

    if (rc_action_flag != 0)
    {
        buzzer_time++;
    }
    //��������������
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
    {
        cali_buzzer_off();
    }
}




//У׼�������豸����ҪУ׼��Ư
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
            gyro_cali_disable_control(); //����ң�����Է������
            imu_start_buzzer();
            return 0;
        }
    }
    return 0;
}

//��ʼ��У׼�ṹ�����飬��ȡflashֵ�����δУ׼��ʹ��У׼����,ͬʱ��ʼ����ӦУ׼����
/**
  * @brief ������У׼ֵ��ʼ��
  * @param None
  * @retval None
  */
void SensorOffsetInit(void)
{
	BSP_FLASH_Read(ADDR_FLASH_SECTOR_11, AllDataUnion.data, sizeof(AllDataOffset__));
	
	//-.-��ԭ����C����ˮƽ���ޡ����������Ҵ˴�����ת���ǳ�֮���ҡ�����������
	gyro_cali.offset[0]=AllDataUnion.AllData.imu_offset.GYRO_Offset.x;
	gyro_cali.offset[1]=AllDataUnion.AllData.imu_offset.GYRO_Offset.y;
	gyro_cali.offset[2]=AllDataUnion.AllData.imu_offset.GYRO_Offset.z;
	
	cali_gyro_hook((uint32_t *)&gyro_cali, CALI_FUNC_CMD_INIT);
}


//����mpu6500��Ҫ���Ƶ����¶�
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
