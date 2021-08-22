
#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "main.h"
#include "adc.h"
#include "INS_task.h"
#include "rc.h"
#include "pid.h"
#include "buzzer.h"
#include "stm32f4xx.h"
//#include "parameters.h"

#include "sys_config.h"
#define PIDGROUPLEN 8

#define Chassis_Rot_PID_arg ( *PID_arg )
#define Chassis_Vec_PID_arg ( *(PID_arg+1) )
#define GimbalPitch_Pos_PID_arg ( *(PID_arg+2) )
#define GimbalPitch_Vec_PID_arg ( *(PID_arg+3) )
#define GimbalYaw_Pos_PID_arg ( *(PID_arg+4) )
#define GimbalYaw_Vec_PID_arg ( *(PID_arg+5) )
#define Slibing_Pos_PID_arg ( *(PID_arg+6) )
#define Slibing_Vec_PID_arg ( *(PID_arg+7) )

#define Chassis_Rot_PID_val ( *PID_val )
#define Chassis_Vec_PID_val1 ( *(PID_val+1) )
#define Chassis_Vec_PID_val2 ( *(PID_val+2) )
#define Chassis_Vec_PID_val3 ( *(PID_val+3) )
#define Chassis_Vec_PID_val4 ( *(PID_val+4) )
#define GimbalPitch_Pos_PID_val ( *(PID_val+5) )
#define GimbalPitch_Vec_PID_val ( *(PID_val+6) )
#define GimbalYaw_Pos_PID_val ( *(PID_val+7) )
#define GimbalYaw_Vec_PID_val ( *(PID_val+8) )
#define Slibing_Pos_PID_val ( *(PID_val+9) )
#define Slibing_Vec_PID_val ( *(PID_val+10) )



#define CALIBRATE_CONTROL_TIME 1 //У׼���������е����� Ϊ1ms

#define CALIBRATE_END_TIME 20000 //ң����У׼ʱ��20s ����20s��Ҫ���²���
#define RC_CALI_BUZZER_MIDDLE_TIME 10000 //У׼ʱ���ı������Ƶ�ʳ���̨�ĸ�Ƶ����������������20sУ׼ʱ���������
#define RCCALI_BUZZER_CYCLE_TIME 400  //У׼ѡ��ʱ��20s��������������������ʱ��
#define RC_CALI_BUZZER_PAUSE_TIME 200 //У׼ѡ��ʱ��20s����������������ͣ��ʱ��
//У׼�豸��
typedef enum
{
    CALI_HEAD,
    CALI_GIMBAL,
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    CALI_LIST_LENGHT,
} cali_id_e;



//�����ǣ����ٶȼƣ�������ͨ��У׼����
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

typedef __packed struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;

typedef __packed struct 
{
	xyz_f_t GYRO_Offset;
	xyz_f_t ACCER_Offset;
	xyz_f_t MAG_Offset;
}IMUSensor_OffSet__;

typedef __packed struct
{
	u8 savedflag;
	IMUSensor_OffSet__ imu_offset;
	uint16_t GimbalPitchOffset;
	uint16_t GimbalYawOffset;
	_PID_arg_st PID_ARG[PIDGROUPLEN];
}AllDataOffset__;


typedef union
{
	u8 data[sizeof(AllDataOffset__)];
	AllDataOffset__ AllData;
}AllDataUnion__;


#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))  //������У׼����
#define gyro_set_cali(cali_scale, cali_offset) INS_set_cali_gyro((cali_scale), (cali_offset)) //����������У׼ֵ
#define get_remote_ctrl_point_cali() get_remote_control_point() //��ȡң�����Ľṹ��ָ��
#define gyro_cali_disable_control() RC_unable()                 //������У׼ʱ�����ң����
#define cali_get_mcu_temperature() get_temprate() //��ȡstm32�ϵ��¶� ��ΪIMUУ׼�Ļ����¶�
#define imu_start_buzzer() buzzer_on(95, 10000) //IMUԪ��У׼�ķ�������Ƶ���Լ�ǿ��
#define cali_buzzer_off() buzzer_off() //�رշ�����
#define gimbal_start_buzzer() buzzer_on(31, 20000) //��̨У׼�ķ�������Ƶ���Լ�ǿ��
#define rc_cali_buzzer_start_on() imu_start_buzzer()

#define RC_CMD_LONG_TIME 2000 //ң����ʹ��У׼��ʱ�䣬�������ڰ˵�ʱ��
#define GYRO_CONST_MAX_TEMP 45.0f //�����ǿ��ƺ��� �������¶�
#define RC_CALI_VALUE_HOLE 600        //ң������˻����ڰ� ��ֵ�ж��� ң����ҡ������� 660 ֻҪ����630 ����Ϊ�����
#define CALI_FUNC_CMD_ON 1   //У׼������ʹ��У׼
#define CALI_FUNC_CMD_INIT 0 //У׼����������flash�е�У׼����
#define GYRO_CALIBRATE_TIME 10000 //������У׼��ʱ�� 10s

extern u8 ParaSavingFlag;
extern u8 CALIFLAG;
extern IMUSensor_OffSet__ IMUSensor_Offset;
extern AllDataUnion__ AllDataUnion; 
extern u8 ParamSavedFlag;
extern _PID_arg_st PID_arg[PIDGROUPLEN];
extern _PID_val_st PID_val[PIDGROUPLEN+3];

//У׼����
extern void calibrate_task(void *pvParameters);
int8_t get_control_temperate(void);
void SensorOffsetInit(void);
void ParametersSave(void);
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);
static void RC_cmd_to_calibrate(void);

extern _PID_arg_st PID_arg[PIDGROUPLEN];
extern _PID_val_st PID_val[PIDGROUPLEN+3];

#endif
