
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



#define CALIBRATE_CONTROL_TIME 1 //校准任务函数运行的周期 为1ms

#define CALIBRATE_END_TIME 20000 //遥控器校准时长20s 超过20s需要重新操作
#define RC_CALI_BUZZER_MIDDLE_TIME 10000 //校准时，改变蜂鸣器频率成云台的高频声音，有助于提醒20s校准时间马上完毕
#define RCCALI_BUZZER_CYCLE_TIME 400  //校准选择时间20s，蜂鸣器断续发声周期时间
#define RC_CALI_BUZZER_PAUSE_TIME 200 //校准选择时间20s，蜂鸣器断续发声停声时间
//校准设备名
typedef enum
{
    CALI_HEAD,
    CALI_GIMBAL,
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    CALI_LIST_LENGHT,
} cali_id_e;



//陀螺仪，加速度计，磁力计通用校准数据
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


#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))  //陀螺仪校准函数
#define gyro_set_cali(cali_scale, cali_offset) INS_set_cali_gyro((cali_scale), (cali_offset)) //设置陀螺仪校准值
#define get_remote_ctrl_point_cali() get_remote_control_point() //获取遥控器的结构体指针
#define gyro_cali_disable_control() RC_unable()                 //陀螺仪校准时候掉线遥控器
#define cali_get_mcu_temperature() get_temprate() //获取stm32上的温度 作为IMU校准的环境温度
#define imu_start_buzzer() buzzer_on(95, 10000) //IMU元件校准的蜂鸣器的频率以及强度
#define cali_buzzer_off() buzzer_off() //关闭蜂鸣器
#define gimbal_start_buzzer() buzzer_on(31, 20000) //云台校准的蜂鸣器的频率以及强度
#define rc_cali_buzzer_start_on() imu_start_buzzer()

#define RC_CMD_LONG_TIME 2000 //遥控器使能校准的时间，即保持内八的时间
#define GYRO_CONST_MAX_TEMP 45.0f //陀螺仪控制恒温 最大控制温度
#define RC_CALI_VALUE_HOLE 600        //遥控器外八或者内八 阈值判定， 遥控器摇杆最大是 660 只要大于630 就认为到最大
#define CALI_FUNC_CMD_ON 1   //校准函数，使能校准
#define CALI_FUNC_CMD_INIT 0 //校准函数，传递flash中的校准参数
#define GYRO_CALIBRATE_TIME 10000 //陀螺仪校准的时间 10s

extern u8 ParaSavingFlag;
extern u8 CALIFLAG;
extern IMUSensor_OffSet__ IMUSensor_Offset;
extern AllDataUnion__ AllDataUnion; 
extern u8 ParamSavedFlag;
extern _PID_arg_st PID_arg[PIDGROUPLEN];
extern _PID_val_st PID_val[PIDGROUPLEN+3];

//校准任务
extern void calibrate_task(void *pvParameters);
int8_t get_control_temperate(void);
void SensorOffsetInit(void);
void ParametersSave(void);
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);
static void RC_cmd_to_calibrate(void);

extern _PID_arg_st PID_arg[PIDGROUPLEN];
extern _PID_val_st PID_val[PIDGROUPLEN+3];

#endif
