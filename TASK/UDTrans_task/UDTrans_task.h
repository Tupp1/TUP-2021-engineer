#ifndef _UDTRANS_TASK_H
#define _UDTRANS_TASK_H

#include "system.h"
#include "pid.h"
#include "remote.h"
#include "CAN_receive.h"
#define UDTRANS_TASK_INIT_TIME 200
#define TIME_STAMP_1MS    1
#define TIME_STAMP_2MS    2
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f     
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define UPTRANS_X_CHANNEL 1 //前后的遥控器通道号码
#define UPTRANS_Y_CHANNEL 0 //左右的遥控器通道号码
#define UPTRANS_W_CHANNEL 3 //前后的遥控器通道号码
#define UPTRANS_RC_DEADLINE 10

#define UPTRANS_VX_RC_SEN 0.0065f  //遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define UPTRANS_VY_RC_SEN 0.0055f  //遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例

#define UPDOWNMOTOR_SPEED_PID_KP 1000.0f       
#define UPDOWNMOTOR_SPEED_PID_KI 0
#define UPDOWNMOTOR_SPEED_PID_KD 0
#define UPDOWNMOTOR_SPEED_PID_MAX_OUT 11000.0f   //8000
#define UPDOWNMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define UPDOWNMOTOR_ANGLE_PID_KP -0.0009f
#define UPDOWNMOTOR_ANGLE_PID_KI 0
#define UPDOWNMOTOR_ANGLE_PID_KD 0
#define UPDOWNMOTOR_ANGLE_PID_MAX_OUT  10.0f
#define UPDOWNMOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define ROTATEMOTOR_SPEED_PID_KP 1200.0f         //900
#define ROTATEMOTOR_SPEED_PID_KI 0
#define ROTATEMOTOR_SPEED_PID_KD 0
#define ROTATEMOTOR_SPEED_PID_MAX_OUT 15000.0f   //8000
#define ROTATEMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define ROTATEMOTOR_ANGLE_PID_KP -0.001f         //-0.0009f
#define ROTATEMOTOR_ANGLE_PID_KI 0
#define ROTATEMOTOR_ANGLE_PID_KD 0
#define ROTATEMOTOR_ANGLE_PID_MAX_OUT  13.0f     //10
#define ROTATEMOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define GOLDMOTOR_SPEED_PID_KP 1100.0f           //900
#define GOLDMOTOR_SPEED_PID_KI 0
#define GOLDMOTOR_SPEED_PID_KD 0
#define GOLDMOTOR_SPEED_PID_MAX_OUT 11000.0f     //8000
#define GOLDMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define GOLDMOTOR_ANGLE_PID_KP -0.0004f          //-0.0009f 
#define GOLDMOTOR_ANGLE_PID_KI 0
#define GOLDMOTOR_ANGLE_PID_KD 0
#define GOLDMOTOR_ANGLE_PID_MAX_OUT  6.0f        //10
#define GOLDMOTOR_ANGLE_PID_MAX_IOUT 3.0f

#define SNAGMOTOR_SPEED_PID_KP 900.0f
#define SNAGMOTOR_SPEED_PID_KI 0
#define SNAGMOTOR_SPEED_PID_KD 0
#define SNAGMOTOR_SPEED_PID_MAX_OUT 11000.0f     //8000
#define SNAGMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define SNAGMOTOR_ANGLE_PID_KP -0.0009f
#define SNAGMOTOR_ANGLE_PID_KI 0
#define SNAGMOTOR_ANGLE_PID_KD 0
#define SNAGMOTOR_ANGLE_PID_MAX_OUT  10.0f
#define SNAGMOTOR_ANGLE_PID_MAX_IOUT 5.0f

typedef enum
{	
	UPTRANS_INIT,          //初始化模式
	UPTRANS_MANUAL,        //手动模式
	UPTRANS_COLLECT_TINY,  //小资源岛取金块模式
  UPTRANS_COLLECT_HUGE,  //大资源岛取金块模式
	UPTRANS_CONVERSION,    //兑换金币模式
	UPTRANS_SNAG,          //电机除障模式  目前没有用到 舍弃
	UPTRANS_SUPPLY,        //补给模式
	UPTRANS_STOP,          //暂停模式
	UPTRANS_RECOVER,       //恢复模式
	UPTRANS_NOFORCE,       //无力模式
	UPTRANS_HELP,          //救援模式      目前没有用到 舍弃
} uptrans_behaviour_e;


typedef struct
{
    const motor_measure_t *uptrans_motor_measure;
	
    PidTypeDef motor_angle_pid;
    PidTypeDef motor_speed_pid;
	
    fp32 motor_ecd_sum;     
	  fp32 motor_angle;
    fp32 motor_angle_set;   
	  fp32 set_ramp_angle;
	
    fp32 motor_speed_set;
    fp32 motor_speed;
	
    fp32 current_set;
    int16_t given_current;
	
	  fp32 min_angle_limit;		
	  fp32 mid_angle_limit;
	  fp32 max_angle_limit;
	  fp32 MAX_angle_limit;
	
		fp32 init_angle_set;  //电磁锁抬升电机用到
		
} UPTrans_Motor_t;

typedef struct
{
	  const RC_ctrl_t *udtrans_rc_ctrl;
	  RC_ctrl_time_t RC_udtrans_ctrl_time;
	
	  uptrans_behaviour_e uptrans_behaviour;
	  uptrans_behaviour_e last_uptrans_behaviour;
	
    UPTrans_Motor_t upmotor_L;
	  UPTrans_Motor_t upmotor_R;
	  UPTrans_Motor_t rotatemotor;
		UPTrans_Motor_t goldmotor;
		UPTrans_Motor_t snagmotor_L;
		UPTrans_Motor_t snagmotor_R;
	
} UDTrans_Control_t;

void UDTrans_task(void *pvParameters);

void Clamp_on(void);                  //夹取装置松开
void Clamp_close(void);               //夹取装置夹紧
void stretch_on(void);                //夹取装置伸出
void stretch_close(void);             //夹取装置收回
void rescue_on(void);                 //救援伸出
void rescue_close(void);              //救援收回
void rotary_actuator_on(void);        //除障旋转气缸打开
void rotary_actuator_close(void);     //除障旋转气缸关闭
void supply1_on(void);                //补给1开启
void supply1_close(void);             //补给1关闭
void supply1_disable(void);           //补给1失能 
void supply2_on(void);                //补给2开启
void supply2_close(void);             //补给2关闭
void supply2_disable(void);           //补给2失能  
void camera_up(void);                 //图传上
void camera_mid(void);                //图传中
void camera_down(void);               //图传下
void camera_screen(void);             //图传显示屏

//气缸状态间隔时间
#define COLLECT_TIME_PART    300             
#define CONVERSION_TIME_PART 400  

//电机速度
#define HAND_FAST       150
#define HAND_SLOW       70
#define TRANS_UP_SPEED  300
#define GOLD_UP_SPEED   200

//大资源岛金块拾取前抬升的高度
#define COLLECT_HUGE_UP_HEIGHT_1    105000
//大资源岛金块拾取夹紧后抬升上升高度
#define COLLECT_HUGE_UP_HEIGHT_2    200000
//大资源岛金块拾取抓手角度
#define COLLECT_HUGE_HAND           80000

//金块拾取完毕后抬升保持的高度
#define COLLECT_UP_GOLD_OVER        180000
//金块拾取取完毕后抓手保持的角度
#define COLLECT_HAND_PICKUP_HEIGHT  40000


//金块兑换抓手扫码高度
#define CONVERSION_HAND_SCAN        60000
//金块兑换抬升到达手动调节高度
#define CONVERSION_UP_ADJUST        300000
//金块兑换抬升下降取金块的高度
#define CONVERSION_UP_PICKUP1       210000
#define CONVERSION_UP_PICKUP2       90000

 
//拾取兑换自动模式
#define COLLECT_CONVERSION_AUTO
	
#endif


