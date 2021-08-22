#ifndef CANTASK_H

#define CANTASK_H
#include "main.h"
#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1
#define MPU_CAN CAN1
/* CAN send and receive ID */
typedef enum
{
    CAN1_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,	
	
	  CAN1_GIMBAL_ALL_ID  = 0x2FF,
	  CAN1_YAW_MOTOR1_ID  = 0x209,
	  CAN1_YAW_MOTOR2_ID  = 0x20A,

	  CAN1_LOCK_ALL_ID   = 0x1FF,	
		CAN1_3508_LOCK     = 0x205,	
	
} can_msg_id_e;


typedef enum
{	
	  CAN2_UDTEANS_GOLD_ALL_ID = 0x200,
		CAN2_3508_UpDown_L = 0x201,
	  CAN2_3508_UpDown_R = 0x202,
	  CAN2_3508_ROTATE   = 0x203,
	  CAN2_3508_GOLD     = 0x204,
	
		CAN2_UDTEANS_SNAG_ALL_ID = 0x1FF,
//		CAN2_3508_SNAG_L   = 0x205,
//		CAN2_3508_SNAG_R   = 0x206,
	
} can2_msg_id_e;


//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
} motor_measure_t;


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} mpu_angle_gyro_t;


extern void CAN_CMD_CHASSIS_RESET_ID(void);

//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor1_Measure_Point(void);  //can1 0x209
extern const motor_measure_t *get_Yaw_Gimbal_Motor2_Measure_Point(void);  //can1 0x20A
const motor_measure_t *get_lock_Motor_Measure_Point(void);                //can1 0x205

//指向工程任务电机地址
const motor_measure_t *get_updown_l_Motor_Measure_Point(void);    //can2 0x201
const motor_measure_t *get_updown_r_Motor_Measure_Point(void);    //can2 0x202
const motor_measure_t *get_rotate_Motor_Measure_Point(void);      //can2 0x203
const motor_measure_t *get_gold_Motor_Measure_Point(void);        //can2 0x204

const motor_measure_t *get_snag_L_Motor_Measure_Point(void);      //can2 0x205
const motor_measure_t *get_snag_R_Motor_Measure_Point(void);      //can2 0x206

extern void CAN1_CMD_200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN1_CMD_1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN1_CMD_2FF(int16_t yaw1, int16_t yaw2, int16_t shoot, int16_t rev);
extern void CAN2_CMD_200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN2_CMD_1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern motor_measure_t motor_yaw,motor_chassis[4];
extern mpu_angle_gyro_t mpu6500_angle,mpu6500_gyro;

extern void CAN_CMD_GIMBAL_STOP(void);
extern void CAN_CMD_CHASSIS_STOP(void);

extern int16_t angle_format_change(int16_t angle,int16_t last_angle,fp32 rate);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
