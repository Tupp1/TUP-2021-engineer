#include "CAN_Receive.h"
#include "system.h"
#include "stm32f4xx.h"
#include "rng.h"
#include "Timer_Send_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define get_mpu_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->x = ((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->y = ((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->z = ((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
    }
		
	
//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);      \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
		

	
//统一处理can接收函数
static void CAN_hook(CanRxMsg*rx_message);
static void CAN2_hook(CanRxMsg *rx_message);
//声明电机变量
motor_measure_t motor1_yaw,motor2_yaw;
motor_measure_t motor_chassis[4],motor_updown_l,motor_updown_r,motor_rotate,motor_gold;
motor_measure_t motor_lock,motor_snag_l,motor_snag_r;

static CanTxMsg GIMBAL_TxMessage;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif
//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook(&rx1_message); 
    }
}

//can2中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_slove(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif
//发送云台YAW两个6020电机数据
void CAN1_CMD_2FF(int16_t yaw1, int16_t yaw2, int16_t pitch, int16_t shoot)
{
    GIMBAL_TxMessage.StdId = CAN1_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw1 >> 8);
    GIMBAL_TxMessage.Data[1] = yaw1;
    GIMBAL_TxMessage.Data[2] = (yaw2 >> 8);
    GIMBAL_TxMessage.Data[3] = yaw2;
    GIMBAL_TxMessage.Data[4] = (pitch >> 8);
    GIMBAL_TxMessage.Data[5] = pitch;
    GIMBAL_TxMessage.Data[6] = (shoot >> 8);
    GIMBAL_TxMessage.Data[7] = shoot;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    //CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
		 xQueueSend(CAN1_Queue,&GIMBAL_TxMessage,1);//向队列中填充内容
#endif

}
//发送云台电磁锁升降3508电机数据
void CAN1_CMD_1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    GIMBAL_TxMessage.StdId = CAN1_LOCK_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (motor1 >> 8);
    GIMBAL_TxMessage.Data[1] = motor1;
    GIMBAL_TxMessage.Data[2] = (motor2 >> 8);
    GIMBAL_TxMessage.Data[3] = motor2;
    GIMBAL_TxMessage.Data[4] = (motor3 >> 8);
    GIMBAL_TxMessage.Data[5] = motor3;
    GIMBAL_TxMessage.Data[6] = (motor4 >> 8);
    GIMBAL_TxMessage.Data[7] = motor4;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    //CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
		 xQueueSend(CAN1_Queue,&GIMBAL_TxMessage,1);//向队列中填充内容
#endif

}
//发送两个障碍块电机控制命令
void CAN2_CMD_1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    GIMBAL_TxMessage.StdId = CAN2_UDTEANS_SNAG_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (motor1 >> 8);
    GIMBAL_TxMessage.Data[1] = motor1;
    GIMBAL_TxMessage.Data[2] = (motor2 >> 8);
    GIMBAL_TxMessage.Data[3] = motor2;
    GIMBAL_TxMessage.Data[4] = (motor3 >> 8);
    GIMBAL_TxMessage.Data[5] = motor3;
    GIMBAL_TxMessage.Data[6] = (motor4 >> 8);
    GIMBAL_TxMessage.Data[7] = motor4;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    //CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
		 xQueueSend(CAN2_Queue,&GIMBAL_TxMessage,1);//向队列中填充内容
#endif

}

void CAN_CMD_GIMBAL_STOP(void)
{
    GIMBAL_TxMessage.StdId = CAN2_UDTEANS_GOLD_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = 0;
    GIMBAL_TxMessage.Data[1] = 0;
    GIMBAL_TxMessage.Data[2] = 0;
    GIMBAL_TxMessage.Data[3] = 0;
    GIMBAL_TxMessage.Data[4] = 0;
    GIMBAL_TxMessage.Data[5] = 0;
    GIMBAL_TxMessage.Data[6] = 0;
    GIMBAL_TxMessage.Data[7] = 0;

    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
}


//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN2, &TxMessage);
}

//发送底盘电机控制命令
void CAN1_CMD_200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN1_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    xQueueSend(CAN1_Queue,&TxMessage,1);//向队列中填充内容
    //CAN_Transmit(CHASSIS_CAN, &TxMessage);
}


//发送有关金块拾取兑换的电机控制命令
void CAN2_CMD_200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN2_UDTEANS_GOLD_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    xQueueSend(CAN2_Queue,&TxMessage,1);//向队列中填充内容
    //CAN_Transmit(CHASSIS_CAN, &TxMessage);
}


//发送底盘电机控制命令
void CAN_CMD_CHASSIS_STOP(void)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN1_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;
   
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

/******************************can1*****************************/
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor1_Measure_Point(void)
{
    return &motor1_yaw;
}
const motor_measure_t *get_Yaw_Gimbal_Motor2_Measure_Point(void)
{
    return &motor2_yaw;
}

/******************************can1*****************************/

/******************************can2*****************************/
//抬升电机地址
const motor_measure_t *get_updown_l_Motor_Measure_Point(void)
{
    return &motor_updown_l;
}
const motor_measure_t *get_updown_r_Motor_Measure_Point(void)
{
    return &motor_updown_r;
}
//夹取机构翻转电机地址
const motor_measure_t *get_rotate_Motor_Measure_Point(void)
{
    return &motor_rotate;
}
//金块抬升电机地址
const motor_measure_t *get_gold_Motor_Measure_Point(void)
{
    return &motor_gold;
}
//电磁锁升降电机地址
const motor_measure_t *get_lock_Motor_Measure_Point(void)
{
    return &motor_lock;
}
//障碍块电机地址
const motor_measure_t *get_snag_L_Motor_Measure_Point(void)
{
    return &motor_snag_l;
}
const motor_measure_t *get_snag_R_Motor_Measure_Point(void)
{
    return &motor_snag_r;
}
/******************************can2*****************************/
		
//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN_hook(CanRxMsg *rx_message)
{
	switch (rx_message->StdId)
	{	
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
				static uint8_t i = 0;
				i = rx_message->StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_chassis[i], rx_message);
				break;
		}
		case CAN1_YAW_MOTOR1_ID:
		{
				get_gimbal_motor_measuer(&motor1_yaw, rx_message);
				break;
		}
		case CAN1_YAW_MOTOR2_ID:
		{
				get_gimbal_motor_measuer(&motor2_yaw, rx_message);
  			break;
		}
		case CAN1_3508_LOCK:
		{
		  	get_motor_measure(&motor_lock, rx_message);
			  break;
		}
		default:
		{
				break;
		}
	}
}
		
//统一处理can2中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN2_hook(CanRxMsg *rx_message)
{
	switch (rx_message->StdId)
	{
		case CAN2_3508_UpDown_L:
		{
		  	get_motor_measure(&motor_updown_l, rx_message);
			  break;
		}
		case CAN2_3508_UpDown_R:
		{
		  	get_motor_measure(&motor_updown_r, rx_message);
			  break;
		}
		case CAN2_3508_ROTATE:
		{
		  	get_motor_measure(&motor_rotate, rx_message);
			  break;
		}
		case CAN2_3508_GOLD:
		{
		  	get_motor_measure(&motor_gold, rx_message);
			  break;
		}
//		case CAN2_3508_SNAG_L:
//		{
//		  	get_motor_measure(&motor_snag_l, rx_message);
//			  break;
//		}
//		case CAN2_3508_SNAG_R:
//		{
//		  	get_motor_measure(&motor_snag_r, rx_message);
//			  break;
//		}
		default:
		{
				break;
		}
	}
}


