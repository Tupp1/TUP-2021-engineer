#ifndef __SYSTEM_H
#define	__SYSTEM_H


/* 步兵ID预编译,仅适用于调试,分区赛国赛需另外对应 */
#define    DEBUG_ID_ZERO    0       //白头翁
#define    DEBUG_ID_ONE     1		//黑无常
#define    DEBUG_ID_TWO     2		//4号金泉第一辆,黑无常
#define    DEBUG_ID_THREE   3		//3号金泉第二辆,白头翁
#define    DEBUG_ID_FOUR    4		//5号金泉第三辆，包青天


#define    INFANTRY_DEBUG_ID    DEBUG_ID_FOUR
/*-----------------------------------------*/

/* yaw上下预编译 */
#define    YAW_UP		(1)//6623
#define    YAW_DOWN    (-1)//6020

#define    YAW_POSITION    YAW_DOWN
/*-----------------------------------------*/

/* 打符摄像头预编译 */
#define    BUFF_CAM_CHAS	0
#define    BUFF_CAM_GIMB	1

#define    BUFF_CAM_TYPE	BUFF_CAM_GIMB
/*-----------------------------------------*/


#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "stdint.h"

#include "delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"



#define abs(x) ((x)>0? (x):(-(x)))


/* for FreeRTOS*/
#define    TIME_STAMP_1MS        1
#define    TIME_STAMP_2MS        2
#define    TIME_STAMP_4MS        4
#define    TIME_STAMP_10MS      10
#define    TIME_STAMP_20MS      20
#define    TIME_STAMP_30MS      30
#define    TIME_STAMP_40MS      40
#define    TIME_STAMP_50MS      50
#define    TIME_STAMP_60MS      60
#define    TIME_STAMP_80MS      80
#define    TIME_STAMP_100MS    100
#define    TIME_STAMP_150MS    150
#define    TIME_STAMP_200MS    200
#define    TIME_STAMP_250MS    250
#define    TIME_STAMP_300MS    300
#define    TIME_STAMP_400MS    400
#define    TIME_STAMP_500MS    500
#define    TIME_STAMP_1000MS  1000
#define    TIME_STAMP_2000MS  2000
#define    TIME_STAMP_10S     10000

#define    FALSE    0
#define    TRUE     1



#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入




/*
	任务堆栈大小
	1字 = 4字节
*/
#define STK_SIZE_64 	64 				//任务堆栈64字
#define STK_SIZE_128 	128 			//任务堆栈128字
#define STK_SIZE_256 	256 			//任务堆栈256字
#define STK_SIZE_512 	512 			//任务堆栈512字


#define KP 0
#define KI 1
#define KD 2

#define OUTER 0
#define INNER 1

int constrain(int amt, int low, int high);
float constrain_float(float amt, float low, float high);
int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high);
int32_t constrain_int32_t(int32_t amt, int32_t low, int32_t high);



void SYSTEM_InitPeripheral(void);
void System_Init(void);


#endif 



