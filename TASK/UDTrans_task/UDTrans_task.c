#include "UDTrans_task.h"
#include "gimbal_behaviour.h"
#include "system.h"
#include "control.h"
#include "user_lib.h"
#include "ramp.h"
#include "User_Task.h"
#include "calibrate_Task.h"
#include "chassis_task.h"
#include "io.h"
#include "Gimbal_Task.h"

//死区限制
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
/***********************************  工程初始化函数  ************************************/
static void UDTRANS_Init(UDTrans_Control_t *udtrans_init);
/***********************************  工程初始化函数  ************************************/
		
/**********************************  任务内部循环函数  ***********************************/
static void UPTrans_Set_Mode(UDTrans_Control_t *udtrans_set_mode);
static void UPTrans_Mode_Change_Control_Transit(UDTrans_Control_t *udtrans_mode_change);
static void UDTrans_Feedback_Update(UDTrans_Control_t *UDTrans_Feedback);
static void UPTrans_control_loop(UDTrans_Control_t *revolver_control);
/**********************************  任务内部循环函数  ***********************************/

/************************************  模式处理函数  *************************************/
static void UPTrans_init_control(UDTrans_Control_t *udtrans_init_control);
static void UPTrans_manual_control(UDTrans_Control_t *udtrans_manual_control);
static void UPTrans_stop_control(UDTrans_Control_t *udtrans_stop_control);
static void UPTrans_recover_control(UDTrans_Control_t *udtrans_recover_control);
static void UPTrans_help_control(UDTrans_Control_t *udtrans_help_control);	
static void UPTrans_snag_control(UDTrans_Control_t *udtrans_snag_control);		
static void UPTrans_supply_control(UDTrans_Control_t *udtrans_supply_control);	
static void UPTrans_collect_tiny_control(UDTrans_Control_t *udtrans_collect_tiny_control);
static void UPTrans_collect_huge_control(UDTrans_Control_t *udtrans_collect_huge_control);
static void UPTrans_conversion_control(UDTrans_Control_t *udtrans_conversion_control);
/************************************  模式处理函数  *************************************/

/**************************************  辅助函数  ***************************************/
static void UPTRANS_UpdateMotorAngleSum(UPTrans_Motor_t *UPTrans_Motor);
static fp32 fp32_constrain_upmotor_angle(UPTrans_Motor_t* motor);
static fp32 fp32_constrain_upmotor_ramp(UPTrans_Motor_t* motor);
/**************************************  辅助函数  ***************************************/

/**************************************  辅助变量  ****************************************/
static uint8_t supply1_step = 1;      //补给1控制标志位   
static uint8_t supply2_step = 1;	    //补给2控制标志位  
static uint8_t help_step    = 1;      //救援伸缩气缸控制标志位   
static uint8_t snag_step    = 1;	    //除障旋转气缸控制标志位
static uint8_t stop_step    = 1;      //停止步骤，只有在解除停止状态时为1
static uint8_t trans_step   = 1;	    //工程大部分任务步骤，受其他模式影响
static fp32    trans_time   = 0;	    //工程大部分任务内部计时，受其他模式影响
static uint8_t gold_number  = 0;      //金块数量，在金块拾取和金块兑换模式中交接数量
static uint8_t collect_gold = 0;      //金块拾取任务内部计数
static uint8_t conversion_gold  = 0;  //金块兑换任务内部计数
static uint8_t conversion_mouse = 1;  //金块兑换中鼠标右键标志位，控制伸缩气缸
static uint8_t press_L_step = 0;      //鼠标左键标志位
static uint8_t press_R_step = 0;      //鼠标左键标志位
static uint8_t camera_place = 1;      //图传位置标志位
/**************************************  辅助变量  ****************************************/
	
/******************************  工程气缸和舵机状态标志位  ********************************/
uint8_t supply1_flag = 0;             //补给1标志位    0为关，1为开
uint8_t supply2_flag = 0;	            //补给2标志位    0为关，1为开
uint8_t help_flag    = 0;             //救援标志位     0为关，1为开
uint8_t snag_flag    = 0;	            //除障标志位     0为关，1为开
uint8_t clamp_flag   = 0;             //夹取标志位     0为关，1为开
uint8_t stretch_flag = 0;	            //伸缩标志位     0为关，1为开
/******************************  工程气缸和舵机状态标志位  ********************************/

/**************************************  任务数据  ****************************************/
extern Gimbal_Control_t gimbal_control;

UDTrans_Control_t UDTrans_Control;
/**************************************  任务数据  ****************************************/

void UDTrans_task(void *pvParameters)
{
	portTickType currentTime;	
	vTaskDelay(UDTRANS_TASK_INIT_TIME);
	UDTRANS_Init(&UDTrans_Control);
	for(;;)
	{	
		currentTime = xTaskGetTickCount(); //当前系统时间
		//设置模式
		UPTrans_Set_Mode(&UDTrans_Control);
		//模式切换数据处理
		UPTrans_Mode_Change_Control_Transit(&UDTrans_Control);
		//数据更新
	  UDTrans_Feedback_Update(&UDTrans_Control);
		//模式控制PID计算
		UPTrans_control_loop(&UDTrans_Control);
	  //CAN1发送  标识符：1FF  抬升电机*2 夹取旋转电机 金块抬升电机
		CAN2_CMD_200(UDTrans_Control.upmotor_L.given_current   , UDTrans_Control.upmotor_R.given_current,
		             UDTrans_Control.rotatemotor.given_current , UDTrans_Control.goldmotor.given_current);
		//CAN2发送  标识符：200  障碍块电机 
//    CAN2_CMD_1FF(UDTrans_Control.snagmotor_L.given_current , UDTrans_Control.snagmotor_R.given_current,0,0);             		 
		//绝对延时 1MS
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS); 
	}
}

static void UDTRANS_Init(UDTrans_Control_t *udtrans_init)
{
   //抬升电机pid参数设置
	 const fp32 updownmotor_speed_pid[3] = {UPDOWNMOTOR_SPEED_PID_KP, UPDOWNMOTOR_SPEED_PID_KI, UPDOWNMOTOR_SPEED_PID_KD};
	 const fp32 updownmotor_angle_pid[3] = {UPDOWNMOTOR_ANGLE_PID_KP, UPDOWNMOTOR_ANGLE_PID_KI, UPDOWNMOTOR_ANGLE_PID_KD};
	 //夹取翻转装置pid参数设置
	 const fp32 rotatemotor_speed_pid[3] = {ROTATEMOTOR_SPEED_PID_KP, ROTATEMOTOR_SPEED_PID_KI, ROTATEMOTOR_SPEED_PID_KD};
	 const fp32 rotatemotor_angle_pid[3] = {ROTATEMOTOR_ANGLE_PID_KP, ROTATEMOTOR_ANGLE_PID_KI, ROTATEMOTOR_ANGLE_PID_KD};
	 //夹取翻转装置pid参数设置
	 const fp32 goldmotor_speed_pid[3]   = {GOLDMOTOR_SPEED_PID_KP, GOLDMOTOR_SPEED_PID_KI, GOLDMOTOR_SPEED_PID_KD};
	 const fp32 goldmotor_angle_pid[3]   = {GOLDMOTOR_ANGLE_PID_KP, GOLDMOTOR_ANGLE_PID_KI, GOLDMOTOR_ANGLE_PID_KD};
	 //夹取翻转装置pid参数设置
	 const fp32 snagmotor_speed_pid[3]   = {SNAGMOTOR_SPEED_PID_KP, SNAGMOTOR_SPEED_PID_KI, SNAGMOTOR_SPEED_PID_KD};
	 const fp32 snagmotor_angle_pid[3]   = {SNAGMOTOR_ANGLE_PID_KP, SNAGMOTOR_ANGLE_PID_KI, SNAGMOTOR_ANGLE_PID_KD};
	 
	 //抬升电机PID初始化
	 PID_Init(&(udtrans_init->upmotor_R.motor_speed_pid), PID_POSITION, updownmotor_speed_pid, //速度环
	            UPDOWNMOTOR_SPEED_PID_MAX_OUT, UPDOWNMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->upmotor_R.motor_angle_pid), PID_POSITION, updownmotor_angle_pid, //角度环
	            UPDOWNMOTOR_ANGLE_PID_MAX_OUT, UPDOWNMOTOR_ANGLE_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->upmotor_L.motor_speed_pid), PID_POSITION, updownmotor_speed_pid,  
	            UPDOWNMOTOR_SPEED_PID_MAX_OUT, UPDOWNMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->upmotor_L.motor_angle_pid), PID_POSITION, updownmotor_angle_pid, 
	            UPDOWNMOTOR_ANGLE_PID_MAX_OUT, UPDOWNMOTOR_ANGLE_PID_MAX_IOUT); 
	 
	 //夹取翻转电机PID初始化
	 PID_Init(&(udtrans_init->rotatemotor.motor_speed_pid), PID_POSITION, rotatemotor_speed_pid, 
	            ROTATEMOTOR_SPEED_PID_MAX_OUT, ROTATEMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->rotatemotor.motor_angle_pid), PID_POSITION, rotatemotor_angle_pid, 
	            ROTATEMOTOR_ANGLE_PID_MAX_OUT, ROTATEMOTOR_ANGLE_PID_MAX_IOUT);
	 
	 //金块抬升电机PID初始化 
	 PID_Init(&(udtrans_init->goldmotor.motor_speed_pid), PID_POSITION, goldmotor_speed_pid,  
	            GOLDMOTOR_SPEED_PID_MAX_OUT, GOLDMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->goldmotor.motor_angle_pid), PID_POSITION, goldmotor_angle_pid, 
	            GOLDMOTOR_ANGLE_PID_MAX_OUT, GOLDMOTOR_ANGLE_PID_MAX_IOUT); 

	 //障碍块电机PID初始化 
	 PID_Init(&(udtrans_init->snagmotor_L.motor_speed_pid), PID_POSITION, snagmotor_speed_pid,  
	            SNAGMOTOR_SPEED_PID_MAX_OUT, SNAGMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->snagmotor_L.motor_angle_pid), PID_POSITION, snagmotor_angle_pid, 
	            SNAGMOTOR_ANGLE_PID_MAX_OUT, SNAGMOTOR_ANGLE_PID_MAX_IOUT); 
	 PID_Init(&(udtrans_init->snagmotor_R.motor_speed_pid), PID_POSITION, snagmotor_speed_pid,  
	            SNAGMOTOR_SPEED_PID_MAX_OUT, SNAGMOTOR_SPEED_PID_MAX_IOUT);
	 PID_Init(&(udtrans_init->snagmotor_R.motor_angle_pid), PID_POSITION, snagmotor_angle_pid, 
	            SNAGMOTOR_ANGLE_PID_MAX_OUT, SNAGMOTOR_ANGLE_PID_MAX_IOUT); 
	  
   //遥控器指针
   udtrans_init->udtrans_rc_ctrl = get_remote_control_point();
	 
	 //电机指针
   udtrans_init->upmotor_L.uptrans_motor_measure   = get_updown_l_Motor_Measure_Point();
	 udtrans_init->upmotor_R.uptrans_motor_measure   = get_updown_r_Motor_Measure_Point();
	 udtrans_init->rotatemotor.uptrans_motor_measure = get_rotate_Motor_Measure_Point();
   udtrans_init->goldmotor.uptrans_motor_measure   = get_gold_Motor_Measure_Point();
   udtrans_init->snagmotor_L.uptrans_motor_measure = get_snag_L_Motor_Measure_Point();
   udtrans_init->snagmotor_R.uptrans_motor_measure = get_snag_R_Motor_Measure_Point();
	 
	 //开始为无力模式
	 udtrans_init->uptrans_behaviour = UPTRANS_NOFORCE;
	 
	 //电机初始角度
	 udtrans_init->upmotor_L.motor_angle = 0;
	 udtrans_init->upmotor_R.motor_angle = 0;
	 udtrans_init->rotatemotor.motor_angle = 0;
	 udtrans_init->goldmotor.motor_angle = 0;
	 udtrans_init->snagmotor_L.motor_angle = 0;
	 udtrans_init->snagmotor_R.motor_angle = 0;
	 
	 //电机角度设置为初始角度（归零）
	 udtrans_init->upmotor_R.motor_angle_set = udtrans_init->upmotor_R.motor_angle;  
	 udtrans_init->upmotor_L.motor_angle_set = udtrans_init->upmotor_L.motor_angle; 
	 udtrans_init->rotatemotor.motor_angle_set = udtrans_init->rotatemotor.motor_angle;	
	 udtrans_init->goldmotor.motor_angle_set = udtrans_init->goldmotor.motor_angle;	
	 udtrans_init->snagmotor_L.motor_angle_set = udtrans_init->snagmotor_L.motor_angle;	
	 udtrans_init->snagmotor_R.motor_angle_set = udtrans_init->snagmotor_R.motor_angle;	
 
   //电机极限位置赋值   需要调整！！！！！！！！！！！
   udtrans_init->upmotor_L.min_angle_limit = 0;        //ID:1  逆时针
   udtrans_init->upmotor_L.mid_angle_limit = 150000;   
	 udtrans_init->upmotor_L.max_angle_limit = 300000;   //小资源岛高度 305000 实际最高位置 再加一点 
	 udtrans_init->upmotor_L.MAX_angle_limit = udtrans_init->upmotor_L.max_angle_limit + 5000;
	 udtrans_init->upmotor_R.min_angle_limit = -300000;
   udtrans_init->upmotor_R.mid_angle_limit = -150000;
	 udtrans_init->upmotor_R.max_angle_limit = 0;
	 udtrans_init->upmotor_R.MAX_angle_limit = udtrans_init->upmotor_R.min_angle_limit - 5000;
	 
	 udtrans_init->rotatemotor.min_angle_limit = 0;      //ID:2
   udtrans_init->rotatemotor.mid_angle_limit = 50000;
   udtrans_init->rotatemotor.max_angle_limit = 77000;
	 udtrans_init->rotatemotor.MAX_angle_limit = 83400;

	 udtrans_init->goldmotor.min_angle_limit = 0;        //ID:4
	 udtrans_init->goldmotor.mid_angle_limit = 80000;    //存两个时升起能够
   udtrans_init->goldmotor.max_angle_limit = 190000;   //最高高度，接金块高度
	 udtrans_init->goldmotor.MAX_angle_limit = udtrans_init->goldmotor.max_angle_limit;

	 udtrans_init->snagmotor_L.min_angle_limit = 0;      //ID:6  逆时针
	 udtrans_init->snagmotor_L.mid_angle_limit = 40000;
   udtrans_init->snagmotor_L.max_angle_limit = 73000; 
	 udtrans_init->snagmotor_L.MAX_angle_limit = udtrans_init->snagmotor_L.max_angle_limit + 20000;
	 udtrans_init->snagmotor_R.min_angle_limit = -73000; //ID:7
	 udtrans_init->snagmotor_R.mid_angle_limit = -40000;
   udtrans_init->snagmotor_R.max_angle_limit = 0; 
	 udtrans_init->snagmotor_R.MAX_angle_limit = udtrans_init->snagmotor_R.max_angle_limit - 20000;
	  
	 //气缸初始化状态
	 Clamp_on();                  //夹取装置松开
	 stretch_close();             //夹取装置收回 
	 rescue_close();              //救援收回 
	 rotary_actuator_close();     //除障关闭 
	 //舵机初始化状态
   supply1_close();             //补给1关闭 
	 supply2_close();             //补给2关闭 
 
	 UDTrans_Feedback_Update(udtrans_init); 
}

static void UPTrans_Set_Mode(UDTrans_Control_t *udtrans_set_mode)
{
	if (udtrans_set_mode == NULL)
	{
			return;
	}
	
	udtrans_set_mode->last_uptrans_behaviour = udtrans_set_mode->uptrans_behaviour; 

	
	/*************************************图传位置转换*****************************************/	
				
	//R键更改图传位置
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R )  
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time == RC_KEY_LONG_TIME)
			{						
				switch (camera_place)
				{
					case 0:
					{
						camera_mid();             //图传中
						camera_place = 1;
						break;
					}
					case 1:
					{
						camera_down();            //图传下
						camera_place = 2;
						break;
					}
					case 2:
					{
						camera_screen();          //图传显示屏位置
						camera_place = 3;
						break;
					}
					case 3:
					{
						camera_up();              //图传上
						camera_place = 0;
						break;
					}					
				}				
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyR_time = 0;
	}
	/*************************************图传位置转换*****************************************/	
	
	
	/*************************************遥控器没打开一直无力*********************************/
	if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
	{
		udtrans_set_mode->uptrans_behaviour=UPTRANS_NOFORCE;
	}
	/*************************************遥控器没打开一直无力*********************************/

	
	/**************************************模式优先级转换**************************************/
	/************************************无力模式优先级最高************************************/
	
	//无力模式优先级最高
	//系统在初始化，为无力模式
	if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
	{
	  udtrans_set_mode->uptrans_behaviour = UPTRANS_NOFORCE;
		return ;  //直接返回
	}
		
	//遥控器右通道在下，为无力模式
	if(switch_is_down(udtrans_set_mode->udtrans_rc_ctrl->rc.s[0]))
	{
	  udtrans_set_mode->uptrans_behaviour = UPTRANS_NOFORCE;
		return ;  //直接返回
	}
	
	/************************************无力模式优先级最高************************************/
	
	
	/***********************************初始化模式优先级第二***********************************/

	//初始化模式优先级第二
	static fp32 init_mode_time = 0;
	if(udtrans_set_mode->uptrans_behaviour == UPTRANS_INIT)
	{
		init_mode_time++;
    if(init_mode_time == 80000)
		{
			//计时归零
		  init_mode_time = 0;
			//切换为停止模式
	  	udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;
		}	
		else
		{
			return ;   //直接返回		
		}
	}
	else
	{
	  //计时归零
	  init_mode_time = 0;		
	}
	/***********************************初始化模式优先级第二***********************************/

	
	/******************************大陀螺模式下为停止优先级第三********************************/
	
	//底盘云台大陀螺模式下为停止模式，优先级第三			
	if(gimbal_control.gimbal_motor_mode == GIMBAL_SPIN)
	{
		rescue_close();           //救援气缸收回，防止大陀螺时气缸碰撞
		help_step = 1;            //救援步骤重置为1，防止下次按F按键时救援不伸出
		rotary_actuator_close();  //除障气缸收回，防止大陀螺时气缸碰撞
		snag_step = 1;            //除障步骤重置为1，防止下次按B按键时除障不伸出
		udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;
		return ;   //直接返回
	}	

	/******************************大陀螺模式下为停止优先级第三********************************/

	/***********************************切出无力后为停止模式***********************************/
	
	if(udtrans_set_mode->last_uptrans_behaviour == UPTRANS_NOFORCE &&
		 !switch_is_down(udtrans_set_mode->udtrans_rc_ctrl->rc.s[0]))
	{	
	  	udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;		
	}
	
	/***********************************切出无力后为停止模式***********************************/
	
	//以下模式优先级等同
	//长按CTRL键停止模式，长长按CTRL键进入手动模式
	if(udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time < RC_KEY_LONG_LONG_TIME)
		{
			 udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time++;
			 if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time == RC_KEY_LONG_TIME)
			 {
				 udtrans_set_mode->uptrans_behaviour = UPTRANS_STOP;     
			 }
			 if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time == RC_KEY_LONG_LONG_TIME)
			 {
				 udtrans_set_mode->uptrans_behaviour = UPTRANS_MANUAL;
			 }
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyCtrl_time = 0;
	}	

  //临时检录：
	//遥控器左右通道在上，为手动模式
	if(switch_is_up(udtrans_set_mode->udtrans_rc_ctrl->rc.s[0]) && switch_is_up(udtrans_set_mode->udtrans_rc_ctrl->rc.s[1]))
	{
	  udtrans_set_mode->uptrans_behaviour = UPTRANS_MANUAL;
	}
	
	//Z键小资源岛拾取（家门口）
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_COLLECT_TINY;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyZ_time = 0;
	}
	
	//X键大资源岛拾取  
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_COLLECT_HUGE;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyX_time = 0;
	}
	
	//C键兑换
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_CONVERSION;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyC_time = 0;
	}
		
	//G键恢复
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_RECOVER;
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyG_time = 0;
	}
	
	//V键补给模式
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time == RC_KEY_LONG_TIME)
			{
				udtrans_set_mode->uptrans_behaviour = UPTRANS_SUPPLY;				
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyV_time = 0;
	}
	
	static  fp32 help_time = 0;
	//F键救援
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time < RC_KEY_LONG_TIME)
		{
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time == RC_KEY_LONG_TIME)
			{
				switch (help_step)      //help_step（救援步骤）全局变量，受其他模式影响
				{
					case 1:
					{
						rescue_on();        //救援伸出
						help_step = 2;	
						break;
					}
					case 2:
					{
						rescue_close();    //救援收回
						help_step = 1;	
						break;
					}	
				}
				udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time = 0;
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyF_time = 0;
		//防止救援气缸未关闭，移动中气缸撞击墙面造成损伤
		if(help_step == 2)
		{
			help_time ++;
			if(help_time == 20000) 
			{
				rescue_close();    //救援收回
				help_step = 1;	   //救援步骤重置为1
				help_time = 0;			
			}			
		}
		else	
		{
		  help_time = 0;			
		}
	}

  static  fp32 snag_time = 0;
	//长按B键气缸除障，长长按B键进入初始化
	if (udtrans_set_mode->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
	{
		if (udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time < RC_KEY_LONG_LONG_TIME)
		{
			//长按进入除障
			udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time++;
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time == RC_KEY_LONG_TIME)
			{						
				switch (snag_step)           //snag_step（除障步骤）全局变量，受其他模式影响
				{
					case 1:
					{
						rotary_actuator_on();     //除障旋转气缸打开
						snag_step = 2;	
						break;
					}
					case 2:
					{
						rotary_actuator_close();  //旋转气缸关闭
						snag_step = 1;	
						break;
					}
				}
			}
			//长长按进入初始化
			if(udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time == RC_KEY_LONG_LONG_TIME)
			{	
				 udtrans_set_mode->uptrans_behaviour = UPTRANS_INIT;						
			}
		}
	}
	else
	{
		udtrans_set_mode->RC_udtrans_ctrl_time.rc_keyB_time = 0;
		//防止旋转气缸未关闭，车速过快导致翻车
		if(snag_step == 2)
		{
			snag_time ++;
			if(snag_time == 10000) 
			{
				rotary_actuator_close();   //除障旋转气缸关闭
				snag_step = 1;		         //除障步骤恢复为1
				snag_time = 0;			
			}			
		}
		else	
		{
			snag_time = 0;			
		}
	}
	
  /******************************************模式优先级转换********************************************/					
}

static void UPTrans_Mode_Change_Control_Transit(UDTrans_Control_t *udtrans_mode_change)
{
	if (udtrans_mode_change == NULL)
	{
		return;
	}
	//无力模式切换角度设置值为当前值，防止下次进入某些模式时电机乱转
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour || udtrans_mode_change->uptrans_behaviour == UPTRANS_NOFORCE)
	{
  	udtrans_mode_change->upmotor_L.motor_angle_set   = udtrans_mode_change->upmotor_L.motor_angle;
  	udtrans_mode_change->upmotor_R.motor_angle_set   = udtrans_mode_change->upmotor_R.motor_angle;
  	udtrans_mode_change->rotatemotor.motor_angle_set = udtrans_mode_change->rotatemotor.motor_angle;
  	udtrans_mode_change->goldmotor.motor_angle_set   = udtrans_mode_change->goldmotor.motor_angle;
  	udtrans_mode_change->snagmotor_L.motor_angle_set = udtrans_mode_change->snagmotor_L.motor_angle;
  	udtrans_mode_change->snagmotor_R.motor_angle_set = udtrans_mode_change->snagmotor_R.motor_angle;
		
  	udtrans_mode_change->upmotor_L.set_ramp_angle   = udtrans_mode_change->upmotor_L.motor_angle;
  	udtrans_mode_change->upmotor_R.set_ramp_angle   = udtrans_mode_change->upmotor_R.motor_angle;
  	udtrans_mode_change->rotatemotor.set_ramp_angle = udtrans_mode_change->rotatemotor.motor_angle;
  	udtrans_mode_change->goldmotor.set_ramp_angle   = udtrans_mode_change->goldmotor.motor_angle;
  	udtrans_mode_change->snagmotor_L.set_ramp_angle = udtrans_mode_change->snagmotor_L.motor_angle;
  	udtrans_mode_change->snagmotor_R.set_ramp_angle = udtrans_mode_change->snagmotor_R.motor_angle;
	}	
	//判断两次模式不同，并且当前为初始化模式，那么所有step重置为1，所有time重置为0，gold重置为1，目的是为了下次进入上述模式时确保其进入第一步
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_INIT)
	{
		snag_step = 1;	
		help_step = 1;
		trans_step = 1;
		trans_time = 0;
    collect_gold = 0;      
		conversion_gold = 0;
    conversion_mouse = 1;
    camera_place = 1;
	}	
	//判断两次模式不同，并且当前为手动模式，抓手夹取、伸缩控制标志位转换，目的是为了进入手动模式时抓手仍处于以前的状态，鼠标按键标志位归零，并且金块相关的辅助变量也回到初始数值
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_MANUAL)
	{
	  press_L_step = clamp_flag;
	  press_R_step = stretch_flag;
		trans_step = 1;	
		trans_time = 0;
    collect_gold = 0;      	
		conversion_gold = 0;
		conversion_mouse = 1;
	}	
	//判断两次模式不同，并且上一次为停止模式（即模式从停止切换出去），停止步骤重置为1，目的是为了下次进入停止模式时确保其进入第一步（停止角度保存）
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 udtrans_mode_change->last_uptrans_behaviour == UPTRANS_STOP )
	{
		stop_step = 1;			
	}
	//判断两次模式不同，并且当前为恢复模式，那么所有step重置为1，所有time重置为0，gold重置为1，目的是为了下次进入上述模式时确保其进入第一步
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 udtrans_mode_change->uptrans_behaviour == UPTRANS_RECOVER )
	{
		snag_step = 1;	
		help_step = 1;
		trans_step = 1;
		trans_time = 0;
		conversion_gold = 0;
    conversion_mouse = 1;
    camera_place = 1;
		switch (collect_gold)    //金块数量赋值判断，拾取个数处理             
		{
			case 0:                             //一个没取到        
			{
	  	  gold_number = collect_gold;	      //直接赋值为0
	    	collect_gold = 0;						      //直接赋值为0
				break;
			}
			case 1:                             //如果取了一个	
			{
	  	  gold_number = collect_gold + 1;		//个数加一，按两个算		
				break; 
			}
			case 2:                             //如果取了两个
			{
	  	  gold_number = collect_gold + 1;	  //个数加一，按三个算				
				break;
			}
			case 3:                             //如果取了两个
			{
	  	  gold_number = collect_gold;	      //直接赋值为3 
	    	collect_gold = 0;			            //归零 		
				break;
			}
			default:
	  	{
				collect_gold = 0;		              //若都不是，归零处理
				break;
		  }
		}	
	}
	//判断两次模式不同，并且上一次为小资源岛取矿、大资源岛取矿、金块兑换模式，则图传回到中间位置，标志位归1
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 (udtrans_mode_change->last_uptrans_behaviour == UPTRANS_COLLECT_TINY ||
	    udtrans_mode_change->last_uptrans_behaviour == UPTRANS_COLLECT_HUGE ||
	    udtrans_mode_change->last_uptrans_behaviour == UPTRANS_CONVERSION   ) )
	{
			camera_mid();             //图传中
			camera_place = 1;			
	}
	//判断两次模式不同，并且上一次为补给模式（即模式从补给切换出去），若补给使用，则仍然关闭，若使用了，则失能，防止舵机过热烧毁
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 udtrans_mode_change->last_uptrans_behaviour == UPTRANS_SUPPLY )
	{
		switch (supply1_step)
		{
			case 1:
			{
			  supply1_close();     //补给1仍然关闭
				break ;
			}
			case 2:
			{
			  supply1_disable();   //补给1失能
				break ;
			}
		}
		switch (supply2_step)
		{
			case 1:
			{
			  supply2_close();     //补给2仍然关闭
				break ;
			}
			case 2:
			{
			  supply2_disable();   //补给2失能
				break ;
			}
		}
	}
	//判断两次模式不同，并且上一次为兑换模式金块拾取个数归零	
	if(udtrans_mode_change->last_uptrans_behaviour != udtrans_mode_change->uptrans_behaviour && 
		 (udtrans_mode_change->uptrans_behaviour == UPTRANS_CONVERSION) )	
	{
		collect_gold = 0;			     
	}
	//手动模式，取矿模式，兑换模式，除障模式，补给模式，无力模式下，救援气缸收回，除障旋转气缸关闭，防止意外
	if(udtrans_mode_change->uptrans_behaviour == UPTRANS_MANUAL ||
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_COLLECT_TINY ||
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_COLLECT_HUGE ||
     udtrans_mode_change->uptrans_behaviour == UPTRANS_CONVERSION ||
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_SNAG ||	
     udtrans_mode_change->uptrans_behaviour == UPTRANS_SUPPLY ||		
	   udtrans_mode_change->uptrans_behaviour == UPTRANS_NOFORCE )	
	{ 
		rescue_close();            //救援收回
		help_step = 1;	           //救援步骤重置为1	
		rotary_actuator_close();   //除障旋转气缸关闭
		snag_step = 1;		         //除障步骤恢复为1		
	}
}

static void UDTrans_Feedback_Update(UDTrans_Control_t *UDTrans_Feedback)
{
	if (UDTrans_Feedback == NULL)
	{
		return;
	}	
	static fp32 Updatetime;
	//rpm计算成速度
	UDTrans_Feedback->upmotor_L.motor_speed   = UDTrans_Feedback->upmotor_L.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->upmotor_R.motor_speed   = UDTrans_Feedback->upmotor_R.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->rotatemotor.motor_speed = UDTrans_Feedback->rotatemotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->goldmotor.motor_speed   = UDTrans_Feedback->goldmotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->snagmotor_L.motor_speed = UDTrans_Feedback->goldmotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	UDTrans_Feedback->snagmotor_R.motor_speed = UDTrans_Feedback->goldmotor.uptrans_motor_measure->speed_rpm *Motor_RMP_TO_SPEED;
	//编码值加和
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->upmotor_L));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->upmotor_R));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->rotatemotor));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->goldmotor));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->snagmotor_L));
	UPTRANS_UpdateMotorAngleSum(&(UDTrans_Feedback->snagmotor_R));
	
  //老电调820R刚上电刚上电编码值会错乱，使其归位
  Updatetime++;
	if (Updatetime < 5000)
	{
		UDTrans_Feedback->goldmotor.motor_angle = 0;
	}	
}

static void UPTrans_control_loop(UDTrans_Control_t *udtrans_control)
{
	if (udtrans_control == NULL)
	{
		return;
	}	

	if(udtrans_control->uptrans_behaviour == UPTRANS_NOFORCE)
	{
		udtrans_control->upmotor_L.given_current   = 0;
		udtrans_control->upmotor_R.given_current   = 0;
		udtrans_control->rotatemotor.given_current = 0;	
		udtrans_control->goldmotor.given_current   = 0;
		udtrans_control->snagmotor_L.given_current = 0;
		udtrans_control->snagmotor_R.given_current = 0;
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_INIT)
	{
		UPTrans_init_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_MANUAL)
	{
		UPTrans_manual_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_STOP)
	{
		UPTrans_stop_control(udtrans_control);
	}
	else if(udtrans_control->uptrans_behaviour == UPTRANS_RECOVER)
	{
		UPTrans_recover_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_SUPPLY)
	{
		UPTrans_supply_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_COLLECT_TINY)
	{
		UPTrans_collect_tiny_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_COLLECT_HUGE)
	{
		UPTrans_collect_huge_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_CONVERSION)
	{
		UPTrans_conversion_control(udtrans_control);
	}	
	else if(udtrans_control->uptrans_behaviour == UPTRANS_HELP)
	{
		UPTrans_help_control(udtrans_control);
	}
	else if(udtrans_control->uptrans_behaviour == UPTRANS_SNAG)
	{
		UPTrans_snag_control(udtrans_control);
	}		
}


//需要更改！！！
static void UPTrans_init_control(UDTrans_Control_t *udtrans_init_control)
{
	static uint8_t udtrans_init_step = 1;
	static fp32 UPTrans_init_time = 0;
	static fp32 angle_out;
	//第一步：恢复所有气缸、舵机状态
	if(udtrans_init_step == 1)  
	{
		//气缸回到初始化状态   
		Clamp_on();                  //夹取装置夹紧 
		stretch_close();             //夹取装置收回 
		rescue_close();              //救援收回 
		rotary_actuator_close();     //除障关闭 
		//舵机回到初始化状态
		//修改：只恢复图传舵机，补给舵机应该也同时关闭，但为了防止舵机过热，不进行关闭处理		
//    supply1_close();             //补给1关闭
//    supply2_close();             //补给2关闭		
    camera_mid();                //图传中				
		//先使电机角度归零（只进行一次），确保下两步初始化计算准确
		udtrans_init_control->upmotor_L.motor_angle   = 0;
		udtrans_init_control->upmotor_R.motor_angle   = 0;
    udtrans_init_control->rotatemotor.motor_angle = 0;
		udtrans_init_control->goldmotor.motor_angle   = 0;
		udtrans_init_control->snagmotor_L.motor_angle = 0;
		udtrans_init_control->snagmotor_R.motor_angle = 0;
		//进行第2步时其他电机保持不动，确保第三步计算准确
	  udtrans_init_control->upmotor_R.motor_angle_set   = udtrans_init_control->upmotor_R.motor_angle;  
	  udtrans_init_control->upmotor_L.motor_angle_set   = udtrans_init_control->upmotor_L.motor_angle; 
	  udtrans_init_control->rotatemotor.motor_angle_set = udtrans_init_control->rotatemotor.motor_angle;	
	  udtrans_init_control->goldmotor.motor_angle_set   = udtrans_init_control->goldmotor.motor_angle;	
	  udtrans_init_control->snagmotor_L.motor_angle_set = udtrans_init_control->snagmotor_L.motor_angle;	
	  udtrans_init_control->snagmotor_R.motor_angle_set = udtrans_init_control->snagmotor_R.motor_angle;	
		udtrans_init_step = 2;
  }	
	//第二步：初始化金块抬升电机：寻找限位点，使角度归零
	else if(udtrans_init_step == 2)
	{
		//金块抬升电机慢速旋转
	  udtrans_init_control->goldmotor.motor_angle_set -= 10; 
		//判断是否到达限位点   注：fabs求浮点数绝对值，abs求整数绝对值
	  if(fabs(udtrans_init_control->goldmotor.motor_angle_set - udtrans_init_control->goldmotor.motor_angle) > 5000)   
		{
		  UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000 || IF_MOUSE_PRESSED_RIGH)   //或按下鼠标左键跳过此步骤
		{
			//此时找到，设置为零
		  udtrans_init_control->goldmotor.motor_angle = 0;
		  udtrans_init_control->goldmotor.motor_angle_set = 0;
	    udtrans_init_step = 3;
			UPTrans_init_time = 0;
		}

	}
	//第三步：初始化抬升电机：寻找限位点，使角度归零
	else if(udtrans_init_step == 3)
	{	
		//抬升电机慢速旋转
	  udtrans_init_control->upmotor_L.motor_angle_set -= 10; 
		//判断是否到达限位点   注：fabs求浮点数绝对值，abs求整数绝对值
	  if(fabs(udtrans_init_control->upmotor_L.motor_angle_set - udtrans_init_control->upmotor_L.motor_angle) > 3000)   
		{
		  UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000 || IF_MOUSE_PRESSED_RIGH)   //或按下鼠标左键跳过此步骤
		{
			//此时找到，设置为零
		  udtrans_init_control->upmotor_L.motor_angle = 0;
		  udtrans_init_control->upmotor_L.motor_angle_set = 0;
			udtrans_init_control->upmotor_R.motor_angle = 0;
		  udtrans_init_control->upmotor_R.motor_angle_set = 0;
	    udtrans_init_step = 4;
			UPTrans_init_time = 0;
		}
	}
	//第四步：初始化夹取装置旋转电机：寻找限位点，使角度归零
	else if(udtrans_init_step == 4)
	{
		//夹取装置旋转电机慢速旋转
		udtrans_init_control->rotatemotor.motor_angle_set -= 10; 
		//判断是否到达限位点
	  if(fabs(udtrans_init_control->rotatemotor.motor_angle_set - udtrans_init_control->rotatemotor.motor_angle) > 3000)
		{
			UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000 || IF_MOUSE_PRESSED_RIGH)    //或按下鼠标左键跳过此步骤
		{
			//此时找到，设置为零
		  udtrans_init_control->rotatemotor.motor_angle = 0;
		  udtrans_init_control->rotatemotor.motor_angle_set = 0;
			//初始化结束，切换为停止模式，初始化步骤重置为1，计时归零
      udtrans_init_control->uptrans_behaviour = UPTRANS_STOP;
	    udtrans_init_step = 1;
			UPTrans_init_time = 0;
		}
	}
	
#if 0   //障碍块电机初始化，目前没有用到
	//第二步：初始化障碍块电机：寻找限位点，使角度归零
	else if(udtrans_init_step == 2)
	{
		//抬升电机慢速旋转
	  udtrans_init_control->snagmotor_L.motor_angle_set -= 10; 
		//判断是否到达限位点   注：fabs求浮点数绝对值，abs求整数绝对值
	  if(fabs(udtrans_init_control->snagmotor_L.motor_angle_set - udtrans_init_control->snagmotor_L.motor_angle) > 3000)   
		{
		  UPTrans_init_time++;
		}
		if(UPTrans_init_time == 1000)
		{
			//此时找到，设置为零
		  udtrans_init_control->snagmotor_L.motor_angle = 0;
		  udtrans_init_control->snagmotor_L.motor_angle_set = 0;
			udtrans_init_control->snagmotor_R.motor_angle = 0;
		  udtrans_init_control->snagmotor_R.motor_angle_set = 0;
	    udtrans_init_step = 3;
			UPTrans_init_time = 0;
		}
	}
#endif	

	//抬升电机PID计算
	angle_out = PID_Calc(&udtrans_init_control->upmotor_L.motor_angle_pid, udtrans_init_control->upmotor_L.motor_angle_set, udtrans_init_control->upmotor_L.motor_angle);
	udtrans_init_control->upmotor_L.given_current = PID_Calc(&udtrans_init_control->upmotor_L.motor_speed_pid, udtrans_init_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用
	angle_out = PID_Calc(&udtrans_init_control->upmotor_R.motor_angle_pid, -udtrans_init_control->upmotor_L.motor_angle_set, udtrans_init_control->upmotor_R.motor_angle);
	udtrans_init_control->upmotor_R.given_current = PID_Calc(&udtrans_init_control->upmotor_R.motor_speed_pid, udtrans_init_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_init_control->rotatemotor.motor_angle_pid, udtrans_init_control->rotatemotor.motor_angle_set, udtrans_init_control->rotatemotor.motor_angle);
	udtrans_init_control->rotatemotor.given_current = PID_Calc(&udtrans_init_control->rotatemotor.motor_speed_pid, udtrans_init_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	angle_out = PID_Calc(&udtrans_init_control->goldmotor.motor_angle_pid, udtrans_init_control->goldmotor.motor_angle_set, udtrans_init_control->goldmotor.motor_angle);
	udtrans_init_control->goldmotor.given_current = PID_Calc(&udtrans_init_control->goldmotor.motor_speed_pid, udtrans_init_control->goldmotor.motor_speed, angle_out);	
  //障碍块电机PID计算
	angle_out = PID_Calc(&udtrans_init_control->snagmotor_L.motor_angle_pid, udtrans_init_control->snagmotor_L.motor_angle_set, udtrans_init_control->snagmotor_L.motor_angle);
	udtrans_init_control->snagmotor_L.given_current = PID_Calc(&udtrans_init_control->snagmotor_L.motor_speed_pid, udtrans_init_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_init_control->snagmotor_R.motor_angle_pid, -udtrans_init_control->snagmotor_L.motor_angle_set, udtrans_init_control->snagmotor_R.motor_angle);
	udtrans_init_control->snagmotor_R.given_current = PID_Calc(&udtrans_init_control->snagmotor_R.motor_speed_pid, udtrans_init_control->snagmotor_R.motor_speed, angle_out);
}

//只需长按CTRL键即可切换到手动模式
static void UPTrans_manual_control(UDTrans_Control_t *udtrans_manual_control)
{
	static fp32 angle_out;
	static fp32 vx_set_channel;
	int16_t vx_channel;	 //遥控器原始通道值
	
	//死区限制，因为遥控器可能存在差异，摇杆在中间，其值不为0
	rc_deadline_limit(udtrans_manual_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//通道值赋值
	vx_set_channel = vx_channel;
	
	//遥控器左通道在下，控制夹取装置旋转电机
	if(switch_is_down(udtrans_manual_control->udtrans_rc_ctrl->rc.s[1]))
	{
		//设置目标角度（累加）
    udtrans_manual_control->rotatemotor.motor_angle_set += vx_set_channel*0.3f; 
	}
	//遥控器左通道在中，控制抬升电机
	if(switch_is_mid(udtrans_manual_control->udtrans_rc_ctrl->rc.s[1]))
	{
		//设置目标角度（累加）
    udtrans_manual_control->upmotor_L.motor_angle_set += vx_set_channel*0.3f; 
	}
	//遥控器左通道在上，控制金块电机
	if(switch_is_up(udtrans_manual_control->udtrans_rc_ctrl->rc.s[1]))
	{
		//设置目标角度（累加）
    udtrans_manual_control->goldmotor.motor_angle_set += vx_set_channel*0.3f; 
	}
	
	//鼠标左键检测
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time < RC_KEY_LONG_TIME)
		{
			udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time++;
			if(udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time == RC_KEY_LONG_TIME)
			{
				switch (press_L_step)         
				{
					case 0:
					{
            Clamp_close();
						press_L_step = 1;
						break;
					}
					case 1:
					{
            Clamp_on();
						press_L_step = 0;
						break;
					}
				}
				udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
			}
		}
	}
	else
	{
		udtrans_manual_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
	}
	
	//鼠标右键检测
	if (IF_MOUSE_PRESSED_RIGH)
	{
		if (udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time < RC_KEY_LONG_TIME)
		{
			udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time++;
			if(udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time == RC_KEY_LONG_TIME)
			{
				//鼠标右键检测	
				switch (press_R_step)         
				{
					case 0:
					{
            stretch_on();
						press_R_step = 1;
						break;
					}
					case 1:
					{
            stretch_close();
						press_R_step = 0;
						break;
					}
				}
				udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
			}
		}
	}
	else
	{
		udtrans_manual_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
	}

  //限幅函数
	udtrans_manual_control->upmotor_L.motor_angle_set=fp32_constrain_upmotor_angle(&udtrans_manual_control->upmotor_L);
	udtrans_manual_control->rotatemotor.motor_angle_set=fp32_constrain_upmotor_angle(&udtrans_manual_control->rotatemotor);
	udtrans_manual_control->goldmotor.motor_angle_set=fp32_constrain_upmotor_angle(&udtrans_manual_control->goldmotor);

	//抬升电机PID计算
	angle_out = PID_Calc(&udtrans_manual_control->upmotor_L.motor_angle_pid, udtrans_manual_control->upmotor_L.motor_angle_set, udtrans_manual_control->upmotor_L.motor_angle);
	udtrans_manual_control->upmotor_L.given_current = PID_Calc(&udtrans_manual_control->upmotor_L.motor_speed_pid, udtrans_manual_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用
	angle_out = PID_Calc(&udtrans_manual_control->upmotor_R.motor_angle_pid,-udtrans_manual_control->upmotor_L.motor_angle_set, udtrans_manual_control->upmotor_R.motor_angle);
	udtrans_manual_control->upmotor_R.given_current = PID_Calc(&udtrans_manual_control->upmotor_R.motor_speed_pid, udtrans_manual_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_manual_control->rotatemotor.motor_angle_pid, udtrans_manual_control->rotatemotor.motor_angle_set, udtrans_manual_control->rotatemotor.motor_angle);
	udtrans_manual_control->rotatemotor.given_current = PID_Calc(&udtrans_manual_control->rotatemotor.motor_speed_pid, udtrans_manual_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	angle_out = PID_Calc(&udtrans_manual_control->goldmotor.motor_angle_pid, udtrans_manual_control->goldmotor.motor_angle_set, udtrans_manual_control->goldmotor.motor_angle);
	udtrans_manual_control->goldmotor.given_current = PID_Calc(&udtrans_manual_control->goldmotor.motor_speed_pid, udtrans_manual_control->goldmotor.motor_speed, angle_out);	
  //障碍块电机PID计算
	angle_out = PID_Calc(&udtrans_manual_control->snagmotor_L.motor_angle_pid, udtrans_manual_control->snagmotor_L.motor_angle_set, udtrans_manual_control->snagmotor_L.motor_angle);
	udtrans_manual_control->snagmotor_L.given_current = PID_Calc(&udtrans_manual_control->snagmotor_L.motor_speed_pid, udtrans_manual_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_manual_control->snagmotor_R.motor_angle_pid, udtrans_manual_control->snagmotor_R.motor_angle_set, udtrans_manual_control->snagmotor_R.motor_angle);
	udtrans_manual_control->snagmotor_R.given_current = PID_Calc(&udtrans_manual_control->snagmotor_R.motor_speed_pid, udtrans_manual_control->snagmotor_R.motor_speed, angle_out);	
}

static void UPTrans_stop_control(UDTrans_Control_t *udtrans_stop_control)
{
 	static fp32 angle_out;
	static fp32 upmotor_L_stop_angle,upmotor_R_stop_angle,rotatemotor_stop_angle;
	static fp32 goldmotor_stop_angle,snagmotor_L_stop_angle,snagmotor_R_stop_angle;

	if(stop_step == 1)     //stop_step（停止步骤）全局变量，受其他模式影响
	{
		//只进行一次暂停位置保存
		upmotor_L_stop_angle   = udtrans_stop_control->upmotor_L.motor_angle;
		upmotor_R_stop_angle   = udtrans_stop_control->upmotor_R.motor_angle;
		rotatemotor_stop_angle = udtrans_stop_control->rotatemotor.motor_angle;
		goldmotor_stop_angle   = udtrans_stop_control->goldmotor.motor_angle;
		snagmotor_L_stop_angle = udtrans_stop_control->snagmotor_L.motor_angle;
		snagmotor_R_stop_angle = udtrans_stop_control->snagmotor_R.motor_angle;
		stop_step = 0;       //只有在切换模式后才会变成0
	}
	
	udtrans_stop_control->upmotor_L.motor_angle_set   = upmotor_L_stop_angle;
  udtrans_stop_control->upmotor_R.motor_angle_set   = upmotor_R_stop_angle;
  udtrans_stop_control->rotatemotor.motor_angle_set = rotatemotor_stop_angle;
  udtrans_stop_control->goldmotor.motor_angle_set   = goldmotor_stop_angle;
  udtrans_stop_control->snagmotor_L.motor_angle_set = snagmotor_L_stop_angle;
  udtrans_stop_control->snagmotor_R.motor_angle_set = snagmotor_R_stop_angle;
	
	//抬升电机PID计算
	angle_out = PID_Calc(&udtrans_stop_control->upmotor_L.motor_angle_pid, udtrans_stop_control->upmotor_L.motor_angle_set, udtrans_stop_control->upmotor_L.motor_angle);
	udtrans_stop_control->upmotor_L.given_current = PID_Calc(&udtrans_stop_control->upmotor_L.motor_speed_pid, udtrans_stop_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用
	angle_out = PID_Calc(&udtrans_stop_control->upmotor_R.motor_angle_pid, udtrans_stop_control->upmotor_R.motor_angle_set, udtrans_stop_control->upmotor_R.motor_angle);
	udtrans_stop_control->upmotor_R.given_current = PID_Calc(&udtrans_stop_control->upmotor_R.motor_speed_pid, udtrans_stop_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_stop_control->rotatemotor.motor_angle_pid, udtrans_stop_control->rotatemotor.motor_angle_set, udtrans_stop_control->rotatemotor.motor_angle);
	udtrans_stop_control->rotatemotor.given_current = PID_Calc(&udtrans_stop_control->rotatemotor.motor_speed_pid, udtrans_stop_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	angle_out = PID_Calc(&udtrans_stop_control->goldmotor.motor_angle_pid, udtrans_stop_control->goldmotor.motor_angle_set, udtrans_stop_control->goldmotor.motor_angle);
	udtrans_stop_control->goldmotor.given_current = PID_Calc(&udtrans_stop_control->goldmotor.motor_speed_pid, udtrans_stop_control->goldmotor.motor_speed, angle_out);	
  //障碍块电机PID计算
	angle_out = PID_Calc(&udtrans_stop_control->snagmotor_L.motor_angle_pid, udtrans_stop_control->snagmotor_L.motor_angle_set, udtrans_stop_control->snagmotor_L.motor_angle);
	udtrans_stop_control->snagmotor_L.given_current = PID_Calc(&udtrans_stop_control->snagmotor_L.motor_speed_pid, udtrans_stop_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_stop_control->snagmotor_R.motor_angle_pid, udtrans_stop_control->snagmotor_R.motor_angle_set, udtrans_stop_control->snagmotor_R.motor_angle);
	udtrans_stop_control->snagmotor_R.given_current = PID_Calc(&udtrans_stop_control->snagmotor_R.motor_speed_pid, udtrans_stop_control->snagmotor_R.motor_speed, angle_out);
}

static void UPTrans_recover_control(UDTrans_Control_t *udtrans_recover_control)
{
	static fp32 angle_out;
	static uint8_t recover_step = 1;
  static fp32    recover_time = 0; 

#if 0	
	//第一步：恢复所有气缸、舵机状态
	if(recover_step == 1)  
	{
		//气缸回到初始化状态   问题：气缸一次性收回是否有影响？
		Clamp_on();                  //夹取装置松开 
		stretch_close();             //夹取装置收回 
		rescue_close();              //救援收回 
		rotary_actuator_close();     //除障关闭 
		//舵机回到初始化状态 
		recover_step = 2;
	}
	//第二步：金块抬升落下
	else if(recover_step == 2)
	{
		udtrans_recover_control->goldmotor.motor_angle_set = udtrans_recover_control->goldmotor.min_angle_limit;    
		if(udtrans_recover_control->goldmotor.set_ramp_angle != udtrans_recover_control->goldmotor.motor_angle_set)
		{
			udtrans_recover_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_recover_control->goldmotor.motor_angle_set, udtrans_recover_control->goldmotor.set_ramp_angle, 300);
		}	
		
		if(udtrans_recover_control->goldmotor.set_ramp_angle == udtrans_recover_control->goldmotor.motor_angle_set )
		{
			recover_step = 3;
		}	
	}	
	//第三步：抬升落下，夹取装置旋转归位 
	else if(recover_step == 3)
	{
		//抬升落下
		udtrans_recover_control->upmotor_L.motor_angle_set = udtrans_recover_control->upmotor_L.min_angle_limit;    
		if(udtrans_recover_control->upmotor_L.set_ramp_angle != udtrans_recover_control->upmotor_L.motor_angle_set)
		{
			udtrans_recover_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_recover_control->upmotor_L.motor_angle_set, udtrans_recover_control->upmotor_L.set_ramp_angle, 300);
		}
		//夹取装置旋转归位
		udtrans_recover_control->rotatemotor.motor_angle_set = udtrans_recover_control->rotatemotor.min_angle_limit; 
		if(udtrans_recover_control->rotatemotor.set_ramp_angle != udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			udtrans_recover_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_recover_control->rotatemotor.motor_angle_set, udtrans_recover_control->rotatemotor.set_ramp_angle, 200);
		}
		//判断恢复是否完成
		if(udtrans_recover_control->upmotor_L.set_ramp_angle == udtrans_recover_control->upmotor_L.motor_angle_set &&
			 udtrans_recover_control->rotatemotor.set_ramp_angle == udtrans_recover_control->rotatemotor.motor_angle_set)
		{
      recover_step = 4;
		}	
	}
	//第四步：障碍块电机归位
	else if(recover_step == 4)
	{
		udtrans_recover_control->snagmotor_L.motor_angle_set = udtrans_recover_control->snagmotor_L.min_angle_limit;    
		if(udtrans_recover_control->snagmotor_L.set_ramp_angle != udtrans_recover_control->snagmotor_L.motor_angle_set)
		{
			udtrans_recover_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_recover_control->snagmotor_L.motor_angle_set, udtrans_recover_control->snagmotor_L.set_ramp_angle, 50);
		}	
		
		if(udtrans_recover_control->snagmotor_L.set_ramp_angle == udtrans_recover_control->snagmotor_L.motor_angle_set )
		{
			//循环一段时间，保证恢复到初始位置
			recover_time++;
			if(recover_time == 500) 
			{
				//恢复完成，切换为停止模式，恢复步骤重置为1
				udtrans_recover_control->uptrans_behaviour = UPTRANS_STOP; 
				recover_step = 1;		
        recover_time = 0;			
			}
		}	
	}	
#else
	//第一步：抓手电机先抬起
	if(recover_step == 1) 
	{
		//夹取装置旋转归位
		udtrans_recover_control->rotatemotor.motor_angle_set = udtrans_recover_control->rotatemotor.mid_angle_limit; 
		if(udtrans_recover_control->rotatemotor.set_ramp_angle != udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			udtrans_recover_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_recover_control->rotatemotor.motor_angle_set, udtrans_recover_control->rotatemotor.set_ramp_angle, 
			                                                                 HAND_FAST);
		}
		if(udtrans_recover_control->rotatemotor.set_ramp_angle == udtrans_recover_control->rotatemotor.motor_angle_set)
		{
      recover_step = 2;		
		}		
	}
	//第二步：恢复所有气缸、舵机状态
	if(recover_step == 2)  
	{
    recover_time++;
		//气缸回到初始化状态   问题：气缸一次性收回是否有影响？
		Clamp_on();                  //夹取装置松开 
		stretch_close();             //夹取装置收回 
		rescue_close();              //救援收回 
		rotary_actuator_close();     //除障关闭 
		//舵机回到初始化状态 
		//修改：只恢复图传舵机，补给舵机应该也同时关闭，但为了防止舵机过热，不进行关闭处理 过热原因：机械安装滑轨时精度不够，连杆设计有缺陷
//    supply1_close();             //补给1关闭
//    supply2_close();             //补给2关闭		
    camera_mid();                //图传中		
		//等待一段时间进入下一步
		if(recover_time == 500)
		{	
      recover_step = 3;						
			recover_time = 0;
		}
	}
	//第三步：金块抬升落下
	else if(recover_step == 3)
	{
		udtrans_recover_control->goldmotor.motor_angle_set = udtrans_recover_control->goldmotor.min_angle_limit;    
		if(udtrans_recover_control->goldmotor.set_ramp_angle != udtrans_recover_control->goldmotor.motor_angle_set)
		{
			udtrans_recover_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_recover_control->goldmotor.motor_angle_set, udtrans_recover_control->goldmotor.set_ramp_angle, 
			                                                               GOLD_UP_SPEED);
		}	
		
		if(udtrans_recover_control->goldmotor.set_ramp_angle == udtrans_recover_control->goldmotor.motor_angle_set )
		{
			recover_step = 4;
		}	
	}	
	//第四步：抬升落下，夹取装置旋转归位 
	else if(recover_step == 4)
	{
		//抬升落下
		udtrans_recover_control->upmotor_L.motor_angle_set = udtrans_recover_control->upmotor_L.min_angle_limit;    
		if(udtrans_recover_control->upmotor_L.set_ramp_angle != udtrans_recover_control->upmotor_L.motor_angle_set)
		{
			udtrans_recover_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_recover_control->upmotor_L.motor_angle_set, udtrans_recover_control->upmotor_L.set_ramp_angle, 
			                                                               TRANS_UP_SPEED);
		}
		//夹取装置旋转归位
		udtrans_recover_control->rotatemotor.motor_angle_set = udtrans_recover_control->rotatemotor.min_angle_limit; 
		if(udtrans_recover_control->rotatemotor.set_ramp_angle != udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			udtrans_recover_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_recover_control->rotatemotor.motor_angle_set, udtrans_recover_control->rotatemotor.set_ramp_angle,
		                                                                 	 HAND_FAST);
		}
		//判断恢复是否完成
		if(udtrans_recover_control->upmotor_L.set_ramp_angle == udtrans_recover_control->upmotor_L.motor_angle_set &&
			 udtrans_recover_control->rotatemotor.set_ramp_angle == udtrans_recover_control->rotatemotor.motor_angle_set)
		{
			//循环一段时间，保证恢复到初始位置
			recover_time++;
			if(recover_time == 500) 
			{
				//恢复完成，切换为停止模式，恢复步骤重置为1
				udtrans_recover_control->uptrans_behaviour = UPTRANS_STOP; 
				recover_step = 1;		
        recover_time = 0;			
			}
		}	
	}

#endif
	
  //抬升电机PID计算
	angle_out = PID_Calc(&udtrans_recover_control->upmotor_L.motor_angle_pid, udtrans_recover_control->upmotor_L.set_ramp_angle, udtrans_recover_control->upmotor_L.motor_angle);
	udtrans_recover_control->upmotor_L.given_current = PID_Calc(&udtrans_recover_control->upmotor_L.motor_speed_pid, udtrans_recover_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_recover_control->upmotor_R.motor_angle_pid,-udtrans_recover_control->upmotor_L.set_ramp_angle, udtrans_recover_control->upmotor_R.motor_angle);
	udtrans_recover_control->upmotor_R.given_current = PID_Calc(&udtrans_recover_control->upmotor_R.motor_speed_pid, udtrans_recover_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_recover_control->rotatemotor.motor_angle_pid, udtrans_recover_control->rotatemotor.set_ramp_angle, udtrans_recover_control->rotatemotor.motor_angle);
	udtrans_recover_control->rotatemotor.given_current = PID_Calc(&udtrans_recover_control->rotatemotor.motor_speed_pid, udtrans_recover_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	angle_out = PID_Calc(&udtrans_recover_control->goldmotor.motor_angle_pid, udtrans_recover_control->goldmotor.set_ramp_angle, udtrans_recover_control->goldmotor.motor_angle);
	udtrans_recover_control->goldmotor.given_current = PID_Calc(&udtrans_recover_control->goldmotor.motor_speed_pid, udtrans_recover_control->goldmotor.motor_speed, angle_out);
  //障碍块电机PID计算
	angle_out = PID_Calc(&udtrans_recover_control->snagmotor_L.motor_angle_pid, udtrans_recover_control->snagmotor_L.set_ramp_angle, udtrans_recover_control->snagmotor_L.motor_angle);
	udtrans_recover_control->snagmotor_L.given_current = PID_Calc(&udtrans_recover_control->snagmotor_L.motor_speed_pid, udtrans_recover_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_recover_control->snagmotor_R.motor_angle_pid,-udtrans_recover_control->snagmotor_L.set_ramp_angle, udtrans_recover_control->snagmotor_R.motor_angle);
	udtrans_recover_control->snagmotor_R.given_current = PID_Calc(&udtrans_recover_control->snagmotor_R.motor_speed_pid, udtrans_recover_control->snagmotor_R.motor_speed, angle_out);
}

static void UPTrans_snag_control(UDTrans_Control_t *udtrans_snag_control)
{
	static fp32 angle_out;
	static fp32 vx_set_channel;  //
	int16_t vx_channel;	         //遥控器原始通道值
	
	//第一步：障碍电机最大位置
	if (trans_step == 1)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.max_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}		
		else
		{
		  trans_step = 2;	
		}
	}
	//第二步：障碍块电机向前缓慢移动，两个障碍抓手触地，使其同步
	else if(trans_step == 2)
	{
		//抬升电机慢速旋转
	  udtrans_snag_control->snagmotor_L.motor_angle_set += 20; 
		//判断是否到达限位点   注：fabs求浮点数绝对值，abs求整数绝对值
	  if(fabs(udtrans_snag_control->snagmotor_L.motor_angle_set - udtrans_snag_control->snagmotor_L.motor_angle) > 5000)   
		{
		  trans_time++;
		}
		if(trans_time == 1000)
		{
	    udtrans_snag_control->snagmotor_L.MAX_angle_limit = udtrans_snag_control->snagmotor_L.motor_angle;
	    trans_step = 3;
			trans_time = 0;
		}
	}
	//第三步：手动调节位置
	else if (trans_step == 3)
	{
		rc_deadline_limit(udtrans_snag_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
    //通道值赋值
		vx_set_channel = vx_channel;
		//设置目标角度（累加）
		udtrans_snag_control->snagmotor_L.set_ramp_angle += vx_set_channel*0.3f;
    //限幅设置
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle > udtrans_snag_control->snagmotor_L.max_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.max_angle_limit;
		}
		else if(udtrans_snag_control->snagmotor_L.set_ramp_angle < udtrans_snag_control->snagmotor_L.min_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.min_angle_limit;
		}	
		
		//调节完毕，再按下V键进入下一步
		if (udtrans_snag_control->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
		{
			if (udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time < RC_KEY_LONG_TIME)
			{
				udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time++;
				if(udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time == RC_KEY_LONG_TIME)
				{
					trans_step = 4;
					udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
				}
			}
		}
		else
		{
			udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
		}
	}
	//第四步：障碍电机中间位置（举起障碍块）
	else if (trans_step == 4)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.mid_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}	
		//到达放置位置，再按下B键进入下一步
		if (udtrans_snag_control->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
		{
			if (udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time < RC_KEY_LONG_TIME)
			{
				udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time++;
				if(udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time == RC_KEY_LONG_TIME)
				{
					trans_step = 5;
					udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time = 0;
				}
			}
		}
		else
		{
			udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyB_time = 0;
		}		
	}
	//第五步：障碍电机最大位置（放下障碍块）
	else if (trans_step == 5)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.max_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}		
		else
		{
		  trans_step = 6;	
		}
	}
	//第六步：手动调节
	else if (trans_step == 6)
	{	

		rc_deadline_limit(udtrans_snag_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
    //通道值赋值
		vx_set_channel = vx_channel;
		//设置目标角度（累加）
		udtrans_snag_control->snagmotor_L.set_ramp_angle += vx_set_channel*0.3f;
    //限幅设置
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle > udtrans_snag_control->snagmotor_L.max_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.max_angle_limit;
		}
		else if(udtrans_snag_control->snagmotor_L.set_ramp_angle < udtrans_snag_control->snagmotor_L.min_angle_limit)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.min_angle_limit;
		}	
		
		//调节完毕，再按下V键进入下一步
		if (udtrans_snag_control->udtrans_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
		{
			if (udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time < RC_KEY_LONG_TIME)
			{
				udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time++;
				if(udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time == RC_KEY_LONG_TIME)
				{
					trans_step = 7;
					udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
				}
			}
		}
		else
		{
			udtrans_snag_control->RC_udtrans_ctrl_time.rc_keyV_time = 0;
		}
	}
	//第七步：障碍电机收回
	else if (trans_step == 7)
	{
		udtrans_snag_control->snagmotor_L.motor_angle_set = udtrans_snag_control->snagmotor_L.min_angle_limit;    
		if(udtrans_snag_control->snagmotor_L.set_ramp_angle != udtrans_snag_control->snagmotor_L.motor_angle_set)
		{
			udtrans_snag_control->snagmotor_L.set_ramp_angle = RAMP_float(udtrans_snag_control->snagmotor_L.motor_angle_set, udtrans_snag_control->snagmotor_L.set_ramp_angle, 100);
		}		
		else
		{
			trans_time++;
			//循环一段时间，保证障碍块电机回到初始位置
			if( trans_time == 500 )  
			{
				//除障完成，切换为停止模式，除障步骤重置为1，计时归零
				udtrans_snag_control->uptrans_behaviour = UPTRANS_STOP; 
				trans_step = 1;		
        trans_time = 0;			
			}
		}
	}
	
	//限幅函数
	if (udtrans_snag_control->snagmotor_L.set_ramp_angle > udtrans_snag_control->snagmotor_L.MAX_angle_limit)
	{
		udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.MAX_angle_limit;
	}
	else if (udtrans_snag_control->snagmotor_L.set_ramp_angle < udtrans_snag_control->snagmotor_L.min_angle_limit)
	{
		udtrans_snag_control->snagmotor_L.set_ramp_angle = udtrans_snag_control->snagmotor_L.min_angle_limit;
	}
	//抬升电机PID计算
	//由于取除障过程中用不到金块抬升电机，角度设置不变（motor_angle_set）
	angle_out = PID_Calc(&udtrans_snag_control->upmotor_L.motor_angle_pid, udtrans_snag_control->upmotor_L.motor_angle_set, udtrans_snag_control->upmotor_L.motor_angle);
	udtrans_snag_control->upmotor_L.given_current = PID_Calc(&udtrans_snag_control->upmotor_L.motor_speed_pid, udtrans_snag_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	//upmotor_R.motor_angle_set可能不会改变，所以直接使upmotor_R.motor_angle_set取负使用	
	angle_out = PID_Calc(&udtrans_snag_control->upmotor_R.motor_angle_pid,-udtrans_snag_control->upmotor_L.motor_angle_set, udtrans_snag_control->upmotor_R.motor_angle);
	udtrans_snag_control->upmotor_R.given_current = PID_Calc(&udtrans_snag_control->upmotor_R.motor_speed_pid, udtrans_snag_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
  //由于除障过程中用不到金块抬升电机，角度设置不变（motor_angle_set）
	angle_out = PID_Calc(&udtrans_snag_control->rotatemotor.motor_angle_pid, udtrans_snag_control->rotatemotor.motor_angle_set, udtrans_snag_control->rotatemotor.motor_angle);
	udtrans_snag_control->rotatemotor.given_current = PID_Calc(&udtrans_snag_control->rotatemotor.motor_speed_pid, udtrans_snag_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	//由于取除障过程中用不到金块抬升电机，角度设置不变（motor_angle_set）
	angle_out = PID_Calc(&udtrans_snag_control->goldmotor.motor_angle_pid, udtrans_snag_control->goldmotor.motor_angle_set, udtrans_snag_control->goldmotor.motor_angle);
	udtrans_snag_control->goldmotor.given_current = PID_Calc(&udtrans_snag_control->goldmotor.motor_speed_pid, udtrans_snag_control->goldmotor.motor_speed, angle_out);	
  //障碍块电机PID计算
	angle_out = PID_Calc(&udtrans_snag_control->snagmotor_L.motor_angle_pid, udtrans_snag_control->snagmotor_L.set_ramp_angle, udtrans_snag_control->snagmotor_L.motor_angle);
	udtrans_snag_control->snagmotor_L.given_current = PID_Calc(&udtrans_snag_control->snagmotor_L.motor_speed_pid, udtrans_snag_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用		
	angle_out = PID_Calc(&udtrans_snag_control->snagmotor_R.motor_angle_pid,-udtrans_snag_control->snagmotor_L.set_ramp_angle, udtrans_snag_control->snagmotor_R.motor_angle);
	udtrans_snag_control->snagmotor_R.given_current = PID_Calc(&udtrans_snag_control->snagmotor_R.motor_speed_pid, udtrans_snag_control->snagmotor_R.motor_speed, angle_out);
}

//补给模式
static void UPTrans_supply_control(UDTrans_Control_t *udtrans_supply_control)
{
	//鼠标左键检测
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time < RC_KEY_LONG_TIME)
		{
			udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time++;
			if(udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time == RC_KEY_LONG_TIME)
			{
				supply1_on();         //补给1开启
				supply1_step = 2;	
				udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
			}
		}
	}
	else
	{
		udtrans_supply_control->RC_udtrans_ctrl_time.mouse_left_time = 0;
	}
	
	//鼠标右键检测
	if (IF_MOUSE_PRESSED_RIGH)
	{
		if (udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time < RC_KEY_LONG_TIME)
		{
			udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time++;
			if(udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time == RC_KEY_LONG_TIME)
			{
				supply2_on();        //补给2开启
				supply2_step = 2;	
				udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
			}
		}
	}
	else
	{
		udtrans_supply_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
	}
	
}

//救援模式  预留了位置，机械没有制作这一功能，只有刷卡救援
static void UPTrans_help_control(UDTrans_Control_t *udtrans_help_control)
{
	static fp32 angle_out;	
//	static fp32 vx_set_channel;
//	int16_t vx_channel;	 //遥控器原始通道值
//	
//	rc_deadline_limit(udtrans_help_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
//	//通道值赋值
//	vx_set_channel = vx_channel;
	
	
	//抬升电机PID计算
	angle_out = PID_Calc(&udtrans_help_control->upmotor_L.motor_angle_pid, udtrans_help_control->upmotor_L.motor_angle_set, udtrans_help_control->upmotor_L.motor_angle);
	udtrans_help_control->upmotor_L.given_current = PID_Calc(&udtrans_help_control->upmotor_L.motor_speed_pid, udtrans_help_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_help_control->upmotor_R.motor_angle_pid,-udtrans_help_control->upmotor_L.motor_angle_set, udtrans_help_control->upmotor_R.motor_angle);
	udtrans_help_control->upmotor_R.given_current = PID_Calc(&udtrans_help_control->upmotor_R.motor_speed_pid, udtrans_help_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_help_control->rotatemotor.motor_angle_pid, udtrans_help_control->rotatemotor.motor_angle_set, udtrans_help_control->rotatemotor.motor_angle);
	udtrans_help_control->rotatemotor.given_current = PID_Calc(&udtrans_help_control->rotatemotor.motor_speed_pid, udtrans_help_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	angle_out = PID_Calc(&udtrans_help_control->goldmotor.motor_angle_pid, udtrans_help_control->goldmotor.motor_angle_set, udtrans_help_control->goldmotor.motor_angle);
	udtrans_help_control->goldmotor.given_current = PID_Calc(&udtrans_help_control->goldmotor.motor_speed_pid, udtrans_help_control->goldmotor.motor_speed, angle_out);	
  //障碍块电机PID计算
	angle_out = PID_Calc(&udtrans_help_control->snagmotor_L.motor_angle_pid, udtrans_help_control->snagmotor_L.motor_angle_set, udtrans_help_control->snagmotor_L.motor_angle);
	udtrans_help_control->snagmotor_L.given_current = PID_Calc(&udtrans_help_control->snagmotor_L.motor_speed_pid, udtrans_help_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用		
	angle_out = PID_Calc(&udtrans_help_control->snagmotor_R.motor_angle_pid,-udtrans_help_control->snagmotor_L.motor_angle_set, udtrans_help_control->snagmotor_R.motor_angle);
	udtrans_help_control->snagmotor_R.given_current = PID_Calc(&udtrans_help_control->snagmotor_R.motor_speed_pid, udtrans_help_control->snagmotor_R.motor_speed, angle_out);
}


//小资源岛，取三个，抬升200，夹取不需要伸出
static void UPTrans_collect_tiny_control(UDTrans_Control_t *udtrans_collect_tiny_control)
{
	static fp32 angle_out;	
	static fp32 vx_set_channel;
	int16_t vx_channel;	      //遥控器原始通道值
	
	rc_deadline_limit(udtrans_collect_tiny_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//通道值赋值
	vx_set_channel = vx_channel;
	
#ifdef COLLECT_CONVERSION_AUTO	 	
	//第一步：夹取装置打开，抬升，金块抬升       
	if(trans_step == 1)      
	{
		Clamp_on();              //夹取装置打开  
		camera_screen();         //图传显视频位置
		
		//抬升
		udtrans_collect_tiny_control->upmotor_L.motor_angle_set = udtrans_collect_tiny_control->upmotor_L.max_angle_limit;   
		//set_ramp_angle为缓慢角度  
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle != udtrans_collect_tiny_control->upmotor_L.motor_angle_set)
		{
			//set_ramp_angle会慢慢变为目标角度
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->upmotor_L.motor_angle_set, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}	
		
    //金块抬升到相应位置，防止金块落下卡住		
		switch (collect_gold)         
		{
			case 0:
			{
	    	udtrans_collect_tiny_control->goldmotor.motor_angle_set = udtrans_collect_tiny_control->goldmotor.max_angle_limit; 		
				break;
			}
			case 1:
	 	 	{
		  	udtrans_collect_tiny_control->goldmotor.motor_angle_set = udtrans_collect_tiny_control->goldmotor.mid_angle_limit; 	
				break;
			}
			case 2:
			{
		  	udtrans_collect_tiny_control->goldmotor.motor_angle_set = udtrans_collect_tiny_control->goldmotor.min_angle_limit; 	
				break;
			}
		}
		
		if(udtrans_collect_tiny_control->goldmotor.set_ramp_angle != udtrans_collect_tiny_control->goldmotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->goldmotor.motor_angle_set, udtrans_collect_tiny_control->goldmotor.set_ramp_angle, 
			                                                                    GOLD_UP_SPEED);
		}
		
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle   == udtrans_collect_tiny_control->upmotor_L.motor_angle_set &&
			 udtrans_collect_tiny_control->goldmotor.set_ramp_angle   == udtrans_collect_tiny_control->goldmotor.motor_angle_set)
		{
			 trans_step = 2;
		}
	}
	//第二步：夹取装置旋转到金块位置
	else if(trans_step == 2)  
	{
		//夹取装置旋转到金块位置
		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = udtrans_collect_tiny_control->rotatemotor.max_angle_limit; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_FAST);
		}	

		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			 trans_step = 3;
		}			
	}		
	//第三步：等待操作手调整位置，手动调节，按下鼠标左键进入第二步  
	else if(trans_step == 3)  
	{	
		//遥控器左通道在下，控制夹取装置旋转电机
		if(switch_is_down(udtrans_collect_tiny_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		//遥控器左通道在中，控制抬升电机
		if(switch_is_mid(udtrans_collect_tiny_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		
		//按下鼠标左键进入下一步
		if (IF_MOUSE_PRESSED_LEFT)
		{		
      trans_step = 4;
		}	
	}        
	//第四步：夹取装置夹紧           
	else if(trans_step == 4)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART)			Clamp_close();        //夹取装置夹紧 
		if(trans_time == 2*COLLECT_TIME_PART)
		{
			trans_step = 5;
			trans_time = 0;		
		}
	}
	//第五步：夹取装置旋转中间位置，底盘晃动出来后，按下鼠标左键进入下一步          
	else if(trans_step == 5)
	{
		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = udtrans_collect_tiny_control->rotatemotor.mid_angle_limit; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}

		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			//晃出来后，按下鼠标左键进入下一步
			if (IF_MOUSE_PRESSED_LEFT)
			{
				switch (collect_gold)         
				{
					case 0:
					{
						collect_gold ++;    //拾取金块个数+1   
						trans_step = 6; 				
						break;
					}
					case 1:
					{
						collect_gold ++;    //拾取金块个数+1   
						trans_step = 6; 				
						break;
					}
					case 2:
					{
						collect_gold ++;    //拾取金块个数+1   
						trans_step = 8;
						break;
					}
				}				
			}
			else if(IF_KEY_PRESSED_Z)  
			{
			  trans_step = 1;   //没有夹到，鼠标右键回到第二步
			} 
		}
	}
	//第六步：抬升落到相应位置，夹取装置旋转归位
	else if(trans_step == 6)
	{	
		switch (collect_gold)         
		{
			case 1:
			{
  	  	udtrans_collect_tiny_control->upmotor_L.motor_angle_set = udtrans_collect_tiny_control->upmotor_L.min_angle_limit;   
				break;
			}
			case 2:
			{
  	  	udtrans_collect_tiny_control->upmotor_L.motor_angle_set = udtrans_collect_tiny_control->upmotor_L.mid_angle_limit;  		
				break;
			}
		}	
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle != udtrans_collect_tiny_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->upmotor_L.motor_angle_set, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}			

		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = udtrans_collect_tiny_control->rotatemotor.min_angle_limit; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle   == udtrans_collect_tiny_control->upmotor_L.motor_angle_set   &&
		   udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set )
		{
			trans_step = 7;        
		} 
	}
	//第七步：夹取装置松开，金块落入
	else if(trans_step == 7)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART) 	Clamp_on();       //夹取装置松开
		if(trans_time == 2*COLLECT_TIME_PART) 
		{
			trans_step = 1;   //回到第一步
			trans_time = 0;				
		}	
	}
	//第八步：抬升落下（举起金块）
	else if(trans_step == 8)
	{
    //抬升升起一定高度，保证内部两个金块不会甩出
	  udtrans_collect_tiny_control->upmotor_L.motor_angle_set = COLLECT_UP_GOLD_OVER; 	
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle != udtrans_collect_tiny_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->upmotor_L.motor_angle_set, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}
		
		//抓手到达举起高度
		udtrans_collect_tiny_control->rotatemotor.motor_angle_set = COLLECT_HAND_PICKUP_HEIGHT; 
		if(udtrans_collect_tiny_control->rotatemotor.set_ramp_angle != udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_tiny_control->rotatemotor.motor_angle_set, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		if(udtrans_collect_tiny_control->upmotor_L.set_ramp_angle == udtrans_collect_tiny_control->upmotor_L.motor_angle_set &&
			 udtrans_collect_tiny_control->rotatemotor.set_ramp_angle == udtrans_collect_tiny_control->rotatemotor.motor_angle_set)
		{				
			trans_time++;
			//循环一段时间，保证抬升回到初始位置
			if( trans_time == 500 )  
			{
				//金块拾取完成，切换为停止模式，步骤重置为1，计时归零，金块数量赋值，拾取金块数归零
				udtrans_collect_tiny_control->uptrans_behaviour = UPTRANS_STOP; 
				trans_step = 1;
				trans_time = 0;		
	  	  gold_number = collect_gold;	    
	    	collect_gold = 0;			           	
				camera_mid();           //图传回到中间位置					
			}
		}
	}

#else
//这里写第二种取金块代码	
#endif

  //set_ramp_angle限幅函数
	udtrans_collect_tiny_control->upmotor_L.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_tiny_control->upmotor_L);
	udtrans_collect_tiny_control->rotatemotor.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_tiny_control->rotatemotor);
	
	//抬升电机PID计算
	angle_out = PID_Calc(&udtrans_collect_tiny_control->upmotor_L.motor_angle_pid, udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, udtrans_collect_tiny_control->upmotor_L.motor_angle);
	udtrans_collect_tiny_control->upmotor_L.given_current = PID_Calc(&udtrans_collect_tiny_control->upmotor_L.motor_speed_pid, udtrans_collect_tiny_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_collect_tiny_control->upmotor_R.motor_angle_pid,-udtrans_collect_tiny_control->upmotor_L.set_ramp_angle, udtrans_collect_tiny_control->upmotor_R.motor_angle);
	udtrans_collect_tiny_control->upmotor_R.given_current = PID_Calc(&udtrans_collect_tiny_control->upmotor_R.motor_speed_pid, udtrans_collect_tiny_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_collect_tiny_control->rotatemotor.motor_angle_pid, udtrans_collect_tiny_control->rotatemotor.set_ramp_angle, udtrans_collect_tiny_control->rotatemotor.motor_angle);
	udtrans_collect_tiny_control->rotatemotor.given_current = PID_Calc(&udtrans_collect_tiny_control->rotatemotor.motor_speed_pid, udtrans_collect_tiny_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	angle_out = PID_Calc(&udtrans_collect_tiny_control->goldmotor.motor_angle_pid, udtrans_collect_tiny_control->goldmotor.set_ramp_angle, udtrans_collect_tiny_control->goldmotor.motor_angle);
	udtrans_collect_tiny_control->goldmotor.given_current = PID_Calc(&udtrans_collect_tiny_control->goldmotor.motor_speed_pid, udtrans_collect_tiny_control->goldmotor.motor_speed, angle_out);	
  //障碍块电机PID计算
	//由于取金块过程中用不到障碍块电机，角度设置不变（motor_angle_set）
	angle_out = PID_Calc(&udtrans_collect_tiny_control->snagmotor_L.motor_angle_pid, udtrans_collect_tiny_control->snagmotor_L.motor_angle_set, udtrans_collect_tiny_control->snagmotor_L.motor_angle);
	udtrans_collect_tiny_control->snagmotor_L.given_current = PID_Calc(&udtrans_collect_tiny_control->snagmotor_L.motor_speed_pid, udtrans_collect_tiny_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	//snagmotor_R.motor_angle_set可能不会改变，所以直接使snagmotor_L.motor_angle_set取负使用	
	angle_out = PID_Calc(&udtrans_collect_tiny_control->snagmotor_R.motor_angle_pid,-udtrans_collect_tiny_control->snagmotor_L.motor_angle_set, udtrans_collect_tiny_control->snagmotor_R.motor_angle);
	udtrans_collect_tiny_control->snagmotor_R.given_current = PID_Calc(&udtrans_collect_tiny_control->snagmotor_R.motor_speed_pid, udtrans_collect_tiny_control->snagmotor_R.motor_speed, angle_out);
}

//大资源岛，取两个，或三个，不需要抬升，夹取装置需要伸出
static void UPTrans_collect_huge_control(UDTrans_Control_t *udtrans_collect_huge_control)
{
	static fp32 angle_out;	
	static fp32 vx_set_channel;
	int16_t vx_channel;	 //遥控器原始通道值	
	
	rc_deadline_limit(udtrans_collect_huge_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//通道值赋值
	vx_set_channel = vx_channel;
	
#ifdef COLLECT_CONVERSION_AUTO	 	
	//第一步：夹取装置打开，并伸出，金块抬升最高高度         
	if(trans_step == 1)    
	{
		trans_time++;
		Clamp_on();              //夹取装置打开 
		camera_screen();         //图传显视频位置
		if(trans_time > COLLECT_TIME_PART)	  	
		{
			stretch_on();         //夹取装置伸出 
			
			//抬升升到目标高度
			udtrans_collect_huge_control->upmotor_L.motor_angle_set = COLLECT_HUGE_UP_HEIGHT_1;   
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
			{
				udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
																																						TRANS_UP_SPEED);
			}			
      //金块抬升到相应位置，防止金块落下卡住		
			switch (collect_gold)         
			{
				case 0:
				{
					udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.max_angle_limit; 		
					break;
				}
				case 1:
				{
					udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.min_angle_limit; 	
					break;
				}
				case 2:
				{
					udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.min_angle_limit; 	
					break;
				}
			} 	
			
			if(udtrans_collect_huge_control->goldmotor.set_ramp_angle != udtrans_collect_huge_control->goldmotor.motor_angle_set)
			{
				udtrans_collect_huge_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->goldmotor.motor_angle_set, udtrans_collect_huge_control->goldmotor.set_ramp_angle, 
				                                                                    GOLD_UP_SPEED);
			}	
			
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set &&
				 udtrans_collect_huge_control->goldmotor.set_ramp_angle == udtrans_collect_huge_control->goldmotor.motor_angle_set  )
			{
				trans_step = 2;   //进入第二步
			  trans_time = 0;
			}
		}
	}
	//第二步：夹取装置旋转到金块位置
	else if(trans_step == 2)  
	{	
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = COLLECT_HUGE_HAND; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_FAST);
		}	
		
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
		  trans_step = 3;   			
		} 	
	}
	//第三步：等待操作手调整位置，手动调节，按下鼠标左键进入下一步 
	else if(trans_step == 3)  
	{	
		//遥控器左通道在下，控制夹取装置旋转电机
		if(switch_is_down(udtrans_collect_huge_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		//遥控器左通道在中，控制抬升电机
		if(switch_is_mid(udtrans_collect_huge_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_collect_huge_control->upmotor_L.set_ramp_angle += vx_set_channel * 0.3f; 
		}		
		
		//按下鼠标左键进入下一步
		if (IF_MOUSE_PRESSED_LEFT)
		{		
      trans_step = 4;
		}	
	} 
	//第四步：夹取装置夹紧            
	else if(trans_step == 4)
	{
		trans_time++;
		Clamp_close();        //夹取装置夹紧 
		if(trans_time > COLLECT_TIME_PART)		
		{
			//抬升上升一段高度
			udtrans_collect_huge_control->upmotor_L.motor_angle_set = COLLECT_HUGE_UP_HEIGHT_2;   
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
			{
				udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
																																						TRANS_UP_SPEED);
			}		
			
			if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set)
			{
				trans_step = 5;
				trans_time = 0;				
			}		
		}
	}      
	//第五步：夹取装置旋转至中间位置
	else if(trans_step == 5)
	{
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = COLLECT_HAND_PICKUP_HEIGHT; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			
			camera_mid();     //图传看是否还有其他金矿
			
			if(IF_MOUSE_PRESSED_LEFT)  //还有金矿，取下一个
			{					
				switch (collect_gold)         
				{
					case 0:
					{
	      		collect_gold ++;						
						trans_step = 6; 				
						break;
					}
					case 1:
					{
		      	collect_gold ++;
						trans_step = 6; 				
						break;
					}
					case 2:     //取了三个跳到第九步
					{
	      		collect_gold ++;
						trans_step = 9;
						break;
					}
				}													
			}
			else if(IF_MOUSE_PRESSED_RIGH)  //没有金矿，举起金块
			{	
	      collect_gold ++;
				trans_step = 9;	
			}		
			else if(IF_KEY_PRESSED_X)  
			{
			  trans_step = 1;          //没有夹到，X键回到第一步
			} 				
		}
	}
	//第六步：夹取装置收回，判断要进行哪一步 
	else if(trans_step == 6)
	{
		trans_time++;
		if(trans_time ==    COLLECT_TIME_PART)			stretch_close();       //夹取装置收回
		if(trans_time ==  2*COLLECT_TIME_PART)
		{
			trans_step = 7;	
			trans_time = 0;					
		}		

	}
	//第七步：夹取装置旋转归位同时抬升回到最小高度
	else if(trans_step == 7)
	{	
		//夹取装置旋转归位
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = udtrans_collect_huge_control->rotatemotor.min_angle_limit; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}

		//抬升回到最小高度
		udtrans_collect_huge_control->upmotor_L.motor_angle_set = udtrans_collect_huge_control->upmotor_L.min_angle_limit;   
		if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}	
		
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set &&
			 udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set)
		{
      trans_step = 8;			
		}
	}
	//第八步：夹取装置松开，金块落入，完成一次拾取
	else if(trans_step == 8)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART)		Clamp_on();        //夹取装置松开	
		if(trans_time == 2*COLLECT_TIME_PART)
		{
			trans_step = 1;	
			trans_time = 0;			
		}	
	}	
	//第九步：夹取装置收回
	else if(trans_step == 9)
	{
		trans_time++;
		if(trans_time ==   COLLECT_TIME_PART)		stretch_close();   //夹取装置收回	
		if(trans_time == 2*COLLECT_TIME_PART)
		{
			trans_step = 10;	
			trans_time = 0;			
		}	
	}	
	//第九步：抬升落下，金块抬升落下
	else if(trans_step == 10)
	{
		//抬升到达对应高度
		switch (collect_gold)         
		{
			case 1:  //取了一个，抬升回到最小高度
			{
			  udtrans_collect_huge_control->upmotor_L.motor_angle_set = udtrans_collect_huge_control->upmotor_L.min_angle_limit; 
				break;
			}
			case 2:  //取了两个，抬升回到最小高度
			{
	  		udtrans_collect_huge_control->upmotor_L.motor_angle_set = udtrans_collect_huge_control->upmotor_L.min_angle_limit; 
				break;
			}
			case 3:  //取了三个，抬升升起一定高度，保证内部两个金块不会甩出
			{
		    udtrans_collect_huge_control->upmotor_L.motor_angle_set = COLLECT_UP_GOLD_OVER; 		
				break;
			}
		}			

		if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle != udtrans_collect_huge_control->upmotor_L.motor_angle_set)
		{
			udtrans_collect_huge_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->upmotor_L.motor_angle_set, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, 
			                                                                    TRANS_UP_SPEED);
		}

		//抓手到达举起高度
		udtrans_collect_huge_control->rotatemotor.motor_angle_set = COLLECT_HAND_PICKUP_HEIGHT; 
		if(udtrans_collect_huge_control->rotatemotor.set_ramp_angle != udtrans_collect_huge_control->rotatemotor.motor_angle_set)
		{
			udtrans_collect_huge_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->rotatemotor.motor_angle_set, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, 
			                                                                      HAND_SLOW);
		}
		
		udtrans_collect_huge_control->goldmotor.motor_angle_set = udtrans_collect_huge_control->goldmotor.min_angle_limit; 			
		if(udtrans_collect_huge_control->goldmotor.set_ramp_angle != udtrans_collect_huge_control->goldmotor.motor_angle_set)
		{
			udtrans_collect_huge_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_collect_huge_control->goldmotor.motor_angle_set, udtrans_collect_huge_control->goldmotor.set_ramp_angle, 
			                                                                    GOLD_UP_SPEED);
		}	
		
		if(udtrans_collect_huge_control->upmotor_L.set_ramp_angle == udtrans_collect_huge_control->upmotor_L.motor_angle_set     &&
		   udtrans_collect_huge_control->rotatemotor.set_ramp_angle == udtrans_collect_huge_control->rotatemotor.motor_angle_set &&
			 udtrans_collect_huge_control->goldmotor.set_ramp_angle == udtrans_collect_huge_control->goldmotor.motor_angle_set )
		{	
			trans_time++;
			//循环一段时间，保证回到初始位置
			if( trans_time == 500 )  
			{
				//金块拾取完成，切换为停止模式，步骤重置为1，计时归零，金块数量赋值，拾取金块数归零
				udtrans_collect_huge_control->uptrans_behaviour = UPTRANS_STOP; 
				trans_step = 1;
				trans_time = 0;
	  	  gold_number = collect_gold;	      
	    	collect_gold = 0;			            
				camera_mid();           //图传回到中间位置				
			}			
		}
	}
#else
//这里写第二种取金块代码	
#endif
	
  //set_ramp_angle限幅函数
	udtrans_collect_huge_control->upmotor_L.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_huge_control->upmotor_L);
	udtrans_collect_huge_control->rotatemotor.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_collect_huge_control->rotatemotor);
	
	//抬升电机PID计算
	angle_out = PID_Calc(&udtrans_collect_huge_control->upmotor_L.motor_angle_pid, udtrans_collect_huge_control->upmotor_L.set_ramp_angle, udtrans_collect_huge_control->upmotor_L.motor_angle);
	udtrans_collect_huge_control->upmotor_L.given_current = PID_Calc(&udtrans_collect_huge_control->upmotor_L.motor_speed_pid, udtrans_collect_huge_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_collect_huge_control->upmotor_R.motor_angle_pid,-udtrans_collect_huge_control->upmotor_L.set_ramp_angle, udtrans_collect_huge_control->upmotor_R.motor_angle);
	udtrans_collect_huge_control->upmotor_R.given_current = PID_Calc(&udtrans_collect_huge_control->upmotor_R.motor_speed_pid, udtrans_collect_huge_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_collect_huge_control->rotatemotor.motor_angle_pid, udtrans_collect_huge_control->rotatemotor.set_ramp_angle, udtrans_collect_huge_control->rotatemotor.motor_angle);
	udtrans_collect_huge_control->rotatemotor.given_current = PID_Calc(&udtrans_collect_huge_control->rotatemotor.motor_speed_pid, udtrans_collect_huge_control->rotatemotor.motor_speed, angle_out);	
	//金块抬升电机PID计算
	angle_out = PID_Calc(&udtrans_collect_huge_control->goldmotor.motor_angle_pid, udtrans_collect_huge_control->goldmotor.set_ramp_angle, udtrans_collect_huge_control->goldmotor.motor_angle);
	udtrans_collect_huge_control->goldmotor.given_current = PID_Calc(&udtrans_collect_huge_control->goldmotor.motor_speed_pid, udtrans_collect_huge_control->goldmotor.motor_speed, angle_out);
  //障碍块电机PID计算
	//由于取金块过程中用不到障碍块电机，角度设置不变（motor_angle_set）
	angle_out = PID_Calc(&udtrans_collect_huge_control->snagmotor_L.motor_angle_pid, udtrans_collect_huge_control->snagmotor_L.motor_angle_set, udtrans_collect_huge_control->snagmotor_L.motor_angle);
	udtrans_collect_huge_control->snagmotor_L.given_current = PID_Calc(&udtrans_collect_huge_control->snagmotor_L.motor_speed_pid, udtrans_collect_huge_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	//snagmotor_R.motor_angle_set可能不会改变，所以直接使snagmotor_L.motor_angle_set取负使用	
	angle_out = PID_Calc(&udtrans_collect_huge_control->snagmotor_R.motor_angle_pid,-udtrans_collect_huge_control->snagmotor_L.motor_angle_set, udtrans_collect_huge_control->snagmotor_R.motor_angle);
	udtrans_collect_huge_control->snagmotor_R.given_current = PID_Calc(&udtrans_collect_huge_control->snagmotor_R.motor_speed_pid, udtrans_collect_huge_control->snagmotor_R.motor_speed, angle_out);
}

//金块兑换，抬升100，夹取装置伸出
static void UPTrans_conversion_control(UDTrans_Control_t *udtrans_conversion_control)
{
	static fp32 angle_out;
	static fp32 vx_set_channel;
	int16_t vx_channel;	 //遥控器原始通道值	
	
	//死区限制，因为遥控器可能存在差异，摇杆在中间，其值不为0
	rc_deadline_limit(udtrans_conversion_control->udtrans_rc_ctrl->rc.ch[3], vx_channel, UPTRANS_RC_DEADLINE);
	//通道值赋值
	vx_set_channel = vx_channel;

#ifdef COLLECT_CONVERSION_AUTO	 		
	//第一步：抬升到扫描二维码高度
	if(trans_step == 1)       
	{
    camera_up();     //图传上
		udtrans_conversion_control->upmotor_L.motor_angle_set = udtrans_conversion_control->upmotor_L.max_angle_limit;    
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
			                                                                  TRANS_UP_SPEED);
		}
		
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle == udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			trans_step = 2;   
		}
	}
	//第二步：夹取装置伸出
	else if(trans_step == 2)  
	{
		trans_time++;
		if(trans_time ==   CONVERSION_TIME_PART)		stretch_on();         //夹取装置伸出   
		if(trans_time == 2*CONVERSION_TIME_PART)	  
		{
			trans_step = 3; 
      trans_time = 0;	
		}			
	}	
  //第三步：夹取装置旋转至兑换台扫码高度
	else if(trans_step == 3)  
	{
		udtrans_conversion_control->rotatemotor.motor_angle_set = CONVERSION_HAND_SCAN; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_SLOW);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set)
		{ 			
     	trans_step = 4;	
		} 
	}
	//第四步：扫码，遥控器调节高度
	else if(trans_step == 4)  
	{
		trans_time++;	
		
		//遥控器左通道在下，控制夹取装置旋转电机
		if(switch_is_down(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle += vx_set_channel * 0.3f; 
		}
		//遥控器左通道在中，控制抬升电机
		if(switch_is_mid(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle += vx_set_channel * 0.3f; 
		}
				
		
		if (trans_time > CONVERSION_TIME_PART)
		{
			//按下鼠标左键进入下一步
			if (IF_MOUSE_PRESSED_LEFT)
			{		
				Clamp_on();          //夹取装置松开 
				camera_screen();     //图传显示屏
				trans_time = 0;
				trans_step = 5;	
			}		
			//按下C键回到第七步，取存储的金矿
			if (IF_KEY_PRESSED_C)
			{		
				trans_time = 0;
				trans_step = 7;	
			}		
			//按下Z键金块数量设为2
			if (IF_KEY_PRESSED_Z)
			{	
        conversion_gold = 0;							
        gold_number = 2;
			}		
			//按下X键金块数量设为3
			if (IF_KEY_PRESSED_X)
			{	
				conversion_gold = 0;				
        gold_number = 3;
			}				
		}

	}
	//第五步：抬升到手动调节高度，夹取装置收回并夹紧
	else if(trans_step == 5)  
	{
		trans_time++;
		
		if (trans_time >= CONVERSION_TIME_PART)
		{	
			//抬升下降到手动调节高度
			udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_ADJUST;    
			if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
			{
				udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
				                                                                  TRANS_UP_SPEED);
			}
			//抓手电机回到最大位置
			udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.max_angle_limit; 
			if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
			{
				udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
				                                                                    HAND_FAST);
			}
		}
		
		if(trans_time ==    CONVERSION_TIME_PART)
		{
			stretch_close();     //夹取装置收回
		}			
		if(trans_time ==  2*CONVERSION_TIME_PART)	  		
		{  
			Clamp_close();       //夹取装置夹紧	
		  trans_step = 6; 
      trans_time = 0;			
		} 

	}
	//第六步：进入手动推金块状态同时金块抬升到相应位置
	else if(trans_step == 6)  
	{ 
		//判断上次为哪种资源岛
		switch (gold_number)
		{	
			case 3:  //存了两个
			{
				if(conversion_gold == 0)      
				{
					//金块升到中间位置
					udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.mid_angle_limit; 	
				}
				else if(conversion_gold == 1)  
				{
					//金块升到最高位置
					udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.max_angle_limit; 	
				}
				break;
			}
			case 2:	 //存了一个	
			{  
				//金块升到最高位置
				udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.max_angle_limit; 	
				break;
			}
		}
		
		if(udtrans_conversion_control->goldmotor.set_ramp_angle != udtrans_conversion_control->goldmotor.motor_angle_set)
		{
			udtrans_conversion_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->goldmotor.motor_angle_set, udtrans_conversion_control->goldmotor.set_ramp_angle, 
			                                                                  GOLD_UP_SPEED);
		}	

		//遥控器左通道在下，控制夹取装置旋转电机
		if(switch_is_down(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			//设置目标角度（累加）
			udtrans_conversion_control->rotatemotor.set_ramp_angle += vx_set_channel*0.3f; 
		}
		//遥控器左通道在中，控制抬升电机
		if(switch_is_mid(udtrans_conversion_control->udtrans_rc_ctrl->rc.s[1]))
		{
			//设置目标角度（累加）
			udtrans_conversion_control->upmotor_L.set_ramp_angle += vx_set_channel*0.3f; 
		}

		//夹取装置伸缩控制：
		//鼠标右键检测
		if (IF_MOUSE_PRESSED_RIGH)
		{
			if (udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time < RC_KEY_SHOT_TIME)
			{
				udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time++;
				if(udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time == RC_KEY_SHOT_TIME)
				{
					//鼠标右键检测	
					switch (conversion_mouse)         
					{
						case 1:
						{
							stretch_on();    //夹取装置伸出							
							conversion_mouse = 0;
							break;
						}
						case 0:
						{
							stretch_close(); //夹取装置收回
							conversion_mouse = 1;
							break;
						}
					}
					udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
				}
			}
		}
		else
		{
			udtrans_conversion_control->RC_udtrans_ctrl_time.mouse_right_time = 0;
		}	
	
		//按下鼠标左键进入下一步
		if (IF_MOUSE_PRESSED_LEFT)
		{		
			conversion_gold++;            //兑换个数加一
			conversion_mouse = 1;         //鼠标左键标志归零
			if(conversion_gold == 1 )     //完成第一次兑换
			{
				switch (gold_number)
		    {	
		     	case 1:  trans_step = 11; break;   //取了一个，跳到第十一步，直接完事
					case 2:  trans_step = 7;  break;   //取了两个，跳到第七步
					case 3:  trans_step = 7;  break;   //取了两个，跳到第七步
		   	}				
			}
			else if(conversion_gold == 2) 				
			{
				switch (gold_number)
		    {	
					case 2:  trans_step = 11; break;  //取了两个，跳到第十一步，直接完事
		     	case 3:  trans_step = 7;  break;  //取了三个，跳到第七步
		   	}			   
			}
			else if(conversion_gold == 3)
			{
				trans_step = 11;       //完成三次兑换，跳到第十一步	
			}
		}		
	}
	//第七步：夹取装置打开收回，旋转归位
	else if(trans_step == 7)  
	{
	  Clamp_on();           //夹取装置松开	
		trans_time++;		
		if(trans_time == CONVERSION_TIME_PART)	
		{
			stretch_close();    //夹取装置收回			
		}				
		
		udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.min_angle_limit; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_SLOW);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set &&
			 trans_time > 2*CONVERSION_TIME_PART)
		{ 
      trans_step = 8;		
      trans_time = 0;					
		} 	

	}
	//第八步：抬升到达金块存储高度
	else if(trans_step == 8) 
	{
		switch (gold_number)
		{	
			case 3:  //存了两个
			{
				if(conversion_gold == 1)      
				{
					//抬升到达第一个金块高度
					udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_PICKUP1;	
				}
				else if(conversion_gold == 2)  
				{
					//抬升到达第二个金块高度
					udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_PICKUP2;	
				}
				break;
			}
			case 2:	 //存了一个，直接到第二个金块高度
			{  
				udtrans_conversion_control->upmotor_L.motor_angle_set = CONVERSION_UP_PICKUP2;	
				break;
			}
		}
		    
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
		                                                                  	TRANS_UP_SPEED);
		}

		if(udtrans_conversion_control->upmotor_L.set_ramp_angle == udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
      trans_step = 9;				
		}
		
	}			
	//第九步：夹取装置夹紧
	else if(trans_step == 9)  
	{
		trans_time++;		
		if(trans_time ==   CONVERSION_TIME_PART)	  Clamp_close();        //夹取装置夹紧	
		if(trans_time == 2*CONVERSION_TIME_PART)				
		{
      trans_step = 10;     
      trans_time = 0;			
		}			
	}
	//第十步：夹取装置旋转至中间位置
	else if(trans_step == 10)  
	{
		udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.mid_angle_limit; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_SLOW);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set)
		{ 
			trans_step = 1;     //回到第一步
		} 
	}
	//第十一步：夹取装置收回松开
	else if(trans_step == 11)  
	{
		camera_mid();           //图传回到中间位置
		trans_time++;		
		if(trans_time ==   CONVERSION_TIME_PART)	   
		{
			Clamp_on();         //夹取装置松开	
			stretch_close();    //夹取装置收回				
		}
		if(trans_time == 2*CONVERSION_TIME_PART)    		
		{  
      trans_step = 12;
      trans_time = 0;			
		}  
	}
	//第十二步：夹取装置旋转归位抬升落下金块抬升落下
	else if(trans_step == 12)  
	{
		udtrans_conversion_control->rotatemotor.motor_angle_set = udtrans_conversion_control->rotatemotor.min_angle_limit; 
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle != udtrans_conversion_control->rotatemotor.motor_angle_set)
		{
			udtrans_conversion_control->rotatemotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->rotatemotor.motor_angle_set, udtrans_conversion_control->rotatemotor.set_ramp_angle, 
			                                                                    HAND_FAST);
		}
		
		udtrans_conversion_control->upmotor_L.motor_angle_set = udtrans_conversion_control->upmotor_L.min_angle_limit; 
		if(udtrans_conversion_control->upmotor_L.set_ramp_angle != udtrans_conversion_control->upmotor_L.motor_angle_set)
		{
			udtrans_conversion_control->upmotor_L.set_ramp_angle = RAMP_float(udtrans_conversion_control->upmotor_L.motor_angle_set, udtrans_conversion_control->upmotor_L.set_ramp_angle, 
			                                                                  TRANS_UP_SPEED);
		}

		udtrans_conversion_control->goldmotor.motor_angle_set = udtrans_conversion_control->goldmotor.min_angle_limit; 
		if(udtrans_conversion_control->goldmotor.set_ramp_angle != udtrans_conversion_control->goldmotor.motor_angle_set)
		{
			udtrans_conversion_control->goldmotor.set_ramp_angle = RAMP_float(udtrans_conversion_control->goldmotor.motor_angle_set, udtrans_conversion_control->goldmotor.set_ramp_angle, 
			                                                                  GOLD_UP_SPEED);
		}
		
		if(udtrans_conversion_control->rotatemotor.set_ramp_angle == udtrans_conversion_control->rotatemotor.motor_angle_set &&
			 udtrans_conversion_control->upmotor_L.set_ramp_angle == udtrans_conversion_control->upmotor_L.motor_angle_set     &&
			 udtrans_conversion_control->goldmotor.set_ramp_angle == udtrans_conversion_control->goldmotor.motor_angle_set)
		{		
			trans_time++;
			//循环一段时间，保证抬升回到初始位置
			if( trans_time == 500 )  
			{
		  	//兑换完成，切换为停止模式，步骤重置为1，计时归零，兑换金块数归零，鼠标右键控制夹取装置伸缩标志位重置为1
		  	udtrans_conversion_control->uptrans_behaviour = UPTRANS_STOP; 
		  	trans_step = 1;
		  	trans_time = 0;
		  	conversion_gold = 0;
	    	conversion_mouse = 1;   //鼠标左键标志归零
			}			
		}
	}
#else
//在这里写第二种兑换代码
#endif
	
  //set_ramp_angle限幅函数
	udtrans_conversion_control->upmotor_L.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_conversion_control->upmotor_L);
	udtrans_conversion_control->rotatemotor.set_ramp_angle = fp32_constrain_upmotor_ramp(&udtrans_conversion_control->rotatemotor);
	
	//抬升电机PID计算
	angle_out = PID_Calc(&udtrans_conversion_control->upmotor_L.motor_angle_pid, udtrans_conversion_control->upmotor_L.set_ramp_angle, udtrans_conversion_control->upmotor_L.motor_angle);
	udtrans_conversion_control->upmotor_L.given_current = PID_Calc(&udtrans_conversion_control->upmotor_L.motor_speed_pid, udtrans_conversion_control->upmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	angle_out = PID_Calc(&udtrans_conversion_control->upmotor_R.motor_angle_pid,-udtrans_conversion_control->upmotor_L.set_ramp_angle, udtrans_conversion_control->upmotor_R.motor_angle);
	udtrans_conversion_control->upmotor_R.given_current = PID_Calc(&udtrans_conversion_control->upmotor_R.motor_speed_pid, udtrans_conversion_control->upmotor_R.motor_speed, angle_out);
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_conversion_control->rotatemotor.motor_angle_pid, udtrans_conversion_control->rotatemotor.set_ramp_angle, udtrans_conversion_control->rotatemotor.motor_angle);
	udtrans_conversion_control->rotatemotor.given_current = PID_Calc(&udtrans_conversion_control->rotatemotor.motor_speed_pid, udtrans_conversion_control->rotatemotor.motor_speed, angle_out);	
	//抓手旋转电机PID计算
	angle_out = PID_Calc(&udtrans_conversion_control->goldmotor.motor_angle_pid, udtrans_conversion_control->goldmotor.set_ramp_angle, udtrans_conversion_control->goldmotor.motor_angle);
	udtrans_conversion_control->goldmotor.given_current = PID_Calc(&udtrans_conversion_control->goldmotor.motor_speed_pid, udtrans_conversion_control->goldmotor.motor_speed, angle_out);	
  //障碍块电机PID计算
	//由于取金块过程中用不到障碍块电机，角度设置不变（motor_angle_set）
	angle_out = PID_Calc(&udtrans_conversion_control->snagmotor_L.motor_angle_pid, udtrans_conversion_control->snagmotor_L.motor_angle_set, udtrans_conversion_control->snagmotor_L.motor_angle);
	udtrans_conversion_control->snagmotor_L.given_current = PID_Calc(&udtrans_conversion_control->snagmotor_L.motor_speed_pid, udtrans_conversion_control->snagmotor_L.motor_speed, angle_out);			
	//电机反向 这里直接用另一个电机角度取负使用	
	//snagmotor_R.motor_angle_set可能不会改变，所以直接使snagmotor_L.motor_angle_set取负使用	
	angle_out = PID_Calc(&udtrans_conversion_control->snagmotor_R.motor_angle_pid,-udtrans_conversion_control->snagmotor_L.motor_angle_set, udtrans_conversion_control->snagmotor_R.motor_angle);
	udtrans_conversion_control->snagmotor_R.given_current = PID_Calc(&udtrans_conversion_control->snagmotor_R.motor_speed_pid, udtrans_conversion_control->snagmotor_R.motor_speed, angle_out);
}

//编码值加和函数
static void UPTRANS_UpdateMotorAngleSum(UPTrans_Motor_t *UPTrans_Motor)
{
	if (abs(UPTrans_Motor->uptrans_motor_measure->ecd - UPTrans_Motor->uptrans_motor_measure->last_ecd) > 4095)
	{
		if (UPTrans_Motor->uptrans_motor_measure->ecd < UPTrans_Motor->uptrans_motor_measure->last_ecd)
		{
			UPTrans_Motor->motor_angle += 8191 -  UPTrans_Motor->uptrans_motor_measure->last_ecd + UPTrans_Motor->uptrans_motor_measure->ecd;
		}
		else
		{
			UPTrans_Motor->motor_angle -= 8191 - UPTrans_Motor->uptrans_motor_measure->ecd + UPTrans_Motor->uptrans_motor_measure->last_ecd;
		}
	}
	else
	{
		UPTrans_Motor->motor_angle += UPTrans_Motor->uptrans_motor_measure->ecd - UPTrans_Motor->uptrans_motor_measure->last_ecd;
	}
}

//motor_angle_set限幅函数 
static fp32 fp32_constrain_upmotor_angle(UPTrans_Motor_t* motor)
{
	  if (motor->motor_angle_set < motor->min_angle_limit)
        return motor->min_angle_limit;
    else if (motor->motor_angle_set > motor->MAX_angle_limit)
        return motor->MAX_angle_limit;
    else
        return motor->motor_angle_set;
}

//set_ramp_angle限幅函数 
static fp32 fp32_constrain_upmotor_ramp(UPTrans_Motor_t* motor)
{
	  if (motor->set_ramp_angle < motor->min_angle_limit)
        return motor->min_angle_limit;
    else if (motor->set_ramp_angle > motor->MAX_angle_limit)
        return motor->MAX_angle_limit;
    else
        return motor->set_ramp_angle;
}

/********************************工程气缸和舵机使用函数************************************/
//夹取气缸初始打开状态 板子没有上电也是打开状态
void Clamp_on(void)                  //夹取装置松开（关）
{
	GPIO_ResetBits(GPIOI,GPIO_Pin_2);
	clamp_flag = 0;
}

void Clamp_close(void)               //夹取装置夹紧（开）
{
	GPIO_SetBits(GPIOI,GPIO_Pin_2);
	clamp_flag = 1;
}

void stretch_close(void)             //夹取装置收回（关）
{
	GPIO_ResetBits(GPIOI,GPIO_Pin_7);
	stretch_flag = 0;	
}

void stretch_on(void)                //夹取装置伸出（开）
{
	GPIO_SetBits(GPIOI,GPIO_Pin_7);
	stretch_flag = 1;	
}

void rescue_close(void)              //救援收回
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);
	help_flag = 0; 
}

void rescue_on(void)                 //救援伸出
{
	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	help_flag = 1; 
}

void rotary_actuator_close(void)     //除障关闭
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	snag_flag = 0;
}

void rotary_actuator_on(void)        //除障打开
{
	GPIO_SetBits(GPIOD,GPIO_Pin_13);
	snag_flag = 1;
}

void supply1_close(void)             //补给1关闭  PA0
{
  TIM_SetCompare1(TIM2,2050); 
  supply1_flag = 0;	
}

void supply1_on(void)                //补给1开启  
{
  TIM_SetCompare1(TIM2,2800);
  supply1_flag = 1;	
}

void supply1_disable(void)           //补给1失能  
{
  TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
  supply1_flag = 1;	
}

void supply2_close(void)             //补给2关闭  PA1
{
  TIM_SetCompare2(TIM2,2450);
	supply2_flag = 0;
}

void supply2_on(void)                //补给2开启  
{
  TIM_SetCompare2(TIM2,1850);
	supply2_flag = 1;
}

void supply2_disable(void)           //补给2失能  
{
  TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
  supply2_flag = 1;	
}

void camera_up(void)                 //图传上     PA3
{
  TIM_SetCompare3(TIM2,3150);
}

void camera_mid(void)                //图传中  正常位置
{
  TIM_SetCompare3(TIM2,2950);
}

void camera_down(void)               //图传下
{
  TIM_SetCompare3(TIM2,2400);
}

void camera_screen(void)             //图传显示屏
{
  TIM_SetCompare3(TIM2,2000);
}

/********************************工程气缸和舵机使用函数************************************/


