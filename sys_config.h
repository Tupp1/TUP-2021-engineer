#ifndef _SYS_CONFIG_H_
#define _SYS_CONFIG_H_



/*
1.校准的时候想要保存数据都必须按一下气压计校准
2.设置pid的时候，先按写入飞控，再按气压计校准
*/

/******************************PID default parameter*****************/
////////////////////////////		kp		ki		kd		
#define CHASSIS_Rot_PID_OFF 	{10.0f,	0.0f,	0.0f}       //目前被弃用（现在无法通过上位机设置）
#define CHASSIS_Vec_PID_OFF 	{10000.0f,	0.0f,	0.0f}   //底盘单个电机速度pid
#define GIMBALP_Pos_PID_OFF 	{8.0f,	0.0f,	0.0f}         
#define GIMBALP_Vec_PID_OFF 	{4000.0f,	0.0f,	0.0f}

#define GIMBALY_Pos_PID_OFF 	{12.0f,	0.0f,	0.0f}   
#define GIMBALY_Vec_PID_OFF 	{8000.0f,	0.0f,	0.0f}//pid有问题
#define SLIBLIN_Pos_PID_OFF 	{0.0f,	0.0f,	0.0f}
#define SLIBLIN_Vec_PID_OFF 	{1000.0f,	0.0f,	0.0f}  //拨弹 速度环


#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//#define SUPER_CAP_TEST
//#define ENGINEER

#define RADIAN_PER_RAD (57.29578f)
#endif


