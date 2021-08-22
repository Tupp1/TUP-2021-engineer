#ifndef _SYS_CONFIG_H_
#define _SYS_CONFIG_H_



/*
1.У׼��ʱ����Ҫ�������ݶ����밴һ����ѹ��У׼
2.����pid��ʱ���Ȱ�д��ɿأ��ٰ���ѹ��У׼
*/

/******************************PID default parameter*****************/
////////////////////////////		kp		ki		kd		
#define CHASSIS_Rot_PID_OFF 	{10.0f,	0.0f,	0.0f}       //Ŀǰ�����ã������޷�ͨ����λ�����ã�
#define CHASSIS_Vec_PID_OFF 	{10000.0f,	0.0f,	0.0f}   //���̵�������ٶ�pid
#define GIMBALP_Pos_PID_OFF 	{8.0f,	0.0f,	0.0f}         
#define GIMBALP_Vec_PID_OFF 	{4000.0f,	0.0f,	0.0f}

#define GIMBALY_Pos_PID_OFF 	{12.0f,	0.0f,	0.0f}   
#define GIMBALY_Vec_PID_OFF 	{8000.0f,	0.0f,	0.0f}//pid������
#define SLIBLIN_Pos_PID_OFF 	{0.0f,	0.0f,	0.0f}
#define SLIBLIN_Vec_PID_OFF 	{1000.0f,	0.0f,	0.0f}  //���� �ٶȻ�


#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//#define SUPER_CAP_TEST
//#define ENGINEER

#define RADIAN_PER_RAD (57.29578f)
#endif


