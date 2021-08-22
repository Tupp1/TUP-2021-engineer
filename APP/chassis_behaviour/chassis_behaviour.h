#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "chassis_task.h"

#define CHASSIS_OPEN_RC_SCALE 10         //��chassis_open ģ���£�ң�������Ըñ������͵�can��

#define CHASSIS_ANGLE_MOUSE_Sen  0.1f    //����û������ʱ��

#define CHASSIS_MODE_CHANGE 1000
#define CHASSIS_RETURN      300

//#define ENGINEER_TANK   //���빤��д��̹�ˣ����ʧ��   

extern int supply_flag;

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
