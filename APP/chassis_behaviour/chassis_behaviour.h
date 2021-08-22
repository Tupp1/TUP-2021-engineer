#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "chassis_task.h"

#define CHASSIS_OPEN_RC_SCALE 10         //在chassis_open 模型下，遥控器乘以该比例发送到can上

#define CHASSIS_ANGLE_MOUSE_Sen  0.1f    //工程没有锁的时候

#define CHASSIS_MODE_CHANGE 1000
#define CHASSIS_RETURN      300

//#define ENGINEER_TANK   //幻想工程写成坦克，结果失败   

extern int supply_flag;

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
