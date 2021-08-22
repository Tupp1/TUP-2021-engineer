#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "Gimbal_Task.h"


#define GIMBAL_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define GIMBAL_RIGHT_KEY KEY_PRESSED_OFFSET_E

#define GIMBAL_LOCK_ANGLE_ERROR 0.05f  //0.1


extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);
extern void gimbalpwm_behaviour_mode_set(Gimbal_Control_t *gimbalpwm_mode_set);

extern void gimbalpwm_behaviour_control_set(fp32 *add_yaw, Gimbal_Control_t *gimbalpwm_control_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, Gimbal_Control_t *gimbal_control_set);
extern bool_t gimbal_cmd_to_chassis_stop(void);

void lock_on(void);     //电磁锁开启
void lock_off(void);    //电磁锁关闭
#endif
