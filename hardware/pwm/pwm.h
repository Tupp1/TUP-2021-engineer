#ifndef PWM_H
#define PWM_H
#include "main.h"

#define Fric_L_UP 1200
#define Fric_R_UP 950
#define Firc_UP 1250
#define Fric_DOWN 500
#define Fric_OFF 900
void TIM_PWM_Config(void);
extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif



