#ifndef __PID_H
#define __PID_H

#include "stm32h7xx_hal.h"

typedef struct{
	float kp,ki,kd;
	float integ;
	float prev_err;
	float out_max,out_min;
	float dt;
	float i_max,i_min;
}PID_t;

void PID_Init(PID_t *pid,float kp,float ki,float kd,float dt);
void PID_SetLimits(PID_t *pid,float out_max,float out_min,float i_max,float i_min);
void PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid,float target,float feedback);
float PID_Calc_Pos(PID_t *pid,float target,float feedback,float Speed);

	
#endif
