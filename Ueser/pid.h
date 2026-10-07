#ifndef __PID_H
#define __PID_H

#include "stm32h7xx_hal.h"

typedef struct{
	float kp,ki,kd;
	float integ;
	float prev_err;
	float out_max,out_min;
	float dt;
}PID_t;

void PID_Init(PID_t *pid,float kp,float ki,float kd,float dt);
void PID_SetLimits(PID_t *pid,float max,float min);
void PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid,float target,float feedback);

	
#endif
