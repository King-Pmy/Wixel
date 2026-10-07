#include "pid.h"

void PID_Init(PID_t *pid,float kp,float ki,float kd,float dt)
{
	pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integ = 0.0f;
    pid->prev_err = 0.0f;
}
void PID_SetLimits(PID_t *pid,float max,float min)
{
	pid->out_max = max;
    pid->out_min = min;
}
void PID_Reset(PID_t *pid)
{
	pid->integ = 0.0f;
    pid->prev_err = 0.0f;
}
float PID_Calc(PID_t *pid,float target,float feedback)
{
	float err = target - feedback;  //计算误差//
	float p_out = pid->kp * err;
	float i_out = pid->integ + err * pid->dt * pid->ki; //计算积分//
	float d_out = (err - pid->prev_err) / pid->dt * pid->kd;
	float out_unclamped = p_out + i_out + d_out;
    float out = out_unclamped;
	if(out > pid->out_max) out = pid->out_max;  //输出限幅//
	if(out < pid->out_min) out = pid->out_min;
	if(out != out_unclamped){
		pid->integ = i_out - (out_unclamped - out); 
    } else {
        pid->integ = i_out;
    }
	//积分限幅？//
	pid->prev_err = err;   //保留误差//
	return out;
}
	


