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
void PID_SetLimits(PID_t *pid,float out_max,float out_min,float i_max,float i_min)
{
	pid->out_max = out_max;
    pid->out_min = out_min;
	pid->i_max = i_max;
	pid->i_min = i_min;
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
	float i_out; //计算积分//
	float d_out = (err - pid->prev_err) / pid->dt * pid->kd;
	float out_unclamped = p_out + d_out;
	int out_over = (out_unclamped > pid->out_max) && err > 0;
	int out_down = (out_unclamped < pid->out_min) && err < 0;
	if(!out_over && ! out_down){
		i_out = pid->integ + err * pid->dt * pid->ki; 		//条件积分  如果输出已经限幅，则冻结
		if(i_out > pid->i_max)i_out = pid->i_max;
		if(i_out < pid->i_min)i_out = pid->i_min;
	}
	float out = out_unclamped + i_out;
	if(out_over) out = pid->out_max;  //输出限幅//
	if(out_down) out = pid->out_min;

	pid->integ = i_out;
	pid->prev_err = err;   //保留误差//
	return out;
}
	
float PID_Calc_Pos(PID_t *pid,float target,float feedback,float Speed)
{
	float err = target - feedback;  //计算误差//
	float p_out = pid->kp * err;
	float i_out; //计算积分//
	float d_out = Speed * pid->kd;
	float out_unclamped = p_out - d_out;
	float DEADDEG = 5.0f;
	int out_over = (out_unclamped > pid->out_max) && err > 0;
	int out_down = (out_unclamped < pid->out_min) && err < 0;
	if(!out_over && ! out_down && (-DEADDEG < err && DEADDEG > err) == 0){
		i_out = pid->integ + err * pid->dt * pid->ki; 		//条件积分  如果输出已经限幅，则冻结
		if(i_out > pid->i_max)i_out = pid->i_max;
		if(i_out < pid->i_min)i_out = pid->i_min;
	}
	float out = out_unclamped + i_out;
	if(out_over) out = pid->out_max;  //输出限幅//
	if(out_down) out = pid->out_min;

	pid->integ = i_out;
	pid->prev_err = err;   //保留误差//
	return out;
}

//	if(out != out_unclamped){
//		pid->integ = i_out - (out_unclamped - out); 
//    } else {                            p_out过饱和反向限制integ————输出限幅只能砍输出，绝不能写状态。积分状态的唯一合法修改路径是 integ += err·ki·dt，饱和时冻结它，但永远不要改写它
//        pid->integ = i_out;             积分状态的合法修改路径是 integ += err·ki·dt，饱和时冻结它，但不得改写它
//    }
	//积分限幅？//


