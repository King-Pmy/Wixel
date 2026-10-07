#include "motor_ctrl.h"

volatile Motor_Mode_t c610_mode = MOTOR_MODE_IDLE;
volatile int16_t c610_target_rpm     = 0;
volatile int16_t c610_target_current = 0;
volatile int32_t c610_target_pos     = 0;

static PID_t s_spd;
static PID_t s_pos;

#define POS_OUT_RPM_MAX   2000.0f   /* 位置环输出限速(转子 rpm) */
#define SPD_OUT_CUR_MAX   3000.0f   /* 速度环输出限流(≈1.2A, 台架起步) */
#define POS_DEADBAND      10         /* 死区(刻度), 进死区清积分防蠕变 */
#define POS_I_CLAMP       800.0f

void C610_SpeedPI_Init(void)
{
    PID_Init(&s_spd,2.0f, 0.5f, 0.0f, 0.01f);
    PID_SetLimits(&s_spd, SPD_OUT_CUR_MAX, -SPD_OUT_CUR_MAX);
}

void C610_PosPI_Init(void)
{
    PID_Init(&s_pos, 1.0f, 0.2f, 0.6f, 0.01f);   
    PID_SetLimits(&s_pos, POS_OUT_RPM_MAX, -POS_OUT_RPM_MAX);
}

void Motor_SetMode(Motor_Mode_t mode)
{
    c610_mode = mode;
    c610_target_rpm = 0;
    c610_target_current = 0;
    PID_Reset(&s_spd);
    PID_Reset(&s_pos);
    if (mode == MOTOR_MODE_POSITION) {
        c610_target_pos = C610_GetPosCounts();   /* 进位置环先锁当前位置, 防突跳 */
    }
}

void Motor_SetTargetPosCounts(int32_t counts) { c610_target_pos = counts; }
void Motor_MoveRelativeCounts(int32_t delta)  { c610_target_pos = C610_GetPosCounts() + delta; }

/* 外环: 位置(刻度) -> 目标 rpm */
//void C610_PosLoop10ms(void)
//{
//    if (c610_mode != MOTOR_MODE_POSITION) return;

//    int32_t err = c610_target_pos - C610_GetPosCounts();
//    if (err > -POS_DEADBAND && err < POS_DEADBAND) {
//        c610_target_rpm = 0;
//        PID_Reset(&s_pos);
//        return;
//    }
//    c610_target_rpm = (int16_t)PID_Calc(&s_pos, (float)c610_target_pos,
//                                        (float)C610_GetPosCounts());
//}
void C610_PosLoop10ms(void)
{
    int32_t err = c610_target_pos - C610_GetPosCounts();  /* 目标位置 - 当前总角度 */
	if(err > -POS_DEADBAND && err < POS_DEADBAND){
		c610_target_rpm = 0;
		s_pos.integ = 0;
		return;
	}
	s_pos.integ += (float)err * 0.01f; 
	if (s_pos.integ >  POS_I_CLAMP) s_pos.integ =  POS_I_CLAMP;
	if (s_pos.integ < -POS_I_CLAMP) s_pos.integ = -POS_I_CLAMP;
	
    int32_t spd = (int32_t)(s_pos.kp * (float)err + s_pos.ki * s_pos.integ - s_pos.kd * (float)c610_fb.rpm); /* Kp_pos=0.5, 起步参数 */

	int16_t v_max = 2000;
	if(err < 1500&& err> 1500)v_max = 400;
    if (spd >  v_max) spd =  v_max;
    if (spd < -v_max) spd = -v_max;
	if(spd > 0&& spd < 100)spd = 100;
	if(spd < 0&& spd > -100)spd = -100;

    c610_target_rpm = (int16_t)spd;   /* 交给内环 */
}

/* 内环: 目标 rpm -> 电流 */
void C610_SpeedLoop10ms(void)
{
    if (c610_mode == MOTOR_MODE_IDLE) {
        c610_target_current = 0;
        PID_Reset(&s_spd);
        return;
    }
    c610_target_current = (int16_t)PID_Calc(&s_spd, (float)c610_target_rpm,
                                            (float)c610_fb.rpm);
}