//#include "motor_ctrl.h"

//volatile Motor_Mode_t m2006_mode = MOTOR_MODE_IDLE;
////volatile int16_t m2006_target_rpm     = 0;
////volatile int16_t m2006_target_current = 0;
////volatile int32_t m2006_target_pos     = 0;
////volatile int32_t m2006_target_angle   = 0;

////void Motor_SetTargetPosCounts(int32_t counts) { m2006_target_pos = counts; }
////void Motor_MoveRelativeAngle(int32_t delta)  { m2006_target_angle = M2006_GetPosDeg() + delta; }  








////PID_t s_spd;
////PID_t s_pos;

//#define POS_OUT_RPM_MAX   3000.0f   /* 位置环输出限速(转子 rpm) */
//#define SPD_OUT_CUR_MAX   3000.0f   /* 速度环输出限流(≈1.2A, 台架起步) */
//#define INTEG_OUT_CUR_MAX     800.0f    /*电流积分限幅*/
//#define INTEG_OUT_RPM_MAX     800.0f    /*转速积分限幅*/
//#define POS_DEADBAND      5         /* 死区(刻度), 进死区清积分防蠕变 */
//#define POS_I_CLAMP       800.0f
//void M2006_AngleLoop(void)
//{
//    float err = m2006_target_angle - M2006_GetPosDeg();  /* 目标位置 - 当前总角度 */
//	if(err > -POS_DEADBAND && err < POS_DEADBAND){
//		m2006_target_rpm = 0;
//		s_pos.integ = 0;
//		return;
//	}
//	s_pos.integ += (float)err * 0.001f; 
//	if (s_pos.integ >  POS_I_CLAMP) s_pos.integ =  POS_I_CLAMP;
//	if (s_pos.integ < -POS_I_CLAMP) s_pos.integ = -POS_I_CLAMP;
//	
//    int32_t spd = ((int32_t)(s_pos.kp * (float)err + s_pos.ki * s_pos.integ - s_pos.kd * (float)m2006_fb.rpm) * 6.0)*M2006_ANGLE_MAX; /* Kp_pos=0.5, 起步参数 */
// 
//	int32_t aerr = (err > 0) ? err : -err;

//    int32_t v_max  = 1500;          /* 巡航限速 */
////    int32_t v_soft = aerr / 3;      /* 线性减速坡: 离目标越近, 允许速度越低 */
////    if (v_soft < v_max) v_max = v_soft;
////    if (v_max < 80) v_max = 80;
//    if (spd >  v_max) spd =  v_max;
//    if (spd < -v_max) spd = -v_max;

//    m2006_target_rpm = (int16_t)spd;   /* 交给内环(int16_t)spd */
//}



////void M2006_SpeedLoop(void)
////{
////    if (m2006_mode == MOTOR_MODE_IDLE) {
////        m2006_target_current = 0;
////        PID_Reset(&s_spd);
////        return;
////    }
////    m2006_target_current = (int16_t)PID_Calc(&s_spd, (float)m2006_target_rpm,
////                                            (float)m2006_fb.rpm);
////}

////void M2006_SpeedPI_Init(void)
////{
////    PID_Init(&s_spd, 1.2f, 0.2f, 0.0f, 0.01f);
////    PID_SetLimits(&s_spd, 3000.0f, -3000.0f,800.0f,-800.0f);
////}

////void M2006_PosPI_Init(void)
////{
////    PID_Init(&s_pos, 1.0f, 0.0f, 0.0f, 0.01f);   
////    PID_SetLimits(&s_pos, POS_OUT_RPM_MAX, -POS_OUT_RPM_MAX,INTEG_OUT_RPM_MAX,-INTEG_OUT_RPM_MAX);
////}


////void Motor_SetMode(Motor_Mode_t mode)
////{
////    m2006_mode = mode;
////    m2006_target_rpm = 0;
////    m2006_target_current = 0;
////    PID_Reset(&s_spd);
////    PID_Reset(&s_pos);
////    if (mode == MOTOR_MODE_POSITION) {
////        m2006_target_pos = M2006_GetPosCounts();   /* 进位置环先锁当前位置, 防突跳 */
////    }
////}



///* 外环: 位置(刻度) -> 目标 rpm */
////void M2006_PosLoop10ms(void)
////{
////    if (m2006_mode != MOTOR_MODE_POSITION) return;
////    int32_t err = m2006_target_pos - M2006_GetPosCounts();
////    if (err > -POS_DEADBAND && err < POS_DEADBAND) {
////        m2006_target_rpm = 0;
////        PID_Reset(&s_pos);
////        return;
////    }
////    m2006_target_rpm = (int16_t)PID_Calc(&s_pos, (float)m2006_target_angle,(float)M2006_GetPosDeg());
////}
////void M2006_PosLoop10ms(void)
////{
////    int32_t err = m2006_target_pos - M2006_GetPosCounts();  /* 目标位置 - 当前总角度 */
////	if(err > -POS_DEADBAND && err < POS_DEADBAND){
////		m2006_target_rpm = 0;
////		s_pos.integ = 0;
////		return;
////	}
////	s_pos.integ += (float)err * 0.01f; 
////	if (s_pos.integ >  POS_I_CLAMP) s_pos.integ =  POS_I_CLAMP;
////	if (s_pos.integ < -POS_I_CLAMP) s_pos.integ = -POS_I_CLAMP;
////	
////    int32_t spd = (int32_t)(s_pos.kp * (float)err + s_pos.ki * s_pos.integ - s_pos.kd * (float)m2006_fb.rpm); /* Kp_pos=0.5, 起步参数 */
//// 
////	int16_t v_max = 2000;
////	if(err < 1500&& err> 1500)v_max = 400;
////    if (spd >  v_max) spd =  v_max;
////    if (spd < -v_max) spd = -v_max;
////	if(spd > 0&& spd < 100)spd = 100;
////	if(spd < 0&& spd > -100)spd = -100;

////    m2006_target_rpm = (int16_t)spd;   /* 交给内环 */
////}

///* 内环: 目标 rpm -> 电流 */
