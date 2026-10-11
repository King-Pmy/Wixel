#include "motor_task.h"
#include "c610.h"
#include "usart.h"      
#include <stdio.h>
#include <string.h>
#include "pid.h"

//extern osMessageQueueId_t uart_rxHandle; 
//static uint8_t rx_byte;

//#define FB_TIMEOUT_MS    100u
volatile int16_t cur_min  ;
PID_t s_spd;
PID_t s_pos;
//int32_t m2006_target_angle   = 0;
int32_t TGT = 5000;


void DJ_MOTOR(void *argument)
{
	
	/*位置环参数*/
    PID_Init(&s_pos, 30.0f, 0.4f, 3.0f, 0.01f);   
    PID_SetLimits(&s_pos, 10000.0f, -10000.0f,1000.0f,-1000.0f);
    /*速度环参数*/
	PID_Init(&s_spd, 2.0f, 0.2f, 0.0f, 0.01f);
    PID_SetLimits(&s_spd, 10000.0f, -10000.0f,1000.0f,-1000.0f);

    /* 等电调自检完 + 收到第一帧反馈 */
    uint32_t t0 = HAL_GetTick();
    while (m2006_rx_count == 0) {
        if (HAL_GetTick() - t0 > 2000) {
            printf("m2006 no feedback! check bus\r\n");
            t0 = HAL_GetTick();
        }
        osDelay(10);
    }

    M2006_PosZero();
//    Motor_SetMode(MOTOR_MODE_POSITION);
    Motor_MoveRelativeAngle(TGT);

//    uint32_t last_rx    = m2006_rx_count;
//    uint32_t last_rx_ms = HAL_GetTick();
//    uint32_t tick       = osKernelGetTickCount();
	
    for (;;) {
		m2006_target_rpm = (int16_t)PID_Calc_Pos(&s_pos, (float)m2006_target_angle,(float)M2006_GetPosDeg(),(float)m2006_fb.rpm);
		m2006_target_current = (int16_t)PID_Calc(&s_spd, (float)m2006_target_rpm,(float)m2006_fb.rpm);
        M2006_SetCurrent(m2006_target_current);   /* m2006_target_current */
		osDelay(1);
    }
}

void VOFA(void *argument)
{
    for (;;) {
        printf("%ld,%ld,%.2f,%d,%d\n",
               (long)m2006_target_angle, (long)M2006_GetPosCounts(),
               M2006_GetPosDeg(), (int)m2006_fb.rpm, (int)m2006_fb.torque);
        osDelay(10);
    }
}




//static void Vofa_Parse(char *s)
//{
//    int val;
//    switch (s[0]) {
//    case 'r':   /* r2048  -> 相对移动 2048 刻度 */
//        if (sscanf(s + 1, "%d", &val) == 1) {
//            Motor_SetMode(MOTOR_MODE_POSITION);
//            Motor_MoveRelativeCounts(val);
//        }
//        break;
//    case 'p':   /* p0     -> 绝对位置目标(相对零点) */
//        if (sscanf(s + 1, "%d", &val) == 1) {
//            Motor_SetMode(MOTOR_MODE_POSITION);
//            Motor_SetTargetPosCounts(val);
//        }
//        break;
//    case 'v':   /* v500   -> 速度模式, 目标 500 rpm */
//        if (sscanf(s + 1, "%d", &val) == 1) {
//            Motor_SetMode(MOTOR_MODE_SPEED);
//            m2006_target_rpm = (int16_t)val;
//        }
//        break;
//    case 'z':   /* z      -> 当前位置设为零点 */
//        m2006_PosZero();
//        if (m2006_mode == MOTOR_MODE_POSITION) Motor_SetTargetPosCounts(0);
//        break;
//    case 'i':   /* i      -> 松轴 */
//        Motor_SetMode(MOTOR_MODE_IDLE);
//        break;
//    default:
//        break;
//    }
//}
//void UART(void *argument)
//{
//    char line[32];
//    uint8_t idx = 0, b;

//    HAL_UART_Receive_IT(&huart8, &rx_byte, 1);   /* 第一次启动中断接收 */

//    for (;;) {
//        if (osMessageQueueGet(uart_rxHandle, &b, NULL, osWaitForever) == osOK) {
//            if (b == '\n' || b == '\r') {
//                if (idx > 0) { line[idx] = '\0'; Vofa_Parse(line); idx = 0; }
//            } else if (idx < sizeof(line) - 1) {
//                line[idx++] = (char)b;
//            } else {
//                idx = 0;    
//            }
//        }
//    }
//}