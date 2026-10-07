#include "motor_task.h"
#include "c610.h"
#include "motor_ctrl.h"
#include "usart.h"      
#include <stdio.h>
#include <string.h>

extern osMessageQueueId_t uart_rxHandle; 
static uint8_t rx_byte;

#define CTRL_PERIOD_MS   10u
#define FB_TIMEOUT_MS    100u

void DJ_MOTOR(void *argument)
{
    C610_SpeedPI_Init();
    C610_PosPI_Init();

    /* 等电调自检完 + 收到第一帧反馈, 否则零点和目标都是瞎算 */
    uint32_t t0 = HAL_GetTick();
    while (c610_rx_count == 0) {
        if (HAL_GetTick() - t0 > 2000) {
            printf("C610 no feedback! check bus\r\n");
            t0 = HAL_GetTick();
        }
        osDelay(10);
    }

    C610_PosZero();
    Motor_SetMode(MOTOR_MODE_POSITION);
    Motor_MoveRelativeCounts(100);       /* 转子 +2048 刻度 = +90°(输出轴 2.5°) */

    uint32_t last_rx    = c610_rx_count;
    uint32_t last_rx_ms = HAL_GetTick();
    uint32_t tick       = osKernelGetTickCount();

    for (;;) {
        tick += CTRL_PERIOD_MS;
        osDelayUntil(tick);               /* 严格 10ms, PID 的 dt=0.01 才成立 */

        /* 掉线保护: 100ms 无新反馈 -> 失能, 帧恢复后需重新设模式 */
        if (c610_rx_count != last_rx) {
            last_rx    = c610_rx_count;
            last_rx_ms = HAL_GetTick();
        } else if (HAL_GetTick() - last_rx_ms > FB_TIMEOUT_MS) {
            Motor_SetMode(MOTOR_MODE_IDLE);
        }

        C610_PosLoop10ms(); 		/* 非 POSITION 模式内部直接返回 */
        C610_SpeedLoop10ms();
        C610_SetCurrent(c610_target_current);   /* c610_target_current */
    }
}

void VOFA(void *argument)
{
    for (;;) {
        printf("%ld,%ld,%.2f,%d,%d\n",
               (long)c610_target_pos, (long)C610_GetPosCounts(),
               C610_GetPosDeg(), (int)c610_fb.rpm, (int)c610_target_current);
        osDelay(20);
    }
}
static void Vofa_Parse(char *s)
{
    int val;
    switch (s[0]) {
    case 'r':   /* r2048  -> 相对移动 2048 刻度 */
        if (sscanf(s + 1, "%d", &val) == 1) {
            Motor_SetMode(MOTOR_MODE_POSITION);
            Motor_MoveRelativeCounts(val);
        }
        break;
    case 'p':   /* p0     -> 绝对位置目标(相对零点) */
        if (sscanf(s + 1, "%d", &val) == 1) {
            Motor_SetMode(MOTOR_MODE_POSITION);
            Motor_SetTargetPosCounts(val);
        }
        break;
    case 'v':   /* v500   -> 速度模式, 目标 500 rpm */
        if (sscanf(s + 1, "%d", &val) == 1) {
            Motor_SetMode(MOTOR_MODE_SPEED);
            c610_target_rpm = (int16_t)val;
        }
        break;
    case 'z':   /* z      -> 当前位置设为零点 */
        C610_PosZero();
        if (c610_mode == MOTOR_MODE_POSITION) Motor_SetTargetPosCounts(0);
        break;
    case 'i':   /* i      -> 松轴 */
        Motor_SetMode(MOTOR_MODE_IDLE);
        break;
    default:
        break;
    }
}
void UART(void *argument)
{
    char line[32];
    uint8_t idx = 0, b;

    HAL_UART_Receive_IT(&huart8, &rx_byte, 1);   /* 第一次启动中断接收 */

    for (;;) {
        if (osMessageQueueGet(uart_rxHandle, &b, NULL, osWaitForever) == osOK) {
            if (b == '\n' || b == '\r') {
                if (idx > 0) { line[idx] = '\0'; Vofa_Parse(line); idx = 0; }
            } else if (idx < sizeof(line) - 1) {
                line[idx++] = (char)b;
            } else {
                idx = 0;    
            }
        }
    }
}