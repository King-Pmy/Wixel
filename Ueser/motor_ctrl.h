#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include "stm32h7xx_hal.h"
#include "pid.h"
#include "c610.h"

typedef enum {
    MOTOR_MODE_IDLE = 0,    /* 电流给 0, 松轴 */
    MOTOR_MODE_SPEED,       /* 速度单环 */
    MOTOR_MODE_POSITION,    /* 位置串级: 位置环输出 -> 速度环目标 */
} Motor_Mode_t;

extern volatile Motor_Mode_t c610_mode;
extern volatile int16_t c610_target_rpm;
extern volatile int16_t c610_target_current;
extern volatile int32_t c610_target_pos;    /* 目标位置, 单位=刻度 */

void C610_SpeedPI_Init(void);
void C610_PosPI_Init(void);

void Motor_SetMode(Motor_Mode_t mode);
void Motor_SetTargetPosCounts(int32_t counts);
void Motor_MoveRelativeCounts(int32_t delta_counts);

/* 10ms 节拍内按 位置环 -> 速度环 -> 发电流 顺序调用 */
void C610_PosLoop10ms(void);
void C610_SpeedLoop10ms(void);

#endif