#ifndef __M2006_H
#define __M2006_H

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"

/* ---- M2006/M2006 协议常量 ---- */
#define M2006_CMD_ID_1_4     0x200u    /* 控制帧: ID1~4 */
#define M2006_CMD_ID_5_8     0x1FFu    /* 控制帧: ID5~8, byte2-3 -> ID6 */
#define M2006_FB_ID_BASE     0x200u    /* 反馈帧 ID = 0x200 + 电调ID */
#define M2006_CURRENT_MAX    10000     /* 手册上限 ±16384≈±10A, 这里留裕量 */
#define M2006_ANGLE_MAX      8191u
#define M2006_ENCODER_LINES  8192      /* 转子一圈 8192 刻度 */
#define M2006_GEAR_RATIO     36        /* M2006 减速比 36:1 */

/* ---- 本机配置(换电机只改这里) ---- */
#define M2006_LOCAL_ID       6
#define M2006_LOCAL_BUS      (&hfdcan2)
#define M2006_LOCAL_CMD_ID   M2006_CMD_ID_5_8
#define M2006_LOCAL_SLOT     (M2006_LOCAL_ID - 5)   /* ID5->槽0 ... ID8->槽3 */

/* ---- 反馈数据: 中断写, 任务只读 ---- */
typedef struct {
    uint16_t angle;     /* 转子机械角 0~8191 */
    int16_t  rpm;       /* 转子转速 RPM */
    int16_t  torque;    /* 实际转矩电流 */
} M2006_Feedback_t;

extern volatile M2006_Feedback_t m2006_fb;
extern volatile uint32_t m2006_rx_count;    /* 累计反馈帧数, 掉线判断用 */
extern volatile int32_t  m2006_pos_counts;  /* 多圈累计刻度 */

extern volatile int16_t m2006_target_rpm;
extern volatile int16_t m2006_target_current;
extern volatile int32_t m2006_target_pos;
extern volatile int32_t m2006_target_angle;

/* ---- 驱动接口 ---- */
HAL_StatusTypeDef M2006_Init(void);
HAL_StatusTypeDef M2006_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const int16_t current[4]);
HAL_StatusTypeDef M2006_SetCurrent(int16_t current);   /* 便捷接口: 只控本机 */
uint8_t M2006_FeedbackDecode(const uint8_t data[8], M2006_Feedback_t *fb);

int32_t M2006_GetPosCounts(void);    /* 相对零点的多圈刻度 */
float   M2006_GetPosDeg(void);       /* 转子角度 */
void    M2006_PosZero(void);         /* 记当前位置为零点 */

void Motor_SetTargetPosCounts(int32_t counts);
void Motor_MoveRelativeAngle(int32_t delta_counts);
#endif