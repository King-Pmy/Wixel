#ifndef __C610_H
#define __C610_H

#include "stm32h7xx_hal.h"
#include "fdcan_bsp.h"
#include "cmsis_os2.h"

/* ---- C610/M2006 协议常量 ---- */
#define C610_CMD_ID_1_4     0x200u    /* 控制帧: ID1~4 */
#define C610_CMD_ID_5_8     0x1FFu    /* 控制帧: ID5~8, byte2-3 -> ID6 */
#define C610_FB_ID_BASE     0x200u    /* 反馈帧 ID = 0x200 + 电调ID */
#define C610_CURRENT_MAX    10000     /* 手册上限 ±16384≈±10A, 这里留裕量 */
#define C610_ANGLE_MAX      8191u
#define C610_ENCODER_LINES  8192      /* 转子一圈 8192 刻度 */
#define C610_GEAR_RATIO     36        /* M2006 减速比 36:1 */

/* ---- 本机配置(换电机只改这里) ---- */
#define C610_LOCAL_ID       6
#define C610_LOCAL_BUS      (&hfdcan2)
#define C610_LOCAL_CMD_ID   C610_CMD_ID_5_8
#define C610_LOCAL_SLOT     (C610_LOCAL_ID - 5)   /* ID5->槽0 ... ID8->槽3 */

/* ---- 反馈数据: 中断写, 任务只读 ---- */
typedef struct {
    uint16_t angle;     /* 转子机械角 0~8191 */
    int16_t  rpm;       /* 转子转速 RPM */
    int16_t  torque;    /* 实际转矩电流 */
} C610_Feedback_t;

extern volatile C610_Feedback_t c610_fb;
extern volatile uint32_t c610_rx_count;    /* 累计反馈帧数, 掉线判断用 */
extern volatile int32_t  c610_pos_counts;  /* 多圈累计刻度, 仅中断累加 */

/* ---- 驱动接口 ---- */
HAL_StatusTypeDef C610_Init(void);
HAL_StatusTypeDef C610_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const int16_t current[4]);
HAL_StatusTypeDef C610_SetCurrent(int16_t current);   /* 便捷接口: 只控本机 */
uint8_t C610_FeedbackDecode(const uint8_t data[8], C610_Feedback_t *fb);

int32_t C610_GetPosCounts(void);    /* 相对零点的多圈刻度 */
float   C610_GetPosDeg(void);       /* 转子角度, 仅显示用 */
void    C610_PosZero(void);         /* 记当前位置为零点(不碰中断状态) */

#endif