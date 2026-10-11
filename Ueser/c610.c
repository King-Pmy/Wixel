#include "c610.h"
#include "fdcan_bsp.h"

volatile M2006_Feedback_t m2006_fb = {0};
volatile uint32_t m2006_rx_count = 0;
volatile int32_t  m2006_pos_counts = 0;
volatile float    m2006_posdeg =0;
/* 多圈累计内部状态 —— 中断是唯一写者, 任务严禁触碰 */
uint16_t s_last_angle  = 0;
static uint8_t  s_seeded      = 0;   /* 第一帧只播种 */
int32_t  s_zero_offset = 0;   /* PosZero 的偏移, 任务写/任务读, 中断不用 */

volatile int16_t m2006_target_rpm     = 0;
volatile int16_t m2006_target_current = 0;
volatile int32_t m2006_target_pos     = 0;
volatile int32_t m2006_target_angle   = 0;

HAL_StatusTypeDef M2006_Init(void)
{
    /* 只收本机电调反馈帧 0x200+ID; 驱动初始化只管总线, 不调上层 */
    if (FDCAN_AddFilter(M2006_LOCAL_BUS, 0, FDCAN_FILTER_MASK,
                        M2006_FB_ID_BASE + M2006_LOCAL_ID, 0x7FF,
                        FDCAN_FILTER_TO_RXFIFO0) != HAL_OK) {
        printf("M2006 filter fail\r\n");
        return HAL_ERROR;
    }
    if (FDCAN_BringUp(M2006_LOCAL_BUS) != HAL_OK) {
        printf("M2006 bringup fail\r\n");
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t M2006_FeedbackDecode(const uint8_t data[8], M2006_Feedback_t *fb)
{
    if (fb == NULL) return 0;
    fb->angle  = (uint16_t)((data[0] << 8) | data[1]);
    fb->rpm    = (int16_t)((data[2] << 8) | data[3]);   /* 有符号, 显式强转 */
    fb->torque = (int16_t)((data[4] << 8) | data[5]);
    if (fb->angle > M2006_ANGLE_MAX) return 0;
    return 1;
}

HAL_StatusTypeDef M2006_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const int16_t current[4])
{
    uint8_t cmd[8] = {0};

    if (std_id != M2006_CMD_ID_1_4 && std_id != M2006_CMD_ID_5_8) {
        return HAL_ERROR;
    }
    for (int i = 0; i < 4; i++) {
        int32_t c = current ? current[i] : 0;
        if (c >  M2006_CURRENT_MAX) c =  M2006_CURRENT_MAX;
        if (c < -M2006_CURRENT_MAX) c = -M2006_CURRENT_MAX;
        cmd[2 * i]     = (uint8_t)((uint16_t)c >> 8);
        cmd[2 * i + 1] = (uint8_t)((uint16_t)c & 0xFF);
    }
    return FDCAN_SendStd(hfdcan, std_id, cmd, 8);
}

HAL_StatusTypeDef M2006_SetCurrent(int16_t current)
{
    int16_t slot[4] = {0};
    if (current >  M2006_CURRENT_MAX) current =  M2006_CURRENT_MAX;
    if (current < -M2006_CURRENT_MAX) current = -M2006_CURRENT_MAX;
    slot[M2006_LOCAL_SLOT] = current;
    return M2006_SendCurrent(M2006_LOCAL_BUS, M2006_LOCAL_CMD_ID, slot);
}

/* 收到一帧合法反馈立即做多圈累加 —— 仅中断上下文调用 */
static void M2006_PosUpdate(uint16_t raw_angle)
{
    if (!s_seeded) {                    /* 上电第一帧: 播种, 防假跨界 */
        s_last_angle = raw_angle;
        s_seeded = 1;
        return;
    }
    int32_t d = (int32_t)raw_angle - (int32_t)s_last_angle;
    if (d >  M2006_ENCODER_LINES / 2) d -= M2006_ENCODER_LINES;  /* 反向跨零: 100->8000 实为 -292 */
    if (d < -M2006_ENCODER_LINES / 2) d += M2006_ENCODER_LINES;  /* 正向跨零: 8000->100 实为 +292 */
    s_last_angle = raw_angle;
    m2006_pos_counts += d;               /* 纯整数累加, 零精度损失, 天然多圈 */
}

/* 中断函数 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN2) return;

    FDCAN_RxHeaderTypeDef rxh;
    uint8_t d[8];
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, d) == HAL_OK) {
        if (rxh.Identifier == M2006_FB_ID_BASE + M2006_LOCAL_ID) {
            M2006_Feedback_t tmp;
            if (M2006_FeedbackDecode(d, &tmp)) {
                m2006_fb = tmp;              /* 任务中即时读取 */
                M2006_PosUpdate(tmp.angle);  
                m2006_rx_count++;
            }
        }
    }
}

int32_t M2006_GetPosCounts(void) { return m2006_pos_counts - s_zero_offset; }

float M2006_GetPosDeg(void)    /*将刻度转化为度数*/
{
	m2006_posdeg = (float)M2006_GetPosCounts() * 360.0f / (float)M2006_ENCODER_LINES;
    return m2006_posdeg;
    /* 输出轴角度再 / M2006_GEAR_RATIO */
}

void M2006_PosZero(void) { s_zero_offset = m2006_pos_counts; }

void Motor_SetTargetPosCounts(int32_t counts) { m2006_target_pos = counts; }
void Motor_MoveRelativeAngle(int32_t delta)  { m2006_target_angle = M2006_GetPosDeg() + delta; }