#include "c610.h"

volatile C610_Feedback_t c610_fb = {0};
volatile uint32_t c610_rx_count = 0;
volatile int32_t  c610_pos_counts = 0;
volatile float    c610_posdeg =0;
/* 多圈累计内部状态 —— 中断是唯一写者, 任务严禁触碰 */
static uint16_t s_last_angle  = 0;
static uint8_t  s_seeded      = 0;   /* 第一帧只播种 */
static int32_t  s_zero_offset = 0;   /* PosZero 的偏移, 任务写/任务读, 中断不用 */

HAL_StatusTypeDef C610_Init(void)
{
    /* 只收本机电调反馈帧 0x200+ID; 驱动初始化只管总线, 不调上层 */
    if (FDCAN_AddFilter(C610_LOCAL_BUS, 0, FDCAN_FILTER_MASK,
                        C610_FB_ID_BASE + C610_LOCAL_ID, 0x7FF,
                        FDCAN_FILTER_TO_RXFIFO0) != HAL_OK) {
        printf("C610 filter fail\r\n");
        return HAL_ERROR;
    }
    if (FDCAN_BringUp(C610_LOCAL_BUS) != HAL_OK) {
        printf("C610 bringup fail\r\n");
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t C610_FeedbackDecode(const uint8_t data[8], C610_Feedback_t *fb)
{
    if (fb == NULL) return 0;
    fb->angle  = (uint16_t)((data[0] << 8) | data[1]);
    fb->rpm    = (int16_t)((data[2] << 8) | data[3]);   /* 有符号, 显式强转 */
    fb->torque = (int16_t)((data[4] << 8) | data[5]);
    if (fb->angle > C610_ANGLE_MAX) return 0;
    return 1;
}

HAL_StatusTypeDef C610_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const int16_t current[4])
{
    uint8_t cmd[8] = {0};

    if (std_id != C610_CMD_ID_1_4 && std_id != C610_CMD_ID_5_8) {
        return HAL_ERROR;
    }
    for (int i = 0; i < 4; i++) {
        int32_t c = current ? current[i] : 0;
        if (c >  C610_CURRENT_MAX) c =  C610_CURRENT_MAX;
        if (c < -C610_CURRENT_MAX) c = -C610_CURRENT_MAX;
        cmd[2 * i]     = (uint8_t)((uint16_t)c >> 8);
        cmd[2 * i + 1] = (uint8_t)((uint16_t)c & 0xFF);
    }
    return FDCAN_SendStd(hfdcan, std_id, cmd, 8);
}

HAL_StatusTypeDef C610_SetCurrent(int16_t current)
{
    int16_t slot[4] = {0};
    if (current >  C610_CURRENT_MAX) current =  C610_CURRENT_MAX;
    if (current < -C610_CURRENT_MAX) current = -C610_CURRENT_MAX;
    slot[C610_LOCAL_SLOT] = current;
    return C610_SendCurrent(C610_LOCAL_BUS, C610_LOCAL_CMD_ID, slot);
}

/* 收到一帧合法反馈立即做多圈累加 —— 仅中断上下文调用 */
static void C610_PosUpdate(uint16_t raw_angle)
{
    if (!s_seeded) {                    /* 上电第一帧: 播种, 防假跨界 */
        s_last_angle = raw_angle;
        s_seeded = 1;
        return;
    }
    int32_t d = (int32_t)raw_angle - (int32_t)s_last_angle;
    if (d >  C610_ENCODER_LINES / 2) d -= C610_ENCODER_LINES;  /* 反向跨零: 100->8000 实为 -292 */
    if (d < -C610_ENCODER_LINES / 2) d += C610_ENCODER_LINES;  /* 正向跨零: 8000->100 实为 +292 */
    s_last_angle = raw_angle;
    c610_pos_counts += d;               /* 纯整数累加, 零精度损失, 天然多圈 */
}

/* 中断函数 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN2) return;

    FDCAN_RxHeaderTypeDef rxh;
    uint8_t d[8];
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, d) == HAL_OK) {
        if (rxh.Identifier == C610_FB_ID_BASE + C610_LOCAL_ID) {
            C610_Feedback_t tmp;
            if (C610_FeedbackDecode(d, &tmp)) {
                c610_fb = tmp;              /* 任务侧只读 */
                C610_PosUpdate(tmp.angle);  
                c610_rx_count++;
            }
        }
    }
}

int32_t C610_GetPosCounts(void) { return c610_pos_counts - s_zero_offset; }

float C610_GetPosDeg(void)
{
	c610_posdeg = (float)C610_GetPosCounts() * 360.0f / (float)C610_ENCODER_LINES;
    return c610_posdeg;
    /* 想要输出轴角度再 / C610_GEAR_RATIO */
}

void C610_PosZero(void) { s_zero_offset = c610_pos_counts; }

