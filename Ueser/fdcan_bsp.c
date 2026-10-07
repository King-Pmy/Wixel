#include "fdcan_bsp.h"

HAL_StatusTypeDef FDCAN_AddFilter(FDCAN_HandleTypeDef *hfdcan,uint32_t index,uint32_t type,uint32_t id1,uint32_t id2,uint32_t fifo)
{
	if(index > 127)return HAL_ERROR;
	
	FDCAN_FilterTypeDef f = {
		.IdType = FDCAN_STANDARD_ID, //固定：只过滤标准帧
		.FilterIndex = index,        //槽号位
		.FilterType = type,          //MASK还是RANGE
		.FilterConfig =fifo,         //匹配成功进fifo0还是fifo1
		.FilterID1 = id1,            //期望Id /范围起点
		.FilterID2 = id2,            //掩码   /范围终点
	};
	if(HAL_FDCAN_ConfigFilter(hfdcan, &f) != HAL_OK) {
		return HAL_ERROR;
	}
	
	for(int i = 0;i < 2;i++){
		if(can_manager[i].hfdcan == hfdcan){ //指针比较
			can_manager[i].std_filter_cnt++;  //标准帧过滤器数+1
			break;
		}
	}
	return HAL_OK;
}
HAL_StatusTypeDef FDCAN_BringUp(FDCAN_HandleTypeDef *hfdcan)
{
	HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	if(HAL_FDCAN_Start(hfdcan) !=HAL_OK)return HAL_ERROR;
	HAL_FDCAN_ActivateNotification(hfdcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF,0);
	return HAL_OK;
}
//HAL_StatusTypeDef C610_CAN_Init(void)   /* 驱动C610的一站式入口 */
//{
//    if (FDCAN_AddFilter(&hfdcan2, 0, FDCAN_FILTER_MASK,
//                        0x201, 0x7FF, FDCAN_FILTER_TO_RXFIFO0) != HAL_OK)
//        return HAL_ERROR;
//    return FDCAN_BringUp(&hfdcan2);
//}

HAL_StatusTypeDef FDCAN_SendStd(FDCAN_HandleTypeDef *hfdcan,uint32_t std_id,const uint8_t *data,uint32_t len)
{
	if(len>8)return HAL_ERROR;
	
	static const uint32_t dlc_map[] = {
		FDCAN_DLC_BYTES_0,FDCAN_DLC_BYTES_1,FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
	};
	
	FDCAN_TxHeaderTypeDef txh = {
		.Identifier = std_id,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = FDCAN_DATA_FRAME,
		.DataLength = dlc_map[len],
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0,
	};
	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, (uint8_t *)data);
}
	