#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include <stdio.h>
typedef struct{
	FDCAN_HandleTypeDef *hfdcan;
	uint8_t std_filter_cnt;
	uint8_t stated;
	uint32_t rx_total;
	uint32_t tx_fail_cnt;
	uint32_t busoff_cnt;
}CAN_Instance_t;

extern CAN_Instance_t can_manager[2];



#endif
