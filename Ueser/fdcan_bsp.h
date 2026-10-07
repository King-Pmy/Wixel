#ifndef __FDCAN_BSP_H
#define __FDCAN_BSP_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32h7xx_hal.h" /*HAL库源：FDCAN_HandleTypeDef,HAL_SatausTypedef等*/
#include "fdcan.h"
#include "can_manager.h"
#include <stdio.h>
#include <stdbool.h>


HAL_StatusTypeDef FDCAN_AddFilter(FDCAN_HandleTypeDef *hfdcan,uint32_t index,uint32_t type,uint32_t id1,uint32_t id2,uint32_t fifo);
HAL_StatusTypeDef FDCAN_BringUp(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef FDCAN_SendStd(FDCAN_HandleTypeDef *hfdcan,uint32_t std_id,const uint8_t *data,uint32_t len);
#ifdef __cplusplus
}
#endif
#endif /* __FDCAN_BSP_H */
