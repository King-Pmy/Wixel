#include "can_manager.h"

CAN_Instance_t can_manager[2] = {
	[0] = { .hfdcan = &hfdcan1 },
	[1] = { .hfdcan = &hfdcan2 },
};