
#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H


#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

typedef enum
{
	ENUM_AT_CMD_TEST = 0,
	ENUM_AT_CMD_VERSION,
	ENUM_AT_CMD_LADDR,
	ENUM_AT_CMD_NAME,
	ENUM_AT_CMD_ROLE,
	ENUM_AT_CMD_BAUD,
	ENUM_AT_CMD_INQM,
	ENUM_AT_CMD_INQ,
	ENUM_AT_CMD_IMME,
	ENUM_AT_CMD_HELP
} BT_MODULE_AT_CMD_T;
	
	
int32_t nInitBT(void);

#endif
	
