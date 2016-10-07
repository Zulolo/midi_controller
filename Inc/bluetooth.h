
#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H


#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

typedef enum
{
	AT_CMD_TEST = 0,
	AT_CMD_VERSION,
	AT_CMD_LADDR,
	AT_CMD_NAME
} BT_MODULE_AT_CMD_T;
	
	
int32_t nInitBT(void);

#endif
	
