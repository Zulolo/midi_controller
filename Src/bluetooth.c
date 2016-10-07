
#include "bluetooth.h"

#define BT_MODULE_RESET_LOW_DURATION		7

extern osSemaphoreId BT_tUARTRsrHandle;

int32_t nSendATCmd(BT_MODULE_AT_CMD_T tBtAtCmd, uint8_t* pArguments, uint8_t unArgLength)
{
	xSemaphoreTake(BT_tUARTRsrHandle, portMAX_DELAY);
	
	xSemaphoreGive(BT_tUARTRsrHandle);
	
	return 0;
}

int32_t nInitBT(void)
{
	// BT is connected with UART1
	// UART1 has been already configured as 9600, 8, none, 1 in main
	// Set bluetooth master/slave control to be HW control
	HAL_GPIO_WritePin(BT_SLV_MST_SW_HW_GPIO_Port, BT_SLV_MST_SW_HW_Pin, GPIO_PIN_RESET);
	// Set bluetooth to be master
	HAL_GPIO_WritePin(BT_SLV_MST_GPIO_Port, BT_SLV_MST_Pin, GPIO_PIN_SET);
	
	// Reset BT module
	HAL_GPIO_WritePin(BT_NRST_GPIO_Port, BT_NRST_Pin, GPIO_PIN_RESET);
	osDelay(BT_MODULE_RESET_LOW_DURATION);
	HAL_GPIO_WritePin(BT_NRST_GPIO_Port, BT_NRST_Pin, GPIO_PIN_SET);
	
	// Read BT module info
	
	// Read BT module slave/master mode
	
	// Check BT module info
	
	// Check BT module slave/master mode
	
	// Change BT module baudrate
	
	// Change self baudrate
	
	// Read BT module baudrate
	
	// Check BT module baudrate
	
	return 0;
}

void BT_Comm(void const * argument)
{
	
}
