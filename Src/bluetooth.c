
#include <string.h>
#include "bluetooth.h"

#define BT_MODULE_RESET_LOW_DURATION		7
#define MAX_BT_UART_RX_LENGTH						128
#define MAX_BT_UART_TX_LENGTH						64
#define BT_UART_HANDLE									huart1
#define AT_CMD_END_CHARACTERS						"\r\n"

extern osSemaphoreId BT_tUART_TxIsrHandle;
extern osSemaphoreId BT_tUART_RxIsrHandle;
extern UART_HandleTypeDef huart1;

/*
typedef enum
{
	AT_CMD_TEST = 0,
	AT_CMD_VERSION,
	AT_CMD_LADDR,
	AT_CMD_NAME
} BT_MODULE_AT_CMD_T;

*/
// MAX_BT_UART_RX_LENGTH means unknow
const uint8_t UN_AT_CMD_RSP_LEN[] = {MAX_BT_UART_RX_LENGTH, MAX_BT_UART_RX_LENGTH, MAX_BT_UART_RX_LENGTH, MAX_BT_UART_RX_LENGTH};
const char* P_AT_CMD[] = {"AT", "AT+VERSION", "AT+LADDR", "AT+NAME"};
//static FlagStatus tBtUartRxTimeout;

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	tHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(BT_tUART_TxIsrHandle, &tHigherPriorityTaskWoken);
	if(tHigherPriorityTaskWoken == pdTRUE)
	{
		portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
	}

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	tHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(BT_tUART_RxIsrHandle, &tHigherPriorityTaskWoken);
	if(tHigherPriorityTaskWoken == pdTRUE)
	{
		portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
	}
  
}

// Only write to BT module, no check return
//int32_t nWriteBtModule(BT_MODULE_AT_CMD_T tBtAtCmd, uint8_t* pTxBuffer, uint8_t unTxLength, uint16_t unTimeoutMs)
//{
//	xSemaphoreTake(BT_tUART_TxRsrHandle, portMAX_DELAY);
//	if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)pTxBuffer, unTxLength)!= HAL_OK)
//  {
//    return (-1);
//  }
//	return 0;
//}

//// Write to BT module and then read response
//int32_t nReadBtModule(BT_MODULE_AT_CMD_T tBtAtCmd, uint8_t* pRxBuffer, uint8_t unRxLength, uint16_t unTimeoutMs)
//{
//	xSemaphoreTake(BT_tUART_RxRsrHandle, portMAX_DELAY);
//	if(HAL_UART_Receive_DMA(&huart1, (uint8_t*)pRxBuffer, unRxLength)!= HAL_OK)
//  {
//    return (-1);
//  }
//	return 0;
//}

uint8_t getAtCmdRspLen(BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, uint8_t unArgLength)
{
	if ((uint8_t)tBtAtCmd >= sizeof(UN_AT_CMD_RSP_LEN)){
		return MAX_BT_UART_RX_LENGTH;
	}else{
		return UN_AT_CMD_RSP_LEN[tBtAtCmd];	
	}
}

uint16_t getAtCmdTimeout(BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, uint8_t unArgLength)
{
	return 1000;	// 50ms
}

uint8_t getAtCmdString(char* pTxBuffer, BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, 
	uint8_t unArgLength, uint8_t unMaxBtUartTxLen)
{
	uint8_t unAtCmdLen;
	if ((uint8_t)tBtAtCmd >= sizeof(P_AT_CMD)){
		pTxBuffer[0] = '\0';
		return 0;
	}
	unAtCmdLen = unArgLength + strlen(P_AT_CMD[tBtAtCmd]) + strlen(AT_CMD_END_CHARACTERS);
	if (unAtCmdLen > (unMaxBtUartTxLen - 1)){
		pTxBuffer[0] = '\0';
		return 0;
	}
	
	if (NULL == pArguments){
		strncat(strcpy(pTxBuffer, P_AT_CMD[tBtAtCmd]), AT_CMD_END_CHARACTERS, strlen(AT_CMD_END_CHARACTERS));
	}else{
		strncat(strncat(strcpy(pTxBuffer, P_AT_CMD[tBtAtCmd]), pArguments, unArgLength), AT_CMD_END_CHARACTERS, strlen(AT_CMD_END_CHARACTERS));
	}
	pTxBuffer[unMaxBtUartTxLen - 1] = '\0';
	
	return unAtCmdLen;
}
// no matter write or read the AT command is, received BT module's response
char* pSendATCmd(BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, uint8_t unArgLength)
{
	static char unRxBuffer[MAX_BT_UART_RX_LENGTH];
	static char unTxBuffer[MAX_BT_UART_TX_LENGTH];
	uint16_t unTimeout;
	uint8_t unAtCmdRspLen;
	uint8_t unAtCmdLen;
		
	// 0. Analysis and prepare all parameter used to operate UART API
	if ((uint8_t)tBtAtCmd >= sizeof(P_AT_CMD)){
		return NULL;
	}
	unAtCmdLen = getAtCmdString(unTxBuffer, tBtAtCmd, pArguments, unArgLength, MAX_BT_UART_TX_LENGTH);
	if (('\0' == unTxBuffer[0]) || (0 == unAtCmdLen)){
		return NULL;
	}
	memset(unRxBuffer, 0, MAX_BT_UART_RX_LENGTH);
	unAtCmdRspLen = getAtCmdRspLen(tBtAtCmd, pArguments, unArgLength);
	
	// 1. Make sure no one is using RX and TX
	xSemaphoreTake(BT_tUART_TxIsrHandle, portMAX_DELAY);
	xSemaphoreTake(BT_tUART_RxIsrHandle, portMAX_DELAY);
	
  /* 2. DMA is programmed for reception before starting the transmission, in order to
     be sure DMA Rx is ready when board 2 will start transmitting */
  /* Program the Reception process #####################################*/  
  if(HAL_UART_Receive_DMA(&BT_UART_HANDLE, (uint8_t*)unRxBuffer, unAtCmdRspLen) != HAL_OK)
  {
    return NULL;
  }

  /* 3. Start the transmission process #####################################*/  
  /* While the UART in reception process, user can transmit data through 
     "aTxBuffer" buffer */
  if(HAL_UART_Transmit_DMA(&BT_UART_HANDLE, (uint8_t*)unTxBuffer, unAtCmdLen)!= HAL_OK)
  {
    return NULL;
  }
	
	// 4. Get timeout according to different AT command
	unTimeout = getAtCmdTimeout(tBtAtCmd, pArguments, unArgLength);
	
	// 5. Wait until UART receive finished or timeout
	if (pdTRUE == xSemaphoreTake(BT_tUART_RxIsrHandle, unTimeout)){
//		HAL_UART_DMAStop(&BT_UART_HANDLE);
		xSemaphoreGive(BT_tUART_RxIsrHandle);
//		xSemaphoreGive(BT_tUART_TxIsrHandle);
		
		return unRxBuffer;
	}else{
		HAL_UART_DMAStop(&BT_UART_HANDLE);
		xSemaphoreGive(BT_tUART_RxIsrHandle);
//		xSemaphoreGive(BT_tUART_TxIsrHandle);

		return NULL;
	}
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
	
	// BT module AT test
	pSendATCmd(AT_CMD_TEST, NULL, 0);
	// Read BT module version
	pSendATCmd(AT_CMD_VERSION, NULL, 0);
	// Read BT module BT address
	pSendATCmd(AT_CMD_LADDR, NULL, 0);
	// Read BT module name
	pSendATCmd(AT_CMD_NAME, NULL, 0);	
	
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
	nInitBT();
}
