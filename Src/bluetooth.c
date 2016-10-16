
#include <string.h>
#include "bluetooth.h"

#define BT_MODULE_RESET_LOW_DURATION		30
#define BT_MODULE_RESET_AFTER_DURATION	20
#define MAX_BT_UART_RX_LENGTH						512
#define MAX_BT_UART_TX_LENGTH						64
#define BT_UART_HANDLE									huart1
#define AT_CMD_END_CHARACTERS						"\r\n"
#define AT_CMD_END_LEN									(sizeof(AT_CMD_END_CHARACTERS) - 1)
#define AT_CMD_RSP_OK_CHARACTERS				"OK\r\n"
#define AT_CMD_RSP_OK_LEN								(sizeof(AT_CMD_RSP_OK_CHARACTERS) - 1)
#define TARGET_BT_MODULE_BAUD						"8"		//115200
#define TARGET_MIDI_CTRL_BAUD						115200
#define MAX_BT_MDL_BAUD_RETRY_NUM				5

#define TEST_AT_VERSION_RSP							"+VERSION"

#define AT_CMD_AT												"AT"
#define AT_CMD_VERSION									"AT+VERSION"
#define AT_CMD_LADDR										"AT+LADDR"
#define AT_CMD_NAME											"AT+NAME"
#define AT_CMD_ROLE											"AT+ROLE"
#define AT_CMD_BAUD											"AT+BAUD"
#define AT_CMD_INQM											"AT+INQM"
#define AT_CMD_INQ											"AT+INQ"
#define AT_CMD_IMME											"AT+IMME"
#define AT_CMD_HELP											"AT+HELP"

#define AT_CMD_RSP_AT										"+INQS\r\n"
#define AT_CMD_RSP_VERSION							"+VERSION=Firmware V3.0.6,Bluetooth V4.0 LE\r\n"
#define AT_CMD_RSP_LADDR								"+LADDR=00:00:00:00:00:00\r\n"
#define AT_CMD_RSP_NAME									"+NAME=BT05\r\n"
#define AT_CMD_RSP_ROLE									"+ROLE=1\r\n"
#define AT_CMD_RSP_BAUD									"AT+BAUD\r\n"
#define AT_CMD_RSP_INQM									""
#define AT_CMD_RSP_INQ									""
#define AT_CMD_RSP_IMME									"+IMME=0\r\n"
#define AT_CMD_RSP_HELP									""

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

const char* P_AT_CMD[] = {AT_CMD_AT, AT_CMD_VERSION, AT_CMD_LADDR, AT_CMD_NAME, 
													AT_CMD_ROLE, AT_CMD_BAUD, AT_CMD_INQM, AT_CMD_INQ, 
													AT_CMD_IMME, AT_CMD_HELP};
const uint16_t UN_AT_CMD_LEN[] = {sizeof(AT_CMD_AT) - 1, sizeof(AT_CMD_VERSION) - 1, sizeof(AT_CMD_LADDR) - 1, sizeof(AT_CMD_NAME) - 1,
																	sizeof(AT_CMD_ROLE) - 1, sizeof(AT_CMD_BAUD) - 1, sizeof(AT_CMD_INQM) - 1, sizeof(AT_CMD_INQ) - 1,
																	sizeof(AT_CMD_IMME) - 1, sizeof(AT_CMD_HELP) - 1};

const char* P_AT_CMD_RSP[] = {AT_CMD_RSP_AT, AT_CMD_RSP_VERSION, AT_CMD_RSP_LADDR, AT_CMD_RSP_NAME, 
															AT_CMD_RSP_ROLE, AT_CMD_RSP_BAUD, AT_CMD_RSP_INQM, AT_CMD_RSP_INQ,
															AT_CMD_RSP_IMME, AT_CMD_RSP_HELP};
const uint16_t UN_AT_CMD_RSP_LEN[] = {sizeof(AT_CMD_RSP_AT) - 1, sizeof(AT_CMD_RSP_VERSION) - 1, sizeof(AT_CMD_RSP_LADDR) - 1, sizeof(AT_CMD_RSP_NAME) - 1,
																			sizeof(AT_CMD_RSP_ROLE) - 1, sizeof(AT_CMD_RSP_BAUD) - 1, sizeof(AT_CMD_RSP_INQM) - 1, sizeof(AT_CMD_RSP_INQ) - 1,
																			sizeof(AT_CMD_RSP_IMME) - 1, sizeof(AT_CMD_RSP_HELP) - 1};

const uint32_t UN_BAUD_LIST[] = {9600, 19200, 38400, 57600, 115200, 230400};
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
//int32_t nWriteBtModule(BT_MODULE_AT_CMD_T tBtAtCmd, uint16_t* pTxBuffer, uint16_t unTxLength, uint16_t unTimeoutMs)
//{
//	xSemaphoreTake(BT_tUART_TxRsrHandle, portMAX_DELAY);
//	if(HAL_UART_Transmit_DMA(&huart1, (uint16_t*)pTxBuffer, unTxLength)!= HAL_OK)
//  {
//    return (-1);
//  }
//	return 0;
//}

//// Write to BT module and then read response
//int32_t nReadBtModule(BT_MODULE_AT_CMD_T tBtAtCmd, uint16_t* pRxBuffer, uint16_t unRxLength, uint16_t unTimeoutMs)
//{
//	xSemaphoreTake(BT_tUART_RxRsrHandle, portMAX_DELAY);
//	if(HAL_UART_Receive_DMA(&huart1, (uint16_t*)pRxBuffer, unRxLength)!= HAL_OK)
//  {
//    return (-1);
//  }
//	return 0;
//}

uint16_t getAtCmdRspLen(BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, uint16_t unArgLength)
{
	if ((uint16_t)tBtAtCmd >= sizeof(UN_AT_CMD_RSP_LEN)){
		return MAX_BT_UART_RX_LENGTH;
	}else{
		switch((uint32_t)tBtAtCmd){
			case ENUM_AT_CMD_TEST:
				return MAX_BT_UART_RX_LENGTH;
			case ENUM_AT_CMD_INQ:
				return MAX_BT_UART_RX_LENGTH;
			case ENUM_AT_CMD_HELP:
				return MAX_BT_UART_RX_LENGTH;			
			default:
				if (NULL == pArguments){
					return UN_AT_CMD_RSP_LEN[tBtAtCmd];	
				}else{
					return UN_AT_CMD_RSP_LEN[tBtAtCmd] + AT_CMD_RSP_OK_LEN;	
				}		
		}	
	}
}

uint16_t getAtCmdTimeout(BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, uint16_t unArgLength)
{
	switch((uint32_t)tBtAtCmd){
		case ENUM_AT_CMD_INQ:
			return 20000;
		case ENUM_AT_CMD_TEST:
			return 20000;
		default:
			return ((NULL == pArguments)?(100):(300));	
	}	
}

uint16_t getAtCmdString(char* pTxBuffer, BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, 
	uint16_t unArgLength, uint16_t unMaxBtUartTxLen)
{
	uint16_t unAtCmdLen;
	if ((uint16_t)tBtAtCmd >= sizeof(P_AT_CMD)){
		pTxBuffer[0] = '\0';
		return 0;
	}
	unAtCmdLen = unArgLength + UN_AT_CMD_LEN[tBtAtCmd] + AT_CMD_END_LEN;
	if (unAtCmdLen > (unMaxBtUartTxLen - 1)){
		pTxBuffer[0] = '\0';
		return 0;
	}
	
	if (NULL == pArguments){
		strncat(strcpy(pTxBuffer, P_AT_CMD[tBtAtCmd]), AT_CMD_END_CHARACTERS, strlen(AT_CMD_END_CHARACTERS));
	}else{
		strncat(strncat(strcpy(pTxBuffer, P_AT_CMD[tBtAtCmd]), pArguments, unArgLength), AT_CMD_END_CHARACTERS, strlen(AT_CMD_END_CHARACTERS));
	}
	pTxBuffer[unMaxBtUartTxLen] = '\0';
	
	return unAtCmdLen;
}
// no matter write or read the AT command is, received BT module's response
char* pSendATCmd(BT_MODULE_AT_CMD_T tBtAtCmd, char* pArguments, uint16_t unArgLength)
{
	static char unRxBuffer[MAX_BT_UART_RX_LENGTH];
	static char unTxBuffer[MAX_BT_UART_TX_LENGTH + 1];
	uint16_t unTimeout;
	uint16_t unAtCmdRspLen;
	uint16_t unAtCmdLen;
		
	// 0. Analysis and prepare all parameter used to operate UART API
	if ((uint16_t)tBtAtCmd >= sizeof(P_AT_CMD)){
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

		return unRxBuffer;
	}
}

void resetBtModule(void)
{
	// Reset BT module
	HAL_GPIO_WritePin(BT_NRST_GPIO_Port, BT_NRST_Pin, GPIO_PIN_RESET);
	osDelay(BT_MODULE_RESET_LOW_DURATION);
	HAL_GPIO_WritePin(BT_NRST_GPIO_Port, BT_NRST_Pin, GPIO_PIN_SET);
	osDelay(BT_MODULE_RESET_AFTER_DURATION);
}

int32_t nSetBtModuleBaudRate(void)
{
	uint16_t unBaudRateIndex;
	uint16_t unRetryNum;
	for (unBaudRateIndex = 0; unBaudRateIndex < sizeof(UN_BAUD_LIST); unBaudRateIndex++){
		resetBtModule();
		// Try to see which baud rate can work with BT module because even after power of, the previous configuration will be kept
		BT_UART_HANDLE.Init.BaudRate = UN_BAUD_LIST[unBaudRateIndex];
		if (HAL_UART_Init(&BT_UART_HANDLE) != HAL_OK)
		{
			return (-1);
		}
		unRetryNum = 0;
		while ((strstr(pSendATCmd(ENUM_AT_CMD_VERSION, NULL, 0), TEST_AT_VERSION_RSP) == NULL) && (unRetryNum < MAX_BT_MDL_BAUD_RETRY_NUM)){
			osDelay(5);
			unRetryNum++;
		}
		if (unRetryNum != MAX_BT_MDL_BAUD_RETRY_NUM){
			// This baud rate has response
			break;
		}
	}
	if (sizeof(UN_BAUD_LIST) == unBaudRateIndex){
		// Didn't find any baud rate which can communicate with BT module
		return (-1);
	}else if (UN_BAUD_LIST[unBaudRateIndex] != TARGET_MIDI_CTRL_BAUD){
		// Use this baud rate to change BT module's baud rate
		pSendATCmd(ENUM_AT_CMD_BAUD, TARGET_BT_MODULE_BAUD, strlen(TARGET_BT_MODULE_BAUD));
		osDelay(50);
		
		BT_UART_HANDLE.Init.BaudRate = TARGET_MIDI_CTRL_BAUD;
		if (HAL_UART_Init(&BT_UART_HANDLE) != HAL_OK)
		{
			return (-1);
		}
		unRetryNum = 0;
		while ((strstr(pSendATCmd(ENUM_AT_CMD_VERSION, NULL, 0), TEST_AT_VERSION_RSP) == NULL) && unRetryNum < MAX_BT_MDL_BAUD_RETRY_NUM){
			osDelay(5);
			unRetryNum++;
		}
		if (unRetryNum != MAX_BT_MDL_BAUD_RETRY_NUM){
			// This baud rate has response
			return 0;
		}else{
			// WTF, kidding me??
			return (-1);
		}
	}else{
		// Good nothing need to do, already target baud rate
		return 1;
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
	
	if (nSetBtModuleBaudRate() < 0){
		return (-1);
	}
	
	// Check BT module version
	if (strstr(pSendATCmd(ENUM_AT_CMD_VERSION, NULL, 0), P_AT_CMD_RSP[ENUM_AT_CMD_VERSION]) == NULL){
		return (-1);
	}
	// Check BT module name
	if (strstr(pSendATCmd(ENUM_AT_CMD_NAME, NULL, 0), P_AT_CMD_RSP[ENUM_AT_CMD_NAME]) == NULL){
		return (-1);
	}	
	// Check BT module slave/master mode
	if (strstr(pSendATCmd(ENUM_AT_CMD_ROLE, NULL, 0), P_AT_CMD_RSP[ENUM_AT_CMD_ROLE]) == NULL){
		return (-1);
	}

	return 0;
}

void BT_Comm(void const * argument)
{
	if (nInitBT() < 0){
		vTaskDelete(NULL);
	}
	if ((strstr(pSendATCmd(ENUM_AT_CMD_IMME, NULL, 0), P_AT_CMD_RSP[ENUM_AT_CMD_IMME]) == NULL)){
		// Set power on auto start
		pSendATCmd(ENUM_AT_CMD_IMME, "0", 1);
		resetBtModule();		
	}
	pSendATCmd(ENUM_AT_CMD_TEST, NULL, 0);
//	pSendATCmd(ENUM_AT_CMD_IMME, NULL, 0);
//	pSendATCmd(ENUM_AT_CMD_INQ, NULL, 0);
	
}
