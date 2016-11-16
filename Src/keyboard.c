
#include "stm32f1xx_hal.h"
#include "keyboard.h"

#define KB_COMM_IF										SPI1
#define KB_COMM_IF_HANDLE							hspi1	//huart1
#define RD_KEYBOARD_CMD								0x15

extern SPI_HandleTypeDef KB_COMM_IF_HANDLE;
extern osSemaphoreId KB_PressedHandle;
extern osSemaphoreId KB_ReleaseHandle;
extern osMessageQId tNoteEventQueueHandle;
//static uint8_t unCommErr = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IO_INT_Pin){
		if (HAL_GPIO_ReadPin(IO_INT_GPIO_Port, IO_INT_Pin) == GPIO_PIN_RESET){
			osSemaphoreRelease(KB_PressedHandle);
		}else{
			osSemaphoreRelease(KB_ReleaseHandle);
		} 
  }
}
uint8_t unPressedKey;
void KB_Routine(void const * argument)
{
	static uint8_t unRdPressedKeyCmd;
//	static uint8_t unPressedKey;
	int i;
	unRdPressedKeyCmd = RD_KEYBOARD_CMD;
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_RESET);
	osDelay(50);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_SET);
	osSemaphoreWait(KB_PressedHandle, 0);
	osSemaphoreWait(KB_ReleaseHandle, 0);
	while(1){
		// 1. Read pressed key
		osSemaphoreWait(KB_PressedHandle, portMAX_DELAY);
		HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_RESET);
		if (HAL_OK != HAL_SPI_Transmit(&KB_COMM_IF_HANDLE, &unRdPressedKeyCmd, 1, 2)){
			HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
			while(1);
		}
		
		for (i = 0; i< 50; i++){
			__NOP();
		}
		
		if (HAL_OK != HAL_SPI_Receive(&KB_COMM_IF_HANDLE, &unPressedKey, 1, 2)){
			HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
			while(1);
		}
		HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
		osMessagePut(tNoteEventQueueHandle, unPressedKey + 50, 5);
		
		// 2. Wait until key release
		osSemaphoreWait(KB_ReleaseHandle, portMAX_DELAY);
		osMessagePut(tNoteEventQueueHandle, 0, 5);
	}
	
}

