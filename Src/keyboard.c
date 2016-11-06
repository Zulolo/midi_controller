
#include "stm32f1xx_hal.h"
#include "keyboard.h"

#define KB_COMM_IF										SPI1
#define KB_COMM_IF_HANDLE							hspi1	//huart1
#define RD_KEYBOARD_CMD								0x15

extern SPI_HandleTypeDef KB_COMM_IF_HANDLE;
extern osSemaphoreId KB_PressedHandle;
extern osSemaphoreId KB_ReleaseHandle;
//static uint8_t unCommErr = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
  if (GPIO_Pin == IO_INT_Pin){
		tHigherPriorityTaskWoken = pdFALSE;
		if (HAL_GPIO_ReadPin(IO_INT_GPIO_Port, IO_INT_Pin) == GPIO_PIN_RESET){
			xSemaphoreGiveFromISR(KB_PressedHandle, &tHigherPriorityTaskWoken);
		}else{
			xSemaphoreGiveFromISR(KB_ReleaseHandle, &tHigherPriorityTaskWoken);
		}
		if(tHigherPriorityTaskWoken == pdTRUE){
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}  
  }
}

void KB_Routine(void const * argument)
{
	static uint8_t unRdPressedKeyCmd;
	static uint8_t unPressedKey;
	unRdPressedKeyCmd = RD_KEYBOARD_CMD;
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_RESET);
	osDelay(50);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_SET);
	xSemaphoreTake(KB_PressedHandle, portMAX_DELAY);
	while(1){
		// 1. Read pressed key
		xSemaphoreTake(KB_PressedHandle, portMAX_DELAY);
		HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_RESET);
		if (HAL_OK != HAL_SPI_Transmit(&KB_COMM_IF_HANDLE, &unRdPressedKeyCmd, 1, 2)){
			HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
			while(1);
		}

		if (HAL_OK != HAL_SPI_Receive(&KB_COMM_IF_HANDLE, &unPressedKey, 1, 2)){
			HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
			while(1);
		}
		HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);

		// 2. Wait until key release
		xSemaphoreTake(KB_ReleaseHandle, portMAX_DELAY);

		// pressed key read done

	}
	
}

