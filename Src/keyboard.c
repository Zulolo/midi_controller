
#include "stm32f1xx_hal.h"
#include "keyboard.h"

#define KB_COMM_IF										SPI1
#define KB_COMM_IF_HANDLE							hspi1	//huart1
#define RD_KEYBOARD_CMD								0x15

extern SPI_HandleTypeDef KB_COMM_IF_HANDLE;
extern osSemaphoreId KB_PressedHandle;
extern osSemaphoreId KB_CommIF_RxDoneHandle;
extern osSemaphoreId KB_CommIF_TxDoneHandle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
  if (GPIO_Pin == IO_INT_Pin){
		tHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(KB_PressedHandle, &tHigherPriorityTaskWoken);
		if(tHigherPriorityTaskWoken == pdTRUE){
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}  
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
  if (hspi->Instance == KB_COMM_IF){
		tHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(KB_CommIF_TxDoneHandle, &tHigherPriorityTaskWoken);
		if(tHigherPriorityTaskWoken == pdTRUE){
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}  
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
  if (hspi->Instance == KB_COMM_IF){
		tHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(KB_CommIF_RxDoneHandle, &tHigherPriorityTaskWoken);
		if(tHigherPriorityTaskWoken == pdTRUE){
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}  
  }
}

void KB_Routine(void const * argument)
{
	static uint8_t unRdKeyboardCmd;
	static uint8_t unBuff;
	unRdKeyboardCmd = RD_KEYBOARD_CMD;
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_RESET);
	osDelay(50);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_SET);
	while(1){
		xSemaphoreTake(KB_PressedHandle, portMAX_DELAY);
		HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_RESET);
		if (HAL_OK != HAL_SPI_Transmit_IT(&KB_COMM_IF_HANDLE, &unRdKeyboardCmd, 1)){
			osDelay(1);
			continue;
		}
		xSemaphoreTake(KB_CommIF_TxDoneHandle, portMAX_DELAY);
		if (HAL_OK != HAL_SPI_Receive_IT(&KB_COMM_IF_HANDLE, &unBuff, 1)){
			osDelay(1);
			continue;
		}
		xSemaphoreTake(KB_CommIF_RxDoneHandle, portMAX_DELAY);
		HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
		// pressed key read done
	}
	
}

