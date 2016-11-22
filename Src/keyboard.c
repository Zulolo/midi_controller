
#include "stm32f1xx_hal.h"
#include "keyboard.h"

#define KB_COMM_IF										SPI1
#define KB_COMM_IF_HANDLE							hspi1	//huart1
#define KB_COMM_SPI_TIM								htim4
#define RD_KEYBOARD_CMD								0x15
#define KB_READ_KEY_CMD								0xEE

extern SPI_HandleTypeDef KB_COMM_IF_HANDLE;
extern osSemaphoreId KB_PressedHandle;
extern osSemaphoreId KB_ReleaseHandle;
extern osSemaphoreId KB_SPI_BusyHandle;
extern osMessageQId tNoteEventQueueHandle;
extern TIM_HandleTypeDef htim4;
//static uint8_t unCommErr = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IO_INT_Pin){
		if (HAL_GPIO_ReadPin(IO_INT_GPIO_Port, IO_INT_Pin) == GPIO_PIN_RESET){
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			osSemaphoreRelease(KB_PressedHandle);
		} 
  }
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim->Instance == KB_COMM_SPI_TIM.Instance){
//		osSemaphoreRelease(KB_SPI_BusyHandle);
//	}
//}

int32_t waitUS(uint16_t unWaitUS)
{
	KB_COMM_SPI_TIM.Init.Period = unWaitUS - 1;

	osSemaphoreWait(KB_SPI_BusyHandle, 0); 	
  if (HAL_TIM_Base_Init(&KB_COMM_SPI_TIM) != HAL_OK){
    return (-1);
  }
//	__HAL_TIM_CLEAR_FLAG(&KB_COMM_SPI_TIM, TIM_FLAG_UPDATE);
	__HAL_TIM_CLEAR_IT(&KB_COMM_SPI_TIM, TIM_IT_UPDATE);
  if (HAL_TIM_Base_Start_IT(&KB_COMM_SPI_TIM) != HAL_OK){
    return (-1);
  }
	osSemaphoreWait(KB_SPI_BusyHandle, portMAX_DELAY);
	return 0;
}

void waitUntilOneByteSent(void)
{	// This is only valid when SPI at 8MHz and CPU run at 64MHz
	// More save CPU than use 1us timer above
	uint32_t unCNT;
	for(unCNT = 0; unCNT < 5; unCNT++){
		__NOP();
	}
}

uint8_t readPressedKey(void)
{
	uint8_t unPressedKey;
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_RESET);
	__HAL_SPI_ENABLE(&KB_COMM_IF_HANDLE);
	KB_COMM_IF_HANDLE.Instance->DR = KB_READ_KEY_CMD;
	
	// Wait until all bits have been sent out
//	waitUS(200);
	waitUntilOneByteSent();
	
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
	
	waitUntilOneByteSent();
	
	// send dummy data to read
	KB_COMM_IF_HANDLE.Instance->DR = 0xA5;
	waitUS(200);
	
	unPressedKey = KB_COMM_IF_HANDLE.Instance->DR;
	__HAL_SPI_DISABLE(&KB_COMM_IF_HANDLE);
	return unPressedKey;
}

uint8_t unPressedKey;
void KB_Routine(void const * argument)
{
//	static uint8_t unRdPressedKeyCmd;
//	static uint8_t unPressedKey;
//	int i;
//	unRdPressedKeyCmd = RD_KEYBOARD_CMD;
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_RESET);
	osDelay(50);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_SET);
	osSemaphoreWait(KB_PressedHandle, 0);
	osSemaphoreWait(KB_ReleaseHandle, 0);
	osSemaphoreWait(KB_SPI_BusyHandle, 0);
	while(1){
		// 1. Read pressed key
//		osSemaphoreWait(KB_PressedHandle, portMAX_DELAY);
		unPressedKey = readPressedKey();	
		osMessagePut(tNoteEventQueueHandle, unPressedKey + 50, 5);
		
		// 2. Wait until key release
		while(unPressedKey != 0){
			unPressedKey = readPressedKey();
			osDelay(2);
		}
		osSemaphoreWait(KB_ReleaseHandle, portMAX_DELAY);
		osMessagePut(tNoteEventQueueHandle, 0, 5);
		
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	
}

