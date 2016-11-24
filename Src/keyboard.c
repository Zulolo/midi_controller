
#include "stm32f1xx_hal.h"
#include "keyboard.h"
#include "seq_event.h"
#include "global.h"

#define KB_COMM_IF										SPI1
#define KB_COMM_IF_HANDLE							hspi1	//huart1
#define KB_COMM_SPI_TIM								htim4
#define KB_READ_KEY_CMD								0xEE

extern SPI_HandleTypeDef KB_COMM_IF_HANDLE;
extern osSemaphoreId KB_PressedHandle;
extern osSemaphoreId KB_ReleaseHandle;
extern osSemaphoreId KB_SPI_BusyHandle;
extern osMessageQId tNoteEventQueueHandle;
extern TIM_HandleTypeDef htim4;
extern uint8_t unVelocity[];
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

// If connected to multi-keyboard, channel can be used to identify
uint8_t readPressedKey(uint8_t unChannel)
{
	static uint8_t unReadKeyCmd = KB_READ_KEY_CMD;
	uint8_t unPressedKey;
	
	HAL_SPI_Transmit(&KB_COMM_IF_HANDLE, &unReadKeyCmd, 1, 10); 
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_RESET);
	__NOP();
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);

	// send dummy data to receive
	HAL_SPI_TransmitReceive(&KB_COMM_IF_HANDLE, &unReadKeyCmd, &unPressedKey, 1, 10);
	return unPressedKey;
}

portBASE_TYPE isKeyReleased(void)
{
	static uint8_t unReadKeyCmd = KB_READ_KEY_CMD;
	
	HAL_SPI_Transmit(&KB_COMM_IF_HANDLE, &unReadKeyCmd, 1, 10); 
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_RESET);
	__NOP();
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
	// The first bit will be put on the Dout immediately after LOAD rising (Don't need clock)
	if (HAL_GPIO_ReadPin(IO_MISO_GPIO_Port, IO_MISO_Pin) == GPIO_PIN_RESET){
		return pdTRUE;
	}else{
		return pdFALSE;
	}
}

void initKeyboard(void)
{
// 0000 0100 0000 0010
	static uint8_t unInitKeyCmds[] = {0x04, 0x02};
	HAL_SPI_Transmit(&KB_COMM_IF_HANDLE, unInitKeyCmds, sizeof(unInitKeyCmds), 10); 
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_RESET);
	__NOP();
	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
}

//snd_seq_event_t getPressKeySeqEvent(uint8_t unPressedKey)
//{
//	
//}
uint8_t getNoteFromKey(uint8_t unPressedKey)
{
	return unPressedKey;
}

uint8_t unPressedKey;
void KB_NoteRoutine(void const* argument)
{
	static snd_seq_event_t tPressKeySeqEvent;
	static uint8_t unChannel = 1;

	HAL_GPIO_WritePin(IO_NCS_GPIO_Port, IO_NCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_RESET);
	osDelay(50);
	HAL_GPIO_WritePin(IO_RST_GPIO_Port, IO_RST_Pin, GPIO_PIN_SET);
	initKeyboard();
	osSemaphoreWait(KB_PressedHandle, 0);
	
	tPressKeySeqEvent.data.note.channel = unChannel;
	unVelocity[unChannel] = 100;
	while(1){
		// 1. Read pressed key
		osSemaphoreWait(KB_PressedHandle, portMAX_DELAY);
		unPressedKey = readPressedKey(unChannel);	
		tPressKeySeqEvent.type = SND_SEQ_EVENT_NOTEON;
		tPressKeySeqEvent.data.note.note = getNoteFromKey(unPressedKey);
		tPressKeySeqEvent.data.note.velocity = unVelocity[unChannel];
//		tPressKeySeqEvent = getPressKeySeqEvent(unPressedKey);
		xQueueSendToBack(tNoteEventQueueHandle, &tPressKeySeqEvent, 5);
		
		// 2. Wait until key release
		while(isKeyReleased() != pdTRUE){
			osDelay(2);
		}
//		tPressKeySeqEvent = getPressKeySeqEvent(0);
		unPressedKey = 0;
		tPressKeySeqEvent.type = SND_SEQ_EVENT_NOTEOFF;
		tPressKeySeqEvent.data.note.note = getNoteFromKey(unPressedKey);
		tPressKeySeqEvent.data.note.velocity = unVelocity[unChannel];
		xQueueSendToBack(tNoteEventQueueHandle, &tPressKeySeqEvent, 5);
		
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	
}

