
#include <string.h>
#include "analog.h"
#include "seq_event.h"
#include "global.h"

typedef enum
{
	BASE_PRESS = -1,
	BASE_NO_ACTION = 0,
	BASE_RELEASE = 1
} BASE_ACTION_T;

typedef enum
{
	BASE_NON_INIT = 0,
	BASE_PRESSED,
	BASE_RELEASED
} BASE_STATUS_T;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern osSemaphoreId ADC1_CNVT_DONEHandle;
extern osSemaphoreId ADC2_CNVT_DONEHandle;
extern osMessageQId tNoteEventQueueHandle;

extern uint8_t unVelocity[];

#define BASE_SENSOR_ADC_HANDLER				hadc1
#define SYSTEM_SENSOR_ADC_HANDLER			hadc2

#define BASE_SENSOR_ADC_SEM						ADC1_CNVT_DONEHandle
#define SYSTEM_SENSOR_ADC_SEM					ADC2_CNVT_DONEHandle

#define BASE_SENSOR_ADC_CHN_NUM				5
#define BASE_SENSOR_ADC_CHN_NUM_USING	2
#define BASE_SENSOR_ADC_BUF_NUM				4

#define BASE_PRESS_THRESHOLD					(400 * BASE_SENSOR_ADC_BUF_NUM)
#define BASE_RELEASE_THRESHOLD				(400 * BASE_SENSOR_ADC_BUF_NUM)

static uint8_t getNoteFromSomething(uint8_t unChannel)
{
	return ((unChannel == 2)?(80):(100));
}

static snd_seq_event_t getPressSomethingSeqEvent(BASE_ACTION_T tAction, uint8_t unChannel)
{
	static snd_seq_event_t tPressKeySeqEvent;
	
	tPressKeySeqEvent.data.note.channel = unChannel;
	tPressKeySeqEvent.data.note.note = getNoteFromSomething(unChannel);
	if (BASE_PRESS == tAction){
		tPressKeySeqEvent.type = SND_SEQ_EVENT_NOTEON;	
		tPressKeySeqEvent.data.note.velocity = unVelocity[unChannel];
	}else if (BASE_RELEASE == tAction){
		tPressKeySeqEvent.type = SND_SEQ_EVENT_NOTEOFF;
		tPressKeySeqEvent.data.note.velocity = 0;
	}else{
		tPressKeySeqEvent.type = SND_SEQ_EVENT_NONE;
	}

	return tPressKeySeqEvent;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1){
		osSemaphoreRelease(BASE_SENSOR_ADC_SEM);
	}
}

BASE_ACTION_T* handleBaseData(uint16_t* pBuff)
{
	static uint16_t unRawADC_Data[BASE_SENSOR_ADC_BUF_NUM][BASE_SENSOR_ADC_CHN_NUM_USING];
	static uint32_t unLastSumADC_Data[BASE_SENSOR_ADC_CHN_NUM_USING];	
	static uint32_t unCurrentSumADC_Data[BASE_SENSOR_ADC_CHN_NUM_USING];
	static BASE_ACTION_T tResult[BASE_SENSOR_ADC_CHN_NUM_USING];
	static BASE_STATUS_T tStatus[BASE_SENSOR_ADC_CHN_NUM_USING];
	static uint8_t unBuffIndex = 0;
	uint8_t unChnIndex = 0;
	
	memcpy(unRawADC_Data[unBuffIndex], pBuff, BASE_SENSOR_ADC_CHN_NUM_USING * sizeof(uint16_t));
	if (unBuffIndex < (BASE_SENSOR_ADC_BUF_NUM - 1)){		
		unBuffIndex++;
		return NULL;
	}else{
		// Average/Just Sum
		memset(unCurrentSumADC_Data, 0, sizeof(unCurrentSumADC_Data));
		for (unBuffIndex = 0; unBuffIndex < BASE_SENSOR_ADC_BUF_NUM; unBuffIndex++){
			for (unChnIndex = 0; unChnIndex < BASE_SENSOR_ADC_CHN_NUM_USING; unChnIndex++){
				unCurrentSumADC_Data[unChnIndex] += unRawADC_Data[unBuffIndex][unChnIndex];
			}
		}
		unBuffIndex = 0;
		// Compare and judge
		for (unChnIndex = 0; unChnIndex < BASE_SENSOR_ADC_CHN_NUM_USING; unChnIndex++){
			switch(tStatus[unChnIndex]){
				case BASE_NON_INIT:
					unLastSumADC_Data[unChnIndex] = unCurrentSumADC_Data[unChnIndex];
					tResult[unChnIndex] = BASE_NO_ACTION;
					tStatus[unChnIndex] = BASE_RELEASED;
				break;

				case BASE_RELEASED:
					if ((unCurrentSumADC_Data[unChnIndex] < unLastSumADC_Data[unChnIndex]) && 
						((unLastSumADC_Data[unChnIndex] - unCurrentSumADC_Data[unChnIndex]) > BASE_PRESS_THRESHOLD)){
						tResult[unChnIndex] = BASE_PRESS;
						tStatus[unChnIndex] = BASE_PRESSED;
					}else{
						tResult[unChnIndex] = BASE_NO_ACTION;
					}
					unLastSumADC_Data[unChnIndex] = unCurrentSumADC_Data[unChnIndex];					
				break;
						
				case BASE_PRESSED:
					if ((unCurrentSumADC_Data[unChnIndex] > unLastSumADC_Data[unChnIndex]) && 
						((unCurrentSumADC_Data[unChnIndex] - unLastSumADC_Data[unChnIndex]) > BASE_RELEASE_THRESHOLD)){
						tResult[unChnIndex] = BASE_RELEASE;
						tStatus[unChnIndex] = BASE_RELEASED;
					}else{
						tResult[unChnIndex] = BASE_NO_ACTION;
					}
					unLastSumADC_Data[unChnIndex] = unCurrentSumADC_Data[unChnIndex];						
				break;
				
				default:
					
				break;
			}
		}
		return tResult;
	}
}

void ADC1_Routine(void const * argument)
{
	static uint16_t unBuff[2][BASE_SENSOR_ADC_CHN_NUM];
	
	uint8_t unIndex = 0;
	uint8_t unBufSelection = 0;
	static uint8_t unChannel = 2;
	BASE_ACTION_T* pActions;
	static snd_seq_event_t tPressSeqEvent;
	
	unVelocity[unChannel] = 100;
	osSemaphoreWait(BASE_SENSOR_ADC_SEM, 0);
	
	HAL_ADC_Start_DMA(&BASE_SENSOR_ADC_HANDLER, (uint32_t *)(unBuff[unBufSelection]), BASE_SENSOR_ADC_CHN_NUM);
	while(1){
		osSemaphoreWait(BASE_SENSOR_ADC_SEM, portMAX_DELAY);

		unBufSelection = (unBufSelection == 0)?(1):(0);
		HAL_ADC_Start_DMA(&BASE_SENSOR_ADC_HANDLER, (uint32_t *)(unBuff[unBufSelection]), BASE_SENSOR_ADC_CHN_NUM);	
		pActions = handleBaseData(unBuff[(unBufSelection == 0)?(1):(0)]);
		if (pActions != NULL){
			for (unIndex = 0; unIndex < BASE_SENSOR_ADC_CHN_NUM_USING; unIndex++){
				tPressSeqEvent = getPressSomethingSeqEvent(pActions[unIndex], unChannel + unIndex);
				if (tPressSeqEvent.type != SND_SEQ_EVENT_NONE){
					xQueueSendToBack(tNoteEventQueueHandle, &tPressSeqEvent, 10);
				}
			}
		}
	}
}

void ADC2_Routine(void const * argument)
{
	while(1){
		osDelay(1000);
	}
}

