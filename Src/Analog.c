
#include "analog.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern osSemaphoreId ADC1_CNVT_DONEHandle;
extern osSemaphoreId ADC2_CNVT_DONEHandle;

#define BASE_SENSOR_ADC_HANDLER				hadc1
#define SYSTEM_SENSOR_ADC_HANDLER			hadc2

#define BASE_SENSOR_ADC_SEM						ADC1_CNVT_DONEHandle
#define SYSTEM_SENSOR_ADC_SEM					ADC2_CNVT_DONEHandle

#define BASE_SENSOR_ADC_CHN_NUM				5
#define BASE_SENSOR_ADC_BUF_NUM				4

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1){
		osSemaphoreRelease(BASE_SENSOR_ADC_SEM);
	}
}

void ADC1_Routine(void const * argument)
{
	static uint32_t unBuff[BASE_SENSOR_ADC_BUF_NUM][BASE_SENSOR_ADC_CHN_NUM];
	uint8_t unBufIndex = 0;
	osSemaphoreWait(BASE_SENSOR_ADC_SEM, 0);
	
	HAL_ADC_Start_DMA(&BASE_SENSOR_ADC_HANDLER, unBuff[0], BASE_SENSOR_ADC_CHN_NUM);
	while(1){
		osSemaphoreWait(BASE_SENSOR_ADC_SEM, portMAX_DELAY);
		if (unBufIndex < BASE_SENSOR_ADC_BUF_NUM){
			unBufIndex++;
			HAL_ADC_Start_DMA(&BASE_SENSOR_ADC_HANDLER, unBuff[unBufIndex], BASE_SENSOR_ADC_CHN_NUM);
		}else{
			unBufIndex = 0;
			HAL_ADC_Start_DMA(&BASE_SENSOR_ADC_HANDLER, unBuff[unBufIndex], BASE_SENSOR_ADC_CHN_NUM);		
		}		
	}
}

void ADC2_Routine(void const * argument)
{
	while(1){
		osDelay(1000);
	}
}

