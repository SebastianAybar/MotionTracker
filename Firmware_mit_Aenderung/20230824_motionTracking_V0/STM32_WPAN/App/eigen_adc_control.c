/*
 * eigen_adc_control.c
 *
 *  Created on: Jun 3, 2022
 *      Author: renew
 */

#include "main.h"
#include "eigen_adc_control.h"
#include "custom_app.h"
#include "stm32_seq.h"

extern ADC_HandleTypeDef hadc1;
extern Device_Data_t deviceData;
extern uint8_t sleepModeIsActiveFlag;

uint8_t adcStatus = ready;
uint8_t lowVoltageCounter = 0;
uint8_t turnOffVoltageCounter = 0;

ADC_ChannelConfTypeDef sConfig = {0};

void Adc_Measure_Battery_Voltage(void)
{
	uint8_t ret = 0;

	if(adcStatus == ready)
	{
		adcStatus = measuringBattery;

		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		HAL_GPIO_WritePin(CHARGE_ENABLE_GPIO_Port,CHARGE_ENABLE_Pin, 1); // Batterielade-IC deaktivieren, sonst wird die Lade- und nicht die Batteriespannung gemessen
		HAL_GPIO_WritePin(VBAT_ADC_ENABLE_GPIO_Port, VBAT_ADC_ENABLE_Pin, 0); // Spannungsteiler aktivieren

		HAL_Delay(100); // zwischen ADC_BAT_ENABLE und der ADC-Messung muss unbedingt eine Pause sein, sonst zeigt der Mikrocontroller merkwürdiges Verhalten:
		// bei VBAT < 3,8 V funktioniert alles normal, bei VBAT > 3,8 V switcht VREFBUF plötzlich und man kann an dem externen Pin 3,3 V messen
		// keine Ahnung, wie der Mikrocontroller die 3,3 V generiert, er selbst wird lediglich mit 3,0 V versorgt
		// die Register vom VREFBUF sagen VREFBUF ist ready, also keine Auffälligkeiten
		// die Messung ist dann entsprechend falsch (mit 3,3 V Referenzspannung)
		// wenn man diese Pause einbaut, besteht der Fehler nicht

		HAL_ADC_Stop_IT(&hadc1); // MUSS UNBEDINGT HIER STEHEN, sonst liefert der ADC im Schlaf-Modus, wenn der Debugger nicht angeschlossen ist
								// manchmal den Wert 0
		ret=HAL_ADC_Start_IT(&hadc1);

		if(ret != HAL_OK)
		{
			printf("error starting battery voltage measurement\r\n");
		} else
		{
			printf("started battery voltage measurement\r\n");
		}
	}

}


void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	deviceData.batteryVoltage = 1;
	UTIL_SEQ_SetTask(1<<CFG_TASK_BATTERY_VOLTAGE_CALLBACK_ID, 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint32_t adcResult = 0;
	if(adcStatus == measuringBattery)
	{
		// ################### Battery

		Battery_Voltage_Result_t result;

		HAL_GPIO_WritePin(CHARGE_ENABLE_GPIO_Port,CHARGE_ENABLE_Pin, 0); // Laden wieder aktivieren
		HAL_GPIO_WritePin(VBAT_ADC_ENABLE_GPIO_Port, VBAT_ADC_ENABLE_Pin, 1); // Spannungsteiler deaktivieren
		result.status = measurementComplete;

		adcResult = HAL_ADC_GetValue(&hadc1);
		printf("eigen_adc_control.c adc value is %lu\r\n", adcResult);
		result.voltage = adcResult*2*REFERENCE_VOLTAGE/255;

		printf("eigen_adc_control.c Battery Voltage is %lu\r\n", result.voltage);

		deviceData.batteryVoltage = result.voltage/100; // 8 bit for advertising (immer abgerundet auf Zehntel)

		UTIL_SEQ_SetTask(1<<CFG_TASK_BATTERY_VOLTAGE_CALLBACK_ID, 1);

		adcStatus = ready;


	}

}
