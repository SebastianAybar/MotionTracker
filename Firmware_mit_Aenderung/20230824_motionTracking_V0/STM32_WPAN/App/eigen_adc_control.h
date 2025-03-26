/*
 * eigen_adc_control.h
 *
 *  Created on: Jun 3, 2022
 *      Author: renew
 */

#ifndef APP_EIGEN_ADC_CONTROL_H_
#define APP_EIGEN_ADC_CONTROL_H_

#define REFERENCE_VOLTAGE 3300 // mv

/*
typedef enum
{
	voltageChannel,
	brightnessChannel
} adcChannels_t;
*/

typedef enum
{
	ready,
	measuringBattery,
	measuringBrightness
} adcStatus_t;

typedef enum
{
	measurementComplete,
} adcResultStatus_t;

typedef struct
{
	uint8_t status;
	uint32_t voltage;
} Battery_Voltage_Result_t;

typedef struct
{
	uint8_t status;
	uint32_t voltage;

} Brightness_Voltage_Result_t;

void Adc_Configure_Channel_Voltage(void);
void Adc_Configure_Channel_Brightness(void);
void Adc_Measure_Brightness_Voltage(void);
void Adc_Measure_Battery_Voltage(void);
void Reinit_Adc_After_Sleep(void);

#endif /* APP_EIGEN_ADC_CONTROL_H_ */
