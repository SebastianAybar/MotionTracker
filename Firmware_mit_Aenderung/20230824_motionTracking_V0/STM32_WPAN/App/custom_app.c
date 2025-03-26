/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eigen_adc_control.h"
#include "app_ble.h"
#include "eigen_ble.h"
#include "BMI323_eigen.h"
#include "eigen_activities.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* activityData */
  uint8_t               Sactdat_Notification_Status;
  /* deviceData */
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t measure_voltage_timer_id; // eine ID wird angelegt für den Software-Timer, der genutzt wird, um in regelmäßigen Abständen die Batteriespannung zu messen
  uint8_t watchdog_timer_id;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
//uint8_t UpdateCharData[247];
//uint8_t NotifyCharData[247];

//uint8_t SecureReadData;


//uint16_t batteryVoltage=0;
//uint16_t sensorBrightnessAdc=0;

extern ADC_HandleTypeDef hadc1;
extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
extern uint8_t sensorIsInitialised;
extern uint8_t savedData[NUMBER_OF_MAX_SAVED_DATA][BYTES_IN_SAVED_DATA];
//für eigen angelegte Characteristic
extern uint8_t rawData[768];
extern uint16_t nextOut;


uint8_t ret = 0;

uint16_t connectionHandle;
uint8_t waitForRessourcesToNotify=0;
uint8_t notifyOrIndicate=notInitialized;
uint8_t enableDataStreamOverBle = 0;
uint16_t bleConnectionHandle; // stores the handle for the BLE connection after the connection event
Device_Data_t deviceData;
uint8_t advertising_started_flag = false;
uint8_t regularBatteryMeasurementIsActiveFlag = false;
uint8_t updateVoltageCharPending = false;
uint32_t stepCounterInt = 0;

//static const uint16_t* ID = (uint16_t*) 0x08016FF0; // Adresse, unter der im Flash die ID des uC hinterlegt ist

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* activityData */
static void Custom_Sactdat_Update_Char(void);
static void Custom_Sactdat_Send_Notification(void);
/* deviceData */

/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */
	  printf("custom_app.c app_notification\r\n");

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

	 /* eigene Characteristic */
	 case CUSTOM_STM_NEW_CHAR_READ_EVT:
	 /* USER CODE BEGIN CUSTOM_STM_SACTDAT_READ_EVT */
	   printf("custom_app.c s new char read event\r\n");
	   Process_Read_Request_For_Data_v2();
	 /* USER CODE END CUSTOM_STM_SACTDAT_READ_EVT */
	 break;

    /* activityData */
    case CUSTOM_STM_SACTDAT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SACTDAT_READ_EVT */
    	printf("custom_app.c s activity data read event\r\n");
		Process_Read_Request_For_Data();
      /* USER CODE END CUSTOM_STM_SACTDAT_READ_EVT */
      break;

    case CUSTOM_STM_SACTDAT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SACTDAT_NOTIFY_ENABLED_EVT */
    	printf("custom_app.c s activity data notify enabled event\r\n");
      /* USER CODE END CUSTOM_STM_SACTDAT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SACTDAT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SACTDAT_NOTIFY_DISABLED_EVT */
    	printf("custom_app.c s activity data notify disabled event\r\n");
      /* USER CODE END CUSTOM_STM_SACTDAT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NUMACTDAT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NUMACTDAT_READ_EVT */
    	printf("custom_app.c num activity data read event\r\n");
      /* USER CODE END CUSTOM_STM_NUMACTDAT_READ_EVT */
      break;

    case CUSTOM_STM_RTC_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RTC_READ_EVT */
    	printf("custom_app.c rtc read event\r\n");
    	RTC_TimeTypeDef rtcTime;
    	RTC_DateTypeDef rtcDate;
    	uint8_t transmitData[6] =  {0};
    	uint8_t returnValue = 0;

		HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

		transmitData[0] = rtcDate.Date;
		transmitData[1] = rtcDate.Month;
		transmitData[2] = rtcDate.Year;
		transmitData[3] = rtcTime.Hours;
		transmitData[4] = rtcTime.Minutes;
		transmitData[5] = rtcTime.Seconds;

		printf("custom_app.c accel updating date: %u.%u.%u %u:%u:%u\r\n",
				transmitData[0], transmitData[1], transmitData[2], transmitData[3], transmitData[4], transmitData[5]);

    	returnValue = Ble_Update_Characteristic(CUSTOM_STM_RTC, transmitData, sizeof(transmitData));
      /* USER CODE END CUSTOM_STM_RTC_READ_EVT */
      break;

    case CUSTOM_STM_RTC_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RTC_WRITE_EVT */
    	printf("custom_app.c rtc write event\r\n");

    	// a new date-time has to be provided in the format of an 8 bit array:
    	// [day], [month], [year], [hour], [minute], [second]
    	// year is a value between 0 and 99

		RTC_TimeTypeDef sTime = {0};
		RTC_DateTypeDef sDate = {0};

		uint8_t newDay = 0;
		uint8_t newMonth = 0;
		uint8_t newYear = 0;
		uint8_t newHour = 0;
		uint8_t newMinute = 0;
		uint8_t newSecond = 0;

		printf("raw data: ");
		for(uint8_t i=0; i< 6; i++)
		{
			printf("%u - ", pNotification->DataTransfered.pPayload[i]);
		}
		printf("\r\n");
		newDay = pNotification->DataTransfered.pPayload[0];
		newMonth = pNotification->DataTransfered.pPayload[1];
		newYear = pNotification->DataTransfered.pPayload[2];
		newHour = pNotification->DataTransfered.pPayload[3];
		newMinute = pNotification->DataTransfered.pPayload[4];
		newSecond = pNotification->DataTransfered.pPayload[5];

		printf("new date set: %u.%u.%u %u:%u:%u\r\n", newDay, newMonth, newYear, newHour, newMinute, newSecond);

		if(newDay < 32 && newMonth < 13 && newYear < 100 && newHour < 25 && newMinute < 61 && newSecond < 61)
		{
			  sTime.Hours = newHour;
			  sTime.Minutes = newMinute;
			  sTime.Seconds = newSecond;
			  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
			  {
			    printf("updating time NOT successful\r\n");
			  } else
			  {
				  printf("updated time successfully\r\n");
			  }

			  sDate.Month = newMonth;
			  sDate.Date = newDay;
			  sDate.Year = newYear;

			  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
			  {
				    printf("updating date NOT successful\r\n");
			  } else
			  {
				  printf("updated date successfully\r\n");
			  }
		} else
		{
			printf("incorrect time format\r\n");
		}

		// just for debugging:
		printf("checking new date: ");
		uint8_t* dateTimePointer = Get_Date_Time();
		for(uint8_t i=0; i<6; i++)
		{
			printf("%u ", dateTimePointer[i]);
		}
		printf("\r\n");


		printf("\r\n");
      /* USER CODE END CUSTOM_STM_RTC_WRITE_EVT */
      break;

    case CUSTOM_STM_STEPCOUNTERINT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_STEPCOUNTERINT_READ_EVT */
    	printf("custom_app.c step counter read event\r\n");
    	// this code is executed when the step counter characteristic is read via BLE
    	// updates the step counter characteristic with the current 32 bit value of the global variable

    	transmitData[0] = stepCounterInt >> 24;
    	transmitData[1] = stepCounterInt >> 16;
    	transmitData[2] = stepCounterInt >> 8;
    	transmitData[3] = stepCounterInt; // maybe change upper and lower byte, depending on host

    	returnValue = Ble_Update_Characteristic(CUSTOM_STM_STEPCOUNTERINT, transmitData, sizeof(transmitData)); // updates the value of the characteristic to be read

    	if(returnValue != BLE_STATUS_SUCCESS)
    	{
    		printf("custom_app.c updating data char NOT successful\r\n");
    	}
      /* USER CODE END CUSTOM_STM_STEPCOUNTERINT_READ_EVT */
      break;

    case CUSTOM_STM_STEPCOUNTERINT_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_STEPCOUNTERINT_WRITE_EVT */
    	printf("custom_app.c step counter write event\r\n");
    	if(pNotification->DataTransfered.pPayload[0] == 0)
    	{
    		stepCounterInt = 0; // sets step counter (global variable) to 0
    		Clear_Step_Counter_Reg();
    	}
    	// BLE update char is not necessary because the uC updates the char value to the latest written value automatically
      /* USER CODE END CUSTOM_STM_STEPCOUNTERINT_WRITE_EVT */
      break;

    /* deviceData */
    case CUSTOM_STM_BATLVL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATLVL_READ_EVT */
    	printf("custom_app.c batLvl read event\r\n");
      /* USER CODE END CUSTOM_STM_BATLVL_READ_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */
    	printf("custom_app.c notification default event\r\n");
      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
    	printf("connected\r\n");
    	Ble_Connected_To_Client();
    	connectionHandle = pNotification->ConnectionHandle;
//    	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    	ret = aci_gatt_exchange_config(connectionHandle);
    	if (ret != BLE_STATUS_SUCCESS)
    	{
//    		HAL_GPIO_WritePin(TESTPIN2_GPIO_Port, TESTPIN2_Pin, 1);
    		printf("custom_app.c error gatt exchange config\r\n");
    		printf("return value is %i\r\n", ret);
//    		HAL_GPIO_WritePin(TESTPIN2_GPIO_Port, TESTPIN2_Pin, 0);
    	} else
    	{
    		printf("success gatt exchange\r\n");
    	}

//    	UTIL_SEQ_SetTask(1<<CFG_TASK_STAY_AWAKE_ID, 1);
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
    	printf("DISCON_HANDLE\r\n");
//    	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    	enableDataStreamOverBle = 0;
    	notifyOrIndicate=notInitialized;
    	Ble_Disconnected_From_Client();

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */
    	printf("custom_app.c Default app notificationr\n");
      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

	UTIL_SEQ_RegTask(1<<CFG_TASK_ADC_CONTROL_ID, UTIL_SEQ_RFU, Measure_Battery_Timer_Callback);
	UTIL_SEQ_RegTask(1<<CFG_TASK_SETUP_ID, UTIL_SEQ_RFU, Setup);
	UTIL_SEQ_RegTask(1<<CFG_TASK_UPDATE_ADVERTISING_DATA_ID, UTIL_SEQ_RFU, Update_Advertising_Data);
	UTIL_SEQ_RegTask(1<<CFG_TASK_BATTERY_VOLTAGE_CALLBACK_ID, UTIL_SEQ_RFU, Adc_Battery_Measurement_Callback);
	UTIL_SEQ_RegTask(1<<CFG_TASK_MEASURE_BATTERY_ID, UTIL_SEQ_RFU, Adc_Measure_Battery_Voltage);
	UTIL_SEQ_RegTask(1<<CFG_TASK_UPDATE_CHAR_VOLTAGE_ID, UTIL_SEQ_RFU, Update_Voltage_Char);
	UTIL_SEQ_RegTask(1<<CFG_TASK_START_ADVERTISING_ID, UTIL_SEQ_RFU, Start_Advertising);
	UTIL_SEQ_RegTask(1<<CFG_TASK_STOP_ADVERTISING_ID, UTIL_SEQ_RFU, Stop_Advertising);
	UTIL_SEQ_RegTask(1<<CFG_TASK_READ_SENSOR_DATA_ID, UTIL_SEQ_RFU, Read_Sensor_Data);
	UTIL_SEQ_RegTask(1<<CFG_TASK_PROCESS_STEP_INTERRUPT_ID, UTIL_SEQ_RFU, Process_Step_Interrupt);

	ret=HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(Custom_App_Context.measure_voltage_timer_id), hw_ts_Repeated, Measure_Battery_Timer_Callback);
	if(ret != hw_ts_Successful)
	{
		printf(" Error creating timer 1\r\n");
	}
	ret=HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(Custom_App_Context.watchdog_timer_id), hw_ts_Repeated, Watchdog_Timer_Callback);
	if(ret != hw_ts_Successful)
	{
		printf(" Error creating timer 3\r\n");
	}

	HW_TS_Start(Custom_App_Context.watchdog_timer_id, WATCHDOG_TIMESPAN);
	UTIL_SEQ_SetTask(1<<CFG_TASK_SETUP_ID, 1);
	//Start_Regular_Battery_Measurement();
	deviceData.bluetoothStatus = advertisingFast;
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* activityData */
void Custom_Sactdat_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sactdat_UC_1*/
  // this is automatically generated by ST but it is not used
  /* USER CODE END Sactdat_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SACTDAT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sactdat_UC_Last*/

  /* USER CODE END Sactdat_UC_Last*/
  return;
}

void Custom_Sactdat_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sactdat_NS_1*/
  // this is automatically generated by ST but it is not used
  /* USER CODE END Sactdat_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SACTDAT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sactdat_NS_Last*/

  /* USER CODE END Sactdat_NS_Last*/

  return;
}

/* deviceData */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void Process_Read_Request_For_Data_v2(void)
{
	uint8_t transmitRawData[768] =  {0};
	uint8_t returnValue = 0;

	for(uint8_t i = 0; i < 768; i++)
	{
		transmitRawData[i] = rawData[i];
	}
	returnValue = Ble_Update_Characteristic(CUSTOM_STM_EIGEN_ACT, transmitRawData, sizeof(transmitRawData)); // updates the value of the characteristic to be read
	if(returnValue != BLE_STATUS_SUCCESS)
	{
		printf("custom_app.c updating data char NOT successful\r\n");
	}
}

void Process_Read_Request_For_Data(void)
{
	printf("custom_app.c Process_Read_Request_For_Data\r\n");
	// this function is called each time the host reads the characteristic "savedActData"
	// after this function is executed, the value of the characteristic will be returned to the host

	uint8_t transmitData[36] =  {0}; // array to be read, the number of bytes to be read are specified in CubeIDE
	uint8_t returnValue = 0;
//
	// filling the dataBuffer with dummy data
	for(uint8_t i=0; i<36; i++)
	{
		transmitData[i]=savedData[nextOut][i];
	}
	// updates the value of the characteristic to be read
	returnValue = Ble_Update_Characteristic(CUSTOM_STM_SACTDAT, transmitData, sizeof(transmitData)); 

	if(returnValue != BLE_STATUS_SUCCESS)
	{
		printf("custom_app.c updating data char NOT successful\r\n");
	}

	if(returnValue == BLE_STATUS_SUCCESS)
	{
		Move_NextOut();
	}
}

void Update_Number_Of_Activity_Data(uint16_t newNumberOfActivityData)
{
	printf("custom_app.c Update_Number_Of_Activity_Data\r\n");
	// this function updates the numActData char (16 bit)

	uint8_t transmitData[2] =  {0}; // the number of bytes to be read are specified in CubeIDE
	uint8_t returnValue = 0;

	transmitData[0] = newNumberOfActivityData >> 8;
	transmitData[1] = newNumberOfActivityData; // maybe change upper and lower byte, depending on host

	returnValue = Ble_Update_Characteristic(CUSTOM_STM_NUMACTDAT, transmitData, sizeof(transmitData)); // updates the value of the characteristic to be read

	if(returnValue != BLE_STATUS_SUCCESS)
	{
		printf("custom_app.c updating data char NOT successful\r\n");
	}
}

uint8_t* Get_Date_Time(void)
{
	printf("custom_app.c Get_Date_Time\r\n");
	// this function returns a pointer to an 8-bit-array that contains the time and date in the format:
	// [day], [month], [year], [hour], [minute], [second]
	// year is a value between 0 and 99

	static uint8_t dateTimeArray[6] = {0}; // array keeps values between function calls
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	dateTimeArray[0] = sDate.Date;
	dateTimeArray[1] = sDate.Month;
	dateTimeArray[2] = sDate.Year;
	dateTimeArray[3] = sTime.Hours;
	dateTimeArray[4] = sTime.Minutes;
	dateTimeArray[5] = sTime.Seconds;

	return dateTimeArray;
}

void Update_Voltage_Char(void)
{
	updateVoltageCharPending = false;
	uint8_t transmitData[16] =  {0};
	uint8_t returnValue = 0;

	printf("custom_app.c Update_Voltage_Char\r\n");

	transmitData[0] =  deviceData.batteryVoltage;
	returnValue = Ble_Update_Characteristic(CUSTOM_STM_BATLVL, transmitData, 1);



	if(returnValue != BLE_STATUS_SUCCESS)
	{
		// wenn CPU2 busy war, kann es sein, dass die charakteristic nicht upgedated wurde.
		// In diesem Fall wird der Task erneut gescheduled
		//UTIL_SEQ_SetTask(1<<CFG_TASK_UPDATE_CHAR_VOLTAGE_ID, 1);
		printf("custom_app.c updating voltage char unsuccessful\r\n");
//		updateVoltageCharPending = true;
//		HW_TS_Start(Custom_App_Context.wait_for_update_char_timer_id, WAIT_FOR_UPDATE_CHAR_TIMESPAN);
	}
}

void Watchdog_Timer_Callback(void)
{
	HAL_IWDG_Refresh(&hiwdg);
}


void Start_Regular_Battery_Measurement(void)
{
	if(regularBatteryMeasurementIsActiveFlag == false)
	{
		printf("regular battery measurement started\r\n");
		regularBatteryMeasurementIsActiveFlag = true;
		HW_TS_Start(Custom_App_Context.measure_voltage_timer_id, ADC_TIMESPAN_VOLTAGE_NORMAL);
	}
	//HW_TS_Start(Custom_App_Context.measure_voltage_timer_id, ADC_TIMESPAN_VOLTAGE_NORMAL);
}

void Stop_Regular_Battery_Measurement(void)
{
	if(regularBatteryMeasurementIsActiveFlag == true)
	{
		printf("regular battery measurement stopped\r\n");
		regularBatteryMeasurementIsActiveFlag = false;
		HW_TS_Stop(Custom_App_Context.measure_voltage_timer_id);
	}
}

void Measure_Battery_Timer_Callback(void)
{
	// measures battery voltage and brightness
	printf("measure battery timer callback\r\n");

	UTIL_SEQ_SetTask(1<<CFG_TASK_MEASURE_BATTERY_ID, 1);

}

void Adc_Battery_Measurement_Callback()
{
	printf("custom_app.c Battery Measurement Callback\r\n");

	UTIL_SEQ_SetTask(1<<CFG_TASK_UPDATE_CHAR_VOLTAGE_ID, 1);
	UTIL_SEQ_SetTask(1<<CFG_TASK_UPDATE_ADVERTISING_DATA_ID, 1);
}


void Update_Advertising_Data(void)
{
	// Task for scheduler
	Ble_Update_Advertising_Data();
}

void Process_Step_Interrupt(void)
{
	// this task is scheduled after the uC receives a step detector interrupt
	// this task is executed outside the interrupt routine
//	printf("custom_app.c Process_Step_Interrupt\r\n");
	stepCounterInt++;
	printf("custom_app.c step counter interrupt: %lu\r\n", stepCounterInt);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	Clear_Interrupt(); // seems like this is not necessary to reset the interrupt

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	printf("custom_app.c external interrupt\r\n");
	switch(GPIO_Pin)
	{
	case INT2_Pin:
//		printf("custom_app.c INT2 Interrupt (FIFO)\r\n");
//		HAL_GPIO_WritePin(TESTPIN2_GPIO_Port, TESTPIN2_Pin, 1);
		if(sensorIsInitialised==1)
		{
//			printf("custom_app.c INT2 reading data\r\n");
			// watermark interrupt does not have to be manually reset
			// it is reset once the fill level of the data is lower than the watermark
			UTIL_SEQ_SetTask(1<<CFG_TASK_READ_SENSOR_DATA_ID, 1);
		} else
		{
			printf("custom_app.c Sensor not initialized\r\n");
		}

		break;
	case INT1_Pin: // is not used in this version of the code
//		printf("custom_app.c INT1 Interrupt (Step Detector)\r\n");
		UTIL_SEQ_SetTask(1<<CFG_TASK_PROCESS_STEP_INTERRUPT_ID, 1);

		break;
	}

}

void Start_Advertising(void)
{
	Ble_Start_Advertising(FAST_ADVERTISING);
}

void Stop_Advertising(void)
{
	Ble_Stop_Advertising();
}

void aci_gatt_server_confirmation_event( uint16_t Connection_Handle ){
	printf("custom_app.c confirmation event ##########\r\n");
	// diese Callback-Funktion sollte eigentlich aufgerufen werden, wenn ein acknowledge vom Client eintrifft
	// und der Client bestätigt, dass er das letzte Indicate erhalten hat
	// aber die Funktion wird nicht aufgerufen...
}

void Setup()
{
	// Dieser Task wird nach der Initialisierung als erstes aufgerufen
	printf("custom_app.c setup\r\n");

	HAL_GPIO_WritePin(V3_ENABLE_GPIO_Port, V3_ENABLE_Pin, 1);
	HAL_Delay(50); // Spannungsregler einschalten und kurz warten
	UTIL_SEQ_SetTask(1<<CFG_TASK_MEASURE_BATTERY_ID, 1);

	Sensor_Init();
	Start_Regular_Battery_Measurement();
	printf("custom_app.c ################################ setup\r\n");

}
void aci_att_exchange_mtu_resp_event( uint16_t Connection_Handle,
                                             uint16_t Server_RX_MTU )
{
	printf("custom_app.c ATT exchange MTU response event\r\n");
	printf("client can handle an MTU size of: %u", Server_RX_MTU);
}

// WICHTIG: Wenn notify und indicate aktiviert sind, haben verschiedene Bibiliotheken (PyDBUS)
// Deshalb NUR Notify aktivieren (nicht indicate)

/* USER CODE END FD_LOCAL_FUNCTIONS*/
