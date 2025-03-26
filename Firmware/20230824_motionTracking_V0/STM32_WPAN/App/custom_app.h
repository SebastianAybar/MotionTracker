/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.h
  * @author  MCD Application Team
  * @brief   Header for custom_app.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_APP_H
#define CUSTOM_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eigen_adc_control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  CUSTOM_CONN_HANDLE_EVT,
  CUSTOM_DISCON_HANDLE_EVT,
} Custom_App_Opcode_Notification_evt_t;

typedef struct
{
  Custom_App_Opcode_Notification_evt_t     Custom_Evt_Opcode;
  uint16_t                                 ConnectionHandle;
} Custom_App_ConnHandle_Not_evt_t;
/* USER CODE BEGIN ET */

#define sizeOfDummyData 240
#define numberOfTimesToSendData 10000

#define WATCHDOG_TIMESPAN 5000

#define CONNECTION_CANCEL_TIMESPAN 15000
#define CONNECTION_PASSWORD_TIMESPAN 5000

#define WAIT_FOR_UPDATE_CHAR_TIMESPAN 50

// bluetooth connection termination parameters

#define REMOTE_USER_TERMINATED_CONNECTION 0x13


// aus unbekannten Gründen laufen die software-timer mit der doppelten Geschwindigkeit
#define ADC_TIMESPAN_VOLTAGE_NORMAL 120000 // 120000 Frequenz der Batteriespannungsmessung (im Sleep-Mode ausgeschaltet)

#define ADC_TIMESPAN_BRIGHTNESS 10000 // Frequenz der Messung der Helligkeit (im Sleep-Mode ausgeschaltet)
#define HOLDDOWN_RESET_TIMESPAN 6000 // Dauer, wie lange man den Trigger gedrückt halten muss, bis ein Reset ausgelöst wird x5
#define SLEEP_ACTIVITY_TIMER_TIMESPAN 100000 // Wenn der Trigger im Sleep-Mode gedrückt wird, fängt das Gerät an für die angegebene Zeit zu advertisen

typedef enum
{
  FAST_ADVERTISING,
  SLOW_ADVERTISING,
} Advertising_t;

typedef enum
{
	false,
	true,
	null
} boolean_t;




typedef enum
{
	uninitialized,
	charging,
	noChargingNormalVoltage,
	noChargingLowVoltage,
	noChargingTurnOffVoltage,
} batteryStatus;

typedef enum
{
	notInitialized,
	notifying,
	indicating,

} bleNotifyStatus;

typedef enum
{
	connected,
	advertisingSlow,
	advertisingFast,
	notConnectedNotAdvertising
} bluetoothStatus;

typedef struct
{
  uint8_t batteryStatus;
  uint8_t batteryVoltage;
  uint8_t bluetoothStatus;
  uint8_t irIntensity;
  uint8_t brightness;
  uint8_t status;
  uint8_t playerId; // wird im Spiel verändert, um den Spieler zu identifizieren
  uint8_t killerId;
  uint16_t deviceId; // bleibt immer gleich und ist im Flash des Gerätes hinterlegt, um das Gerät eindeutig zu identifizieren

} Device_Data_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void Custom_APP_Init(void);
void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification);
/* USER CODE BEGIN EF */
  void Measure_Battery_Timer_Callback(void);
  void Setup(void);
  void Start_Advertising(void);
  void Stop_Advertising(void);
  void Adc_Control(void);
  void Watchdog_Timer_Callback(void);
  void Adc_Battery_Measurement_Callback();
  void Update_Advertising_Data(void);
  void Start_Regular_Battery_Measurement(void);
  void Stop_Regular_Battery_Measurement(void);
  void Update_Step_Counter_Char();
  void Update_Voltage_Char(void);
  void Wait_For_Update_Char(void);
  void Update_Data_Rates(uint8_t newDataRate);
  void Send_Dummy_Data(void);
  void Process_Read_Request_For_Data(void);
  void Update_Number_Of_Activity_Data(uint16_t newNumberOfActivityData);
  uint8_t* Get_Date_Time(void);
  void Process_Step_Interrupt(void);

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_APP_H */
