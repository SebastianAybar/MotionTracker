

#include "eigen_ble.h"
#include "custom_app.h"
#include "app_ble.h"
#include "main.h"
#include "ble.h"
#include "custom_stm.h"

//##################
// IMPORTANT: Some of these functions are not allowed to be called in an interrupt! Otherwise, they will never finish
//##################

extern uint16_t bleConnectionHandle;
extern Device_Data_t deviceData;;
extern RTC_TimeTypeDef rtcTime;
extern RTC_HandleTypeDef hrtc;
extern uint8_t waitForRessourcesToNotify;

void Ble_Connected_To_Client(void)
{
	printf("eigen_ble.c Ble_Connected_To_Client\r\n");
	// this function is used to handle the connection event in the ST library
	deviceData.bluetoothStatus = connected; // automatically stops advertising after connect
}

void Ble_Disconnected_From_Client(void)
{
	printf("eigen_ble.c Ble_Disconnected_From_Client\r\n");
	// this function is used to handle the disconnection event in the ST library
	deviceData.bluetoothStatus = advertisingFast; // automatically starts fast advertising after disconnect
	Ble_Update_Advertising_Data(); // ist notwendig, damit die eigene Advertising-Data verwendet wird
	// sonst würde die im Cube generierte Advertising-Data angezeigt werden

}

void Ble_Disconnect(void)
{
	printf("eigen_ble.c Ble_Disconnect\r\n");
	uint8_t ret = 0;
	if(deviceData.bluetoothStatus == connected)
	{
		printf("actually disconnecting\r\n");
		ret = aci_gap_terminate( bleConnectionHandle, REMOTE_USER_TERMINATED_CONNECTION);
		if(ret == 0){
			printf("disconnection successful\r\n");
			deviceData.bluetoothStatus = advertisingFast; // automatically starts after disconnect
		} else {
			printf("disconnection NOT successful\r\n");
			printf("return value is %i\r\n", ret);
		}
	} else {
		printf("not disconnecting\r\n");
	}
}

void Ble_Start_Advertising(uint8_t typeOfAdvertising){
	printf("eigen_ble.c Ble_Start_Advertising\r\n");

	// this function can also be used to restart advertising
	uint8_t ret = 0;
	if(deviceData.bluetoothStatus != connected) {
		if(deviceData.bluetoothStatus == advertisingFast || deviceData.bluetoothStatus == advertisingSlow)
		{
			Ble_Stop_Advertising(); // stop before restart
		}

		if(typeOfAdvertising==SLOW_ADVERTISING)
		{
			ret = aci_gap_set_discoverable(ADV_TYPE,
					CFG_LP_CONN_ADV_INTERVAL_MIN,
					CFG_LP_CONN_ADV_INTERVAL_MAX,
											 CFG_BLE_ADDRESS_TYPE,
											 ADV_FILTER,
											 0,
											 0,
											 0,
											 0,
											 0,
											 0);

			if (ret != BLE_STATUS_SUCCESS)
			{
				printf("error starting slow advertising\r\n");
				printf("return value is %i\r\n", ret);

			} else
			{
				printf("slow advertising started\r\n");
				deviceData.bluetoothStatus = advertisingSlow;
			}
		} else
		{
			ret = aci_gap_set_discoverable(ADV_TYPE,
					CFG_FAST_CONN_ADV_INTERVAL_MIN,
					CFG_FAST_CONN_ADV_INTERVAL_MAX,
											 CFG_BLE_ADDRESS_TYPE,
											 ADV_FILTER,
											 0,
											 0,
											 0,
											 0,
											 0,
											 0);
			if (ret != BLE_STATUS_SUCCESS)
			{
				printf("error starting fast advertising\r\n");
				printf("return value is %i\r\n", ret);
			} else
			{
				printf("fast advertising started\r\n");
				deviceData.bluetoothStatus = advertisingFast;
			}
		}
		Ble_Update_Advertising_Data(); // nachdem das Advertising gestartet wurde, muss die advertising Data beschrieben werden
												// sonst haben die Daten den Default-Value
	}
}

void Ble_Stop_Advertising(void) {
	uint8_t ret = 0;

	printf("eigen_ble.c stop advertising\r\n");
	if(deviceData.bluetoothStatus == advertisingFast || deviceData.bluetoothStatus == advertisingSlow)
	{
		printf("actually stopping advertising\r\n");
		ret = aci_gap_set_non_discoverable();

		if (ret != BLE_STATUS_SUCCESS)
		{
			printf("error stopping advertising\r\n");
			printf("return value is %i\r\n", ret);
		} else
		{
			printf("advertising stopped\r\n");
			deviceData.bluetoothStatus = notConnectedNotAdvertising;
		}
	} else {
		printf("not stopping advertising\r\n");
	}
}

void Ble_Update_Advertising_Data()
{
	// DIESE FUNKTION DARF NICHT IN EINEM INTERRUPT AUFGERUFEN WERDEN!

	printf("eigen_ble.c Ble_Update_Advertising_Data\r\n");

	uint8_t ret = 0;
	// Advertising Data darf 25 Byte nicht überschreiten
	// Wenn eine komplette 128 Bit UUID verwendet werden soll, nimmt sie bereits 18 von 25 Stellen der Advertising Data ein
	// Dadurch bleibt zu wenig Speicher, um weitere Daten anzuhängen
	// Um die eigenen Geräte von denen anderer Hersteller zu unterscheiden, wir eine 16 Bit UUID verwendet
	// Es wird "2222" verwendet, da diese in der Liste der 16 Bit UUIDS (von Bluetooth) nicht vorkam
	// Die UUID wird automatisch der Gesamt-UUID angehängt: 00002222-0000-1000-8000-00805F9B34FB
	// es kann vorkommen, dass auch andere Hersteller diese UUID verwenden, allerdings ist das eher unwahrscheinlich
	// und die Wahl einer UUID ermöglicht die Verwendung eines Filters in Flutter

	if(deviceData.bluetoothStatus == advertisingFast || deviceData.bluetoothStatus == advertisingSlow)
	{
		printf("actually updating advertising data\r\n");

//		uint8_t ad_new_data[24] = {
//		7, AD_TYPE_COMPLETE_LOCAL_NAME, 'M', 'o', 't', 'i', 'o', 'n',  /* Complete name nach "Norm" */
//		3, AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST, 0x18, 0x40,
//		10, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x00, 0x00, 0,0,0,0,0,0,
//		// [16] Batteriespannung
//
//		};

		uint8_t ad_new_data[24] =
		{
		  7, AD_TYPE_COMPLETE_LOCAL_NAME, 'M', 'o', 't', 'i', 'o', 'n',  /* Complete name */
		  3, AD_TYPE_APPEARANCE, 0x00, 0xC0 /* GENERIC_WATCH_APPEARANCE */,
		  2, AD_TYPE_LE_ROLE, 0x00 /* Only Peripheral Role supported */,
		  3, AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST, 0x3E, 0x18,
		  4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x30, 0x00, 0x00 /* battery voltage */,
		};

		ad_new_data[23]=deviceData.batteryVoltage;

		ret=aci_gap_update_adv_data(sizeof(ad_new_data), (uint8_t*) ad_new_data);

		if (ret != BLE_STATUS_SUCCESS)
		{
			printf("error updating advertising data\r\n");
			printf("return value is %i\r\n", ret);
		} else
		{
			printf("success updating advertising data\r\n");
		}
	} else {
		printf("did not update advertising data\r\n");
	}
}

uint8_t Ble_Update_Characteristic(uint8_t characteristic, uint8_t *data, uint8_t numberOfBytesToTransmit)
{
	uint8_t ret = 0;

	ret = Custom_STM_App_Update_Char(characteristic, (uint8_t *)data); // update value for read / notify

	if (ret != BLE_STATUS_SUCCESS)
	{
		waitForRessourcesToNotify=1;
//		HAL_GPIO_WritePin(TESTPIN2_GPIO_Port, TESTPIN2_Pin, 1);
		printf("error updating characteristic\r\n");
		printf("return value is %i\r\n", ret);
//		HAL_GPIO_WritePin(TESTPIN2_GPIO_Port, TESTPIN2_Pin, 0);
	} else
	{
//		printf("success updating characteristic\r\n");
	}
//	printf("end char update\r\n");

	return ret;
}
