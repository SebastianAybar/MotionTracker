

#include "app_ble.h"
#include "custom_app.h"
#include "app_ble.h"
#include "main.h"
#include "ble.h"



  void Ble_Update_Advertising_Data(void);

  void Ble_Connected_To_Client(void);

  void Ble_Disconnect(void);
  void Ble_Disconnected_From_Client(void);

  void Ble_Start_Advertising(uint8_t typeOfAdvertising);
  void Ble_Stop_Advertising(void);
  uint8_t Ble_Update_Characteristic(uint8_t characteristic, uint8_t *data, uint8_t bytesToTransmit);
