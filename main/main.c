/* 
  Stroller controller, July 2017
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "esp_log.h"

#include "bt.h"
#include "bta_api.h"
#include "esp_bt_main.h" // bluedroid_init


#include "ble_gap.h"
#include "ble_gatt.h"

#define INF(...) ESP_LOGI("main", __VA_ARGS__)


#if 0
// This doesn't work because of a bluetooth stack bug.
// 128 bit UUIDs screw up the advertising packet
//
const uint8_t stroller_service_uuid[16] = {0xF9, 0xD4, 0x24, 0xD3, 
                                                  0xBD, 0xD6, 0x4F, 0x86, 
                                                  0xAB, 0x67, 0x8D, 0xB2, 
                                                  0x07, 0x69, 0x21, 0xC8};  //C8216907-B28D-67AB-864F-D6BDD324D4F9
#else

// C8100269700000000010000080000000000805093400
const uint8_t stroller_service_uuid[16] = {
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,  // Base UUID
  0x07, 0x69, 0x21, 0xC8                                                   // 32-bit used section
};
#endif

static const uint16_t speed_uuid = 0x1419; // Speed
static const uint16_t sa_uuid    = 0x1428; // Steering Angle
static const uint16_t sh_uuid    = 0x1429; // Steering Home Control Point



esp_gatt_status_t stroller_cb(esp_gatts_cb_event_t event, ble_gatt_char_t * characteristic, esp_ble_gatts_cb_param_t *param)
{
  // Handle write payload
  return ESP_GATT_OK;
}


void ble_init(void)
{
  // Configure BLE
  esp_err_t ret;

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) 
  {
      ESP_LOGE(PROJ, "%s initialize controller failed\n", __func__);
      return;
  }


  ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
  if (ret) 
  {
      ESP_LOGE(PROJ, "%s enable controller failed\n", __func__);
      return;
  }
  
  
  ret = esp_bluedroid_init();
  if (ret) 
  {
      ESP_LOGE(PROJ, "%s init bluetooth failed\n", __func__);
      return;
  }
  ret = esp_bluedroid_enable();
  if (ret) 
  {
      ESP_LOGE(PROJ, "%s enable bluetooth failed\n", __func__);
      return;
  }
  

  ESP_LOGI(PROJ, "Config Adv")
  esp_ble_gap_config_adv_data(&adv_data);

  esp_ble_gatts_register_callback(gatt_event_handler);
  esp_ble_gap_register_callback(gap_event_handler);  
}

void app_main()
{
    printf("Stroller\n");


    ble_init();
      
    // Create the stroller service & characteristics
    ble_gatt_service_t * stroller_service = ble_gatt_service_create(stroller_service_uuid, ESP_UUID_LEN_128, stroller_cb);

    ble_gatt_char_t * speed = ble_gatt_characteristic_create(stroller_service, &speed_uuid, ESP_UUID_LEN_16);
    speed->properties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

    ble_gatt_char_t * sa = ble_gatt_characteristic_create(stroller_service, &sa_uuid, ESP_UUID_LEN_16);
    sa->properties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
    
    ble_gatt_char_t * steering_home = ble_gatt_characteristic_create(stroller_service, &sh_uuid, ESP_UUID_LEN_16);
    steering_home->perm       = ESP_GATT_PERM_WRITE;
    steering_home->properties = ESP_GATT_CHAR_PROP_BIT_WRITE;
    
    // Start BLE
    ble_gatt_start();
    
    return;
}
