#include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "esp_system.h"

#include "bt.h"
#include "bta_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "ble.h"


// Service table
// struct esp_gatts_attr_db_t {
//   esp_attr_control_t attr_control = { uint8_t auto_rsp }
//   esp_attr_desc_t    att_desc = {
//     uint16_t uuid_len
//     uint8_t * uuid_p
//     uint16_t perm (permission)
//     uint16_t max_length (of the element)
//     uint16_t length (current of the element)
//     uint8_t * val (element value array)
//   }
// }
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE; // ?
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE; //?
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY; // ?
static const uint8_t stroller_service_uuid[32] = {0xF9, 0xD4, 0x24, 0xD3, 
                                                  0xBD, 0xD6, 0x4F, 0x86, 
                                                  0xAB, 0x67, 0x8D, 0xB2, 
                                                  0x07, 0x69, 0x21, 0xC8};  //F9D424D3-BDD6-4F86-AB67-8DB2076921C8
static const esp_gatts_attr_db_t heart_rate_gatt_db[ble_index_max] = 
{
  // Service Declaration
  // I don't understand the contents of this entry
  [ble_index_service] = {
    {ESP_GATT_AUTO_RSP},
    {
      ESP_UUID_LEN_16, // uuid_len
      (uint8_t*)&primary_service_uuid, // 0x2800 (???)
      ESP_GATT_PERM_READ, // permissions
      sizeof(uint8_t),
      sizeof(stroller_service_uuid),
      stroller_service_uuid
    }
  },
  
  // Steering Position Characteristic
  [ble_index_steerpos_char] = {
    {ESP_GATT_AUTO_RSP},
    {
      ESP_UUID_LEN_16,
      (uint8_t *)&character_declaration_uuid, 
      ESP_GATT_PERM_READ,
      sizeof(uint8_t),
      sizeof(uint8_t), 
      (uint8_t *)&char_prop_notify
    }
  }
  
};
