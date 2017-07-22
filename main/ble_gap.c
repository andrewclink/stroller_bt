#include <stdio.h>
#include <string.h>
#include "esp_system.h"

#include "bt.h"
#include "bta_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "ble_gap.h"

#define LOGT "GAP "

esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20, // interval min: x*0.625 == (hex) * 1; 0x20 * 0.625 = 20ms
    .adv_int_max        = 0x40, // interval max; 0xA0=100ms
    .adv_type           = ADV_TYPE_IND, // advertising type
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC, // owner bluetooth device address type
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // advertising filter policy 
};


// TODO: determine where this is used and how we should configure
#if 0
static uint8_t test_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0xAB, 0xCD,
};
#endif

extern const uint8_t stroller_service_uuid[16];

uint8_t test_manufacturer_data[4] = {0x11, 0x22, 0x33, 0x44};

esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],

    .service_data_len = 0,
    .p_service_data = NULL,

    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t*)stroller_service_uuid,

    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), //no enhanced data rate
    
};


// Configures advertising data in the BT stack
void gap_start_advertising(void)
{
  esp_err_t result;
  
  printf("Set Device Name\n");
  esp_ble_gap_set_device_name("EStroller");
  
  result = esp_ble_gap_config_adv_data(&adv_data);
  if (ESP_OK != result) { ESP_LOGE(LOGT, "Could not config adv data: %d", result); }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) 
    {
      // Advertising data configuration is done; ready to advertise
      case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(LOGT, "DATA_SET_COMPLETE_EVT\n");

        ESP_LOGI(LOGT, "Start advertising")
        esp_err_t result;

        result = esp_ble_gap_start_advertising(&adv_params);
        if (ESP_OK != result) { ESP_LOGE(LOGT, "Could not start adv: %d", result); }

        break;
      
      case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
      case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGE(LOGT, "RAW Callback received\n");
        break;
      
      case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        ESP_LOGI(LOGT, "ESP_GAP_BLE_ADV_START_COMPLETE_EVT\n");
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(LOGT, "Advertising start failed\n");
        }
        break;

      case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(LOGT, "Advertising stop failed\n");
        }
        else {
            ESP_LOGI(LOGT, "Stop adv successfully\n");
        }
        break;
    
      case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(LOGT, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        ESP_LOGW(LOGT, "Unknown GAP Event %d", event);
        break;
    }
}

void ble_gap_update_connection_params(esp_bd_addr_t * remote_addr)
{
  // Start sent the update connection parameters to the peer device.
  // For iOS, please reference the apple official documents 
  // about the ble connection parameters restrictions
  //
  esp_ble_conn_update_params_t conn_params = {0};
  memcpy(conn_params.bda, remote_addr, sizeof(esp_bd_addr_t));
  conn_params.latency = 0;
  conn_params.max_int = 0x50;    // max_int = 0x50*1.25ms = 100ms
  conn_params.min_int = 0x30;    // min_int = 0x30*1.25ms = 60ms
  conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms
  esp_ble_gap_update_conn_params(&conn_params);
}

