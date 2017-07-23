#ifndef ble_h
#define ble_h

#include "esp_gatts_api.h"

#include "defs.h"

enum {
  ble_index_service,
  
  ble_index_steerpos_char,
  ble_index_steerpos_val,
  
  ble_index_max  
};


// Characteristic Type
//
typedef struct ble_gatt_char_s {
  uint16_t handle;
  esp_bt_uuid_t uuid;
  
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t properties;
  
  esp_bt_uuid_t descr_uuid; // Client Characteristic Configuration Descriptor (added to the characteristic)

  void * write_buffer; // TODO
  uint16_t write_pos;  // TODO

  struct ble_gatt_char_s * next;
} ble_gatt_char_t;



// Characteristic read/write/write_exec callback
//
struct ble_gatt_service_s;
typedef esp_gatt_status_t (*ble_gatt_char_cb_t)(struct ble_gatt_service_s *svc, ble_gatt_char_t * characteristic, void * buf, uint16_t *len);


// Service entry type
//
typedef struct ble_gatt_service_s {
  ble_gatt_char_cb_t didReadCB;  // Assigned by us to handle read/write to characteristics
  ble_gatt_char_cb_t didWriteCB; // Assigned by us to handle read/write to characteristics
  uint8_t  ble_service_id;       // Assigned by us in the order registered. Maybe should be called app_id (but that exists too?)
  uint16_t gatts_if;
  uint16_t app_id; // Is this used?
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t descr_handle;
  
  ble_gatt_char_t * characteristics;
  
  struct ble_gatt_service_s * next;
} ble_gatt_service_t;




// Call to enable all services and characteristics that were created
//
void ble_gatt_start(void); 

// Create a service entry and register it for activation upon calling +ble_gatt_start+
//
ble_gatt_service_t * ble_gatt_service_create(const uint8_t *uuid, int16_t uuid_len, ble_gatt_char_cb_t didReadCB, ble_gatt_char_cb_t didWriteCB);

// Create a characteristic on a service
ble_gatt_char_t * ble_gatt_characteristic_create(ble_gatt_service_t * svc, const void *uuid, int16_t uuid_len);
ble_gatt_char_t * ble_gatt_get_char_by_handle(ble_gatt_service_t * svc, uint16_t handle);

// Central GATT Event handler. This will dispatch to different service callbacks based on which gatts_if is passed in.
//
void gatt_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#endif

