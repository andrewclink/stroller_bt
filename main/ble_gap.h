#ifndef ble_gap_h
#define ble_gap_h

#include "esp_gap_ble_api.h"

#include "defs.h"


// Advertising parameters for starting advertising from another file
extern esp_ble_adv_params_t adv_params;

// Advertising data; should probably be refactored somehow.
extern esp_ble_adv_data_t adv_data;

// Starts the process of configuring BT stack advertising data
// so that advertising can begin in the callback
//
void gap_start_advertising(void);


// For registering as a callback.
// We should probably refactor this.
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);


// Update the connection params for the remote peer specifically
void ble_gap_update_connection_params(esp_bd_addr_t * remote_addr);
#endif
