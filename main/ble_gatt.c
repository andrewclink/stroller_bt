#include <stdio.h>
#include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "esp_system.h"

#include "bt.h"
#include "bta_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "ble_gatt.h"
#include "ble_gap.h"

#define LOGT "GATT"

// Configuration
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE; // ?
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE; //?
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY; // ?

ble_gatt_connectionEventHandler connectionEventHandler = NULL;


// TODO: Should this go in the GAP? TODO: Should what?

uint8_t value_holder = 0;

// #pragma mark -
// #pragma mark Service/Characteristic Creation

static ble_gatt_service_t * service_head = NULL;
static uint16_t service_count = 0;

static void uuid_set(esp_bt_uuid_t * dest, const void * src, int16_t uuid_len)
{
  dest->len = uuid_len;
  
  switch(uuid_len)
  {
    case ESP_UUID_LEN_16: dest->uuid.uuid16 = *(uint16_t*)src; break;
    case ESP_UUID_LEN_32: dest->uuid.uuid32 = *(uint32_t*)src; break;
    case ESP_UUID_LEN_128:
      memcpy(dest->uuid.uuid128, (uint8_t*)src, ESP_UUID_LEN_128);
      break;
    default:
      ESP_LOGE(LOGT, "invalid UUID len %d\n", uuid_len);
  }
}

ble_gatt_service_t * ble_gatt_service_create(const uint8_t *uuid, int16_t uuid_len, ble_gatt_char_cb_t didReadCB, ble_gatt_char_cb_t didWriteCB)
{
  ble_gatt_service_t * svc = malloc(sizeof(ble_gatt_service_t));
  memset(svc, '\0', sizeof(ble_gatt_service_t));
  
  // Find service tail
  ble_gatt_service_t * tail = service_head;
  if (NULL == tail)
  {
    printf("Assigning service_head %p\n", svc);
    service_head = svc;
  }
  else
  {
    while(tail->next != NULL)
      tail = tail->next;

    printf("Pointing service tail to %p\n", svc);
    tail->next = svc;
  }
  
  // Members
  svc->didReadCB      = didReadCB;
  svc->didWriteCB     = didWriteCB;
  svc->ble_service_id = service_count++;
  
  // From example; handle now though.
  // I wonder why unions of structs of structs aren't just wildly popular.
  //
  svc->service_id.is_primary = true;
  svc->service_id.id.inst_id = 0x00;
  uuid_set(&svc->service_id.id.uuid, uuid, uuid_len);
  
  return svc;
}

ble_gatt_service_t * ble_gatt_get_service_by_id(uint16_t app_id)
{
  ble_gatt_service_t * svc = service_head;

  // Count up to ID
  for(int i=0; i < app_id; i++)
  {
    if (NULL == svc)
      break;

    svc = svc->next;
  }

  if (NULL == svc)
  {
    ESP_LOGW(LOGT, "Warning, returning NULL for service id %u\n", app_id);
  }
  
  return svc;
}

ble_gatt_service_t * ble_gatt_get_service_by_if(uint16_t gatt_if)
{
  ble_gatt_service_t * svc = service_head;

  while(svc != NULL)
  {
    if (svc->gatts_if == gatt_if)
      break;
    
    svc = svc->next;
  }

  if (NULL == svc)
  {
    ESP_LOGW(LOGT, "Warning, returning NULL for gatt_if %u\n", gatt_if);
  }
  
  return svc;
}

ble_gatt_char_t * ble_gatt_characteristic_create(ble_gatt_service_t * svc, const void *uuid, int16_t uuid_len)
{
  ble_gatt_char_t * cha = malloc(sizeof(ble_gatt_char_t));
  memset(cha, '\0', sizeof(ble_gatt_char_t));
  
  // Find service's characteristic tail
  ble_gatt_char_t * tail = svc->characteristics;
  if (NULL == tail)
  {
    printf("Adding characteristic %p to service %p head\n", cha, svc);
    svc->characteristics = cha;
  }
  else
  {
    while(tail->next != NULL)
      tail = tail->next;

    printf("Adding characteristic %p to service %p tail\n", cha, svc);
    
    tail->next = cha;
  }
    
  uuid_set(&cha->uuid, uuid, uuid_len);
  cha->perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
  cha->properties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

  return cha;
}

ble_gatt_char_t * ble_gatt_get_char_by_uuid(ble_gatt_service_t * svc, esp_bt_uuid_t * uuid)
{
  printf("Finding char for service %p with uuid len %u\n", svc, uuid->len);
  switch(uuid->len)
  {
    case ESP_UUID_LEN_16:  printf("find  16 %04x\n", uuid->uuid.uuid16); break;
    case ESP_UUID_LEN_32:  printf("find  32 %08x\n", uuid->uuid.uuid32); break;
    case ESP_UUID_LEN_128: printf("find 128 %08x\n", uuid->uuid.uuid32); break;
    default: ESP_LOGE(LOGT, "uuid len %u\n", uuid->len);
  }
  
  // Search all services if it wasn't provided
  if (NULL == svc)
    svc = service_head;

  bool last_service = (svc != NULL);
  
  while (NULL != svc)
  {
    ble_gatt_char_t * cha = svc->characteristics;
//    printf("Searching service %p (char head %p)\n", svc, svc->characteristics);
    while (NULL != cha)
    {
      switch(cha->uuid.len)
      {
        case ESP_UUID_LEN_16:  printf("char  16 %04x\n", cha->uuid.uuid.uuid16); break;
        case ESP_UUID_LEN_32:  printf("char  32 %08x\n", cha->uuid.uuid.uuid32); break;
        case ESP_UUID_LEN_128: printf("char 128 %08x\n", cha->uuid.uuid.uuid32); break;
        default: ESP_LOGE(LOGT, "uuid len %u\n", cha->uuid.len);
      }
      
      if (cha->uuid.len == uuid->len)
      {          
        switch(uuid->len)
        {
          case ESP_UUID_LEN_16:
            printf("  %04x == %04x\n", cha->uuid.uuid.uuid16, uuid->uuid.uuid16);
            if (cha->uuid.uuid.uuid16 == uuid->uuid.uuid16) return cha;
            break;
          case ESP_UUID_LEN_32:
            printf("  %08x == %08x\n", cha->uuid.uuid.uuid32, uuid->uuid.uuid32);
            if (cha->uuid.uuid.uuid32 == uuid->uuid.uuid32) return cha;
            break;
          case ESP_UUID_LEN_128:
            printf("  %08x == %08x (128)\n", cha->uuid.uuid.uuid32, uuid->uuid.uuid32);
            if (!memcmp(&cha->uuid.uuid.uuid128, &uuid->uuid.uuid128, ESP_UUID_LEN_128)) return cha;
            break;

          default:
            ESP_LOGE(LOGT, "invalid uuidlen\n");
            return NULL;
        }
      }
      
      cha = cha->next;
    }
    
    // If the service to search was provided, bound the search within that service.
    if (last_service)
      return NULL;

    svc = svc->next;
  }
  
  return NULL;
}

ble_gatt_char_t * ble_gatt_get_char_by_handle(ble_gatt_service_t * svc, uint16_t handle)
{
  // Search all services if it wasn't provided
  if (NULL == svc)
    svc = service_head;

  bool last_service = (svc != NULL);
  
  while (NULL != svc)
  {
    ble_gatt_char_t * cha = svc->characteristics;
    // printf("Searching service %p (char head %p)\n", svc, svc->characteristics);
    while (NULL != cha)
    {
      if (cha->handle == handle)
        return cha;
      
      cha = cha->next;
    }
    
    // If the service to search was provided, bound the search within that service.
    if (last_service)
      return NULL;

    svc = svc->next;
  }
  
  return NULL;
}



void ble_gatt_start(ble_gatt_connectionEventHandler eventHandler)
{
  
  // Store pointer to connection event handler
  connectionEventHandler = eventHandler;
  
  // Register this layer with the esp api (oh yes, there are layers here.)
  esp_ble_gatts_register_callback(gatt_event_handler);
  
  ble_gatt_service_t * svc = service_head;
  while(svc != NULL)
  {
    printf("Registering ble_service_id %d\n", svc->ble_service_id);
    esp_ble_gatts_app_register(svc->ble_service_id);
    svc = svc->next;
  }
  
  gap_start_advertising();
}


void gatt_register_characteristic(ble_gatt_service_t * svc, ble_gatt_char_t * cha)
{
  if (NULL == cha)
    return;
  
  printf("Adding Characteristic %08x\n", cha->uuid.uuid.uuid32);

/* this is obviously on the stack; it won't hang around.
  esp_attr_value_t value_attr = {
    .attr_max_len = 1,                      //  attribute max value length    
    .attr_len = 1,                          //  attribute current value length
    .attr_value = &value_holder                      // cannot be null
  }; 
*/
  
  esp_err_t ret = esp_ble_gatts_add_char(
    svc->service_handle,
    &cha->uuid,
    cha->perm,
    cha->properties, //
    NULL, //&value_attr, // value (initial?) (can be NULL)
    NULL  // attribute response control byte
  );
    
  if (ret == ESP_OK)
  {
    ESP_LOGI(LOGT, "esp_ble_gatts_add_char: %d\n", ret);
  }
  else
  {
    ESP_LOGE(LOGT, "esp_ble_gatts_add_char: %d\n", ret);
  }
}

// Central GATT Event handler. This will dispatch to different service callbacks based on which gatts_if is passed in.
//
void gatt_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  // On reg event we are being told to store the gatts_if.
  // On other events, if there is a relavent service we will be told
  // through the gatts_if
  //
  // Find the service
  //
  ble_gatt_service_t * svc = NULL;
  
  if (event == ESP_GATTS_REG_EVT) 
  {
    if (param->reg.status != ESP_GATT_OK) 
    {
      ESP_LOGI(LOGT, "Reg app failed, app_id %04x, status %d\n",
              param->reg.app_id, 
              param->reg.status);
      return;
    }

    svc = ble_gatt_get_service_by_id(param->reg.app_id);
  }
  else
  {
    svc = ble_gatt_get_service_by_if(gatts_if);
  }
    

    
  
  
  // Handle the event
  // If gatts_if == ESP_GATT_IF_NONE then the event applies to all "applications" (services)? 
  //
  switch (event) 
  {
    case ESP_GATTS_REG_EVT:
      ESP_LOGI(LOGT, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
      
      if (gatts_if == ESP_GATT_IF_NONE)
      {
        ESP_LOGW(LOGT "GATT", "-> gatts_if = NONE\n");
        ESP_LOGW(LOGT "GATT", "-> We will need to handle this\n");
        break;
      }

      // esp_ble_gap_config_adv_data()

      // Remember the assigned if
      svc->gatts_if = gatts_if;
      
      // Create service
      // This will create and then fire the callback with the event BTA_GATTS_CREATE_SRVC_EVT
      // TODO: number of handles seems to need to be characteristic_count * 4
      esp_ble_gatts_create_service(gatts_if, 
        &svc->service_id,         // Why is this a pointer?
        8); // Number of handles requested?

      break;

    case ESP_GATTS_CREATE_EVT:
    {
      ESP_LOGI(LOGT, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
      svc->service_handle = param->create.service_handle;
      
      //TODO: Should this be done after we add the characteristics?
      esp_err_t ret = esp_ble_gatts_start_service(svc->service_handle);
      if (ret == ESP_OK)
      {
        ESP_LOGI(LOGT, "esp_ble_gatts_start_service: %d\n", ret);
      }
      else
      {
        ESP_LOGE(LOGT, "esp_ble_gatts_start_service: %d\n", ret);
      }
      
      // Add the first characteristic to the service.
      // Each will be added when the appropriate callback is received.
      // This allows us to tolerate Espressif's poorly thought out API
      // with regaurd to characteristic descriptions.
      //
      ble_gatt_char_t *cha = svc->characteristics;
      gatt_register_characteristic(svc, cha);
      break;
    }
    
    case ESP_GATTS_ADD_CHAR_EVT:
    {
      ESP_LOGI(LOGT, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
               param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
               
      // Lookup by characteristic UUID
      //
      ble_gatt_char_t * cha = ble_gatt_get_char_by_uuid(svc, &param->add_char.char_uuid);
      // ESP_LOGI(LOGT, "got characteristic %p\n", cha);
      
      // Record info on the added characteristics
      //
      // Record attr_handle
      cha->handle = param->add_char.attr_handle;
      
      
      // Record and apply the Client Characteristic Configuration Descriptor
      // so that the connected device can tell us that it wants notifications (or indications).
      //
      cha->descr_uuid.len = ESP_UUID_LEN_16;
      cha->descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

      // Adding a char description adds to the last characteristic added, which is stupid.
      // If we're adding a descriptor to this cha, we need to do it now. Otherwise
      // start adding the next characteristic.
      //
      if (false)
      {
        esp_ble_gatts_add_char_descr(svc->service_handle, &cha->descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // descriptor access permission
                                     NULL, // char descriptor value
                                     NULL  // attribute response control byte
                                    );
      }
      else
      {
        gatt_register_characteristic(svc, cha->next);
      }
      break;
    }
        

    // Received after a characteristic descriptor was added
    // We need to use this oportunity to either add more characteristic
    // descriptors or to add the next characteristic
    //
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    {
      ESP_LOGI(LOGT, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
               param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);

     
      //TODO! The uuid passed here is not, as the docs say, the characteristic UUID, but rather the descriptor UUID!
               
     
     
      ble_gatt_char_t * cha = ble_gatt_get_char_by_uuid(svc, &param->add_char_descr.char_uuid);
      if (NULL == cha)
      {
        ESP_LOGW(LOGT, "Could not find cha for UUID %p", &param->add_char_descr.char_uuid);
        return;
      }
      
      
      if (false /*more descriptors to add */)
      {
        /* e.g. */
        esp_ble_gatts_add_char_descr(svc->service_handle, &cha->descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // descriptor access permission
                                     NULL, // char descriptor value
                                     NULL  // attribute response control byte
                                    );
      }
      else
      {
        gatt_register_characteristic(svc, cha->next);
      }

      break;
    }
      
          
    case ESP_GATTS_READ_EVT: 
    {
      ESP_LOGI(LOGT, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);

      // esp_gatt_rsp allocates 600 bytes.
      // They also stuck the fixed-size buffer on the front of the struct 
      // and copy it so many times I can't even cast a trimmed down buffer.
      //
      // TODO: Clean up the espressif sources! They're terrible!
      // TODO: Add length field to response
      // TODO: Modify  everything in btc_gatts.c dealing with the arg for BTC_GATTS_ACT_SEND_RESPONSE
      // TODO: Modify  btc_gatts_arg_deep_copy
      // TODO: Modify  btc_gatts_arg_deep_free, etc
      // 


      ble_gatt_char_t * cha = ble_gatt_get_char_by_handle(svc, param->read.handle);
      
      int len = 0;

      esp_gatt_rsp_t rsp; // 600+ bytes to be memcpy'd several times
      memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
      rsp.attr_value.handle = param->read.handle;

      esp_gatt_status_t status = svc->didReadCB(svc, cha, &rsp.attr_value.value, &rsp.attr_value.len);
      
      esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, status, &rsp);

      break;
    }
  
    case ESP_GATTS_WRITE_EVT: 
    {
      // ESP_LOGI(LOGT, "GATT_WRITE_EVT conn_id %d; trans_id %d; handle %d; need rsp %d; prep %d",
      //           param->write.conn_id, param->write.trans_id, param->write.handle, param->write.need_rsp, param->write.is_prep);
      // ESP_LOGI(LOGT, "                value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
      
      // Call the registered callback
      // Get the relavent characteristic
      ble_gatt_char_t * cha = ble_gatt_get_char_by_handle(svc, param->write.handle);
      
      uint16_t len;
      esp_gatt_status_t status;
      
      if (NULL == cha)
      {
        ESP_LOGE(LOGT, "NULL cha in write CB");
        status = ESP_GATT_NOT_FOUND;
      }
      else
      {
        // len in:  length of written paylod
        // len out: length written
        // TODO: Calculate offset of value based on location of pointer out?
        //
        len = param->write.len;
        status = svc->didWriteCB(svc, cha, param->write.value, &len);
      }

        
      
      // Send response
      // This annoyingly allocates a 600 byte payload.
      if (param->write.need_rsp)
      {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        gatt_rsp->attr_value.len = len;
        gatt_rsp->attr_value.handle = param->write.handle;
        gatt_rsp->attr_value.offset = param->write.offset;
        gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;

        memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);

        esp_err_t ret = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);

        if (ret != ESP_OK)
           ESP_LOGE(PROJ, "Write event: response error %d", ret);

        free(gatt_rsp);
      }
      
      break;
    }
  
  
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
      // TODO: dispatch read/write callback properly
      ESP_LOGW(LOGT,"ESP_GATTS_EXEC_WRITE_EVT");
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
      //example_exec_write_event_env(&b_prepare_write_env, param);
      break;
    }
      






    case ESP_GATTS_START_EVT:
      ESP_LOGI(LOGT, "SERVICE_START_EVT, status %d, service_handle %d\n",
               param->start.status, param->start.service_handle);
      break;
      
    case ESP_GATTS_CONNECT_EVT:
      ESP_LOGI(LOGT, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_preped %d\n",
               param->connect.conn_id,
               param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
               param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
               param->connect.is_connected);

      svc->conn_id = param->connect.conn_id;

      // Inform the higher layer that we're connected.
      // This could probably pass some info about who connected, etc,
      // but right now I don't care.
      //
      connectionEventHandler(true);

      // Start sent the update connection parameters to the peer device.
      // For the iOS system, please reference the apple official documents 
      // about the BLE connection parameters restrictions
      //
      ble_gap_update_connection_params(&param->connect.remote_bda);

      break;

    case ESP_GATTS_DISCONNECT_EVT:
      esp_ble_gap_start_advertising(&adv_params);
      break;
      

    case ESP_GATTS_MTU_EVT: printf("ESP_GATTS_MTU_EVT\n");break;
    case ESP_GATTS_CONF_EVT: printf("ESP_GATTS_CONF_EVT\n");break;
    case ESP_GATTS_UNREG_EVT: printf("ESP_GATTS_UNREG_EVT\n"); break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT: printf("ESP_GATTS_ADD_INCL_SRVC_EVT\n"); break;
    case ESP_GATTS_DELETE_EVT:  printf("ESP_GATTS_DELETE_EVT\n"); break;
    case ESP_GATTS_STOP_EVT: printf("ESP_GATTS_STOP_EVT\n"); break;
    case ESP_GATTS_OPEN_EVT: printf("ESP_GATTS_OPEN_EVT\n"); break;
    case ESP_GATTS_CANCEL_OPEN_EVT: printf("ESP_GATTS_CANCEL_OPEN_EVT\n"); break;
    case ESP_GATTS_CLOSE_EVT: printf("ESP_GATTS_CLOSE_EVT\n"); break;
    case ESP_GATTS_LISTEN_EVT: printf("ESP_GATTS_LISTEN_EVT\n"); break;
    case ESP_GATTS_CONGEST_EVT: printf("ESP_GATTS_CONGEST_EVT\n"); break;
    case ESP_GATTS_RESPONSE_EVT: printf("ESP_GATTS_RESPONSE_EVT\n"); break;
    
    default:
      ESP_LOGW(LOGT, "Unhandled event: %d", event)
      break;
  }
}
 