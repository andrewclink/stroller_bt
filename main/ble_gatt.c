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
static const uint8_t stroller_service_uuid[32] = {0xF9, 0xD4, 0x24, 0xD3, 
                                                  0xBD, 0xD6, 0x4F, 0x86, 
                                                  0xAB, 0x67, 0x8D, 0xB2, 
                                                  0x07, 0x69, 0x21, 0xC8};  //F9D424D3-BDD6-4F86-AB67-8DB2076921C8
// TODO: Should this go in the GAP?

static int __attribute__((unused)) reset_tab=0;


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

ble_gatt_service_t * ble_gatt_service_create(const uint8_t *uuid, int16_t uuid_len, esp_gatts_cb_t cb)
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
  
  
  svc->callback = cb;
  svc->ble_service_id = service_count++;
  
  // From example; handle now though.
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
    case ESP_UUID_LEN_128: printf("find 132 %08x\n", uuid->uuid.uuid32); break;
    default: ESP_LOGE(LOGT, "uuid len %u\n", uuid->len);
  }
  
  // Search all services if it wasn't provided
  if (NULL == svc)
    svc = service_head;

  bool last_service = (svc != NULL);
  
  while (NULL != svc)
  {
    ble_gatt_char_t * cha = svc->characteristics;
    printf("Searching service %p (char head %p)\n", svc, svc->characteristics);
    while (NULL != cha)
    {
      switch(cha->uuid.len)
      {
        case ESP_UUID_LEN_16:  printf("char  16 %04x\n", cha->uuid.uuid.uuid16); break;
        case ESP_UUID_LEN_32:  printf("char  32 %08x\n", cha->uuid.uuid.uuid32); break;
        case ESP_UUID_LEN_128: printf("char 132 %08x\n", cha->uuid.uuid.uuid32); break;
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


void ble_gatt_start(void)
{
  ble_gatt_service_t * svc = service_head;
  while(svc != NULL)
  {
    printf("Registering ble_service_id %d\n", svc->ble_service_id);
    esp_ble_gatts_app_register(svc->ble_service_id);
    svc = svc->next;
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


      // Remember the assigned if
      svc->gatts_if = gatts_if;
      
      // Create service
      // This will create and then fire the callback with the event BTA_GATTS_CREATE_SRVC_EVT
      //
      esp_ble_gatts_create_service(gatts_if, 
        &svc->service_id,         // Why is this a pointer?
        4); // Number of handles requested?

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
      
      // Add all characteristics to the service
      //
      ble_gatt_char_t *cha = svc->characteristics;
      while(NULL != cha)
      {
        printf("Adding Characteristic %08x\n", cha->uuid.uuid.uuid32);
        esp_ble_gatts_add_char(
          svc->service_handle,
          &cha->uuid,
          cha->perm,
          cha->properties, //
          NULL, // value (initial?)
          NULL  // attribute response control byte
        );
          
        cha = cha->next;
      }
        
      // esp_ble_gatts_add_char(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
      //                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      //                        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
      //                        NULL, NULL);
      break;
    }
    
    case ESP_GATTS_ADD_CHAR_EVT:
    {
      ESP_LOGI(LOGT, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
               param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
               
      // Lookup by characteristic UUID
      //
      ble_gatt_char_t * cha = ble_gatt_get_char_by_uuid(svc, &param->add_char.char_uuid);
      ESP_LOGI(LOGT, "got characteristic %p\n", cha);
      
      // Record info on the added characteristics
      //
      // Record attr_handle
      cha->handle = param->add_char.attr_handle;
      
      // Debug something or other?
      /*
      uint16_t length = 0;
      const uint8_t *prf_char;
      esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
      for(int i = 0; i < length; i++)
      {
          ESP_LOGI(LOGT, "prf_char[%x] =%x\n",i,prf_char[i]);
      }
      */

      // Record and apply the Client Characteristic Configuration Descriptor
      // so that the connected device can tell us that it wants notifications (or indications).
      //
      cha->descr_uuid.len = ESP_UUID_LEN_16;
      cha->descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

      esp_ble_gatts_add_char_descr(svc->service_handle, &cha->descr_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // descriptor access permission
                                   NULL, // char descriptor value
                                   NULL  // attribute response control byte
                                  );
      break;
    }
        
          
    case ESP_GATTS_READ_EVT: 
    {
      ESP_LOGI(LOGT, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);

      // TODO: dispatch read/write callback properly

      esp_gatt_rsp_t rsp;
      memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
      rsp.attr_value.handle = param->read.handle;
      rsp.attr_value.len = 4;
      rsp.attr_value.value[0] = 0xde;
      rsp.attr_value.value[1] = 0xed;
      rsp.attr_value.value[2] = 0xbe;
      rsp.attr_value.value[3] = 0xef;
      esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                  ESP_GATT_OK, &rsp);
      break;
    }
  
    case ESP_GATTS_WRITE_EVT: 
    {
      // TODO: dispatch read/write callback properly
      ESP_LOGI(LOGT, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
      ESP_LOGI(LOGT, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
      
      //example_write_event_env(gatts_if, &b_prepare_write_env, param);
      break;
    }
  
  
    case ESP_GATTS_EXEC_WRITE_EVT:
      // TODO: dispatch read/write callback properly
      ESP_LOGI(LOGT,"ESP_GATTS_EXEC_WRITE_EVT");
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
      //example_exec_write_event_env(&b_prepare_write_env, param);
      break;
      
    case ESP_GATTS_MTU_EVT: printf("ESP_GATTS_MTU_EVT\n");break;
    case ESP_GATTS_CONF_EVT: printf("ESP_GATTS_CONF_EVT\n");break;
    case ESP_GATTS_UNREG_EVT: printf("ESP_GATTS_UNREG_EVT\n");
      break;


    case ESP_GATTS_ADD_INCL_SRVC_EVT: printf("ESP_GATTS_ADD_INCL_SRVC_EVT\n");
      break;


    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
      ESP_LOGI(LOGT, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
               param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
      break;

    case ESP_GATTS_DELETE_EVT:
      break;

    case ESP_GATTS_START_EVT:
      ESP_LOGI(LOGT, "SERVICE_START_EVT, status %d, service_handle %d\n",
               param->start.status, param->start.service_handle);
      break;

    case ESP_GATTS_STOP_EVT:
      break;
      
    case ESP_GATTS_CONNECT_EVT:
      ESP_LOGI(LOGT, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d\n",
               param->connect.conn_id,
               param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
               param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
               param->connect.is_connected);

      svc->conn_id = param->connect.conn_id;

      // Start sent the update connection parameters to the peer device.
      // For the IOS system, please reference the apple official documents 
      // about the BLE connection parameters restrictions
      //
      ble_gap_update_connection_params(&param->connect.remote_bda);

      break;

    case ESP_GATTS_DISCONNECT_EVT:
      esp_ble_gap_start_advertising(&adv_params);
      break;
      
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
      break;
  }
}
 