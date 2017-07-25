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

#include "steering.h"
#include "motor.h"
#include "term.h"

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

enum {
  speed_uuid             = 0x1419, // Speed
  steering_angle_uuid    = 0x1428, // Steering Angle
  steering_home_uuid     = 0x1429, // Steering Home Control Point
};


static TaskHandle_t motor_task_h;


void client_didConnect(bool connected)
{
  if (!connected)
  {
    // Phone disconnected; stop the motors
    motor_setPace_SpKM(0);
    motor_set_us(0);
    stepper_set_pos(0);
    printf("main: failsafe\n");
  }
    
}

esp_gatt_status_t stroller_didRequestValue(ble_gatt_service_t *svc, ble_gatt_char_t * characteristic, void * buffer, uint16_t *len)
{
  uint16_t * buf = (uint16_t*)buffer;
  
  // Write back the client
  //
  switch((uint16_t)characteristic->uuid.uuid.uuid16)
  {
    case speed_uuid:
    {
      uint16_t speed = motor_getPace_SpKM();
      *buf++ = speed;
      *len = 2;
      
      return ESP_GATT_OK;
    }
    
    case steering_home_uuid:
    {
      return ESP_GATT_READ_NOT_PERMIT;
    }
    
    case steering_angle_uuid:
    {
      int16_t tmpangle = stepper_get_pos();
      *buf++ = tmpangle;
      *len = 2;
      
      return ESP_GATT_OK;
    }
    
    default:
      return ESP_GATT_NOT_FOUND;
  }
  
}

esp_gatt_status_t stroller_didWriteValue(ble_gatt_service_t *svc, ble_gatt_char_t * characteristic, void * value, uint16_t *len)
{
  // Handle write payload
  //
  // If less than +len+ bytes are handled, the value +len+ points to should be updated
  //
  
  // printf("OUT ");
  // for(int i=0; i<*len; i++)
  //   printf("%02x ", buf[i]);
  // printf("\n");
  
  
  
  switch((uint16_t)characteristic->uuid.uuid.uuid16)
  {
    case speed_uuid:
    {

      int16_t speed = 0;
      uint16_t * buf = (uint16_t*)value;
      speed = *buf++;
      
      // Update the motor module
      motor_setPace_SpKM(speed);
      
      return ESP_GATT_OK;
    }
    
    
    case steering_angle_uuid:
    {
      uint16_t * buf = (uint16_t*)value;
      int16_t tmpangle = *buf++;
      stepper_set_pos(tmpangle);
      
      return ESP_GATT_OK;
    }
    
    
    case steering_home_uuid:
      return ESP_GATT_READ_NOT_PERMIT;
    
    default:
      return ESP_GATT_NOT_FOUND;
  }
}



void app_main()
{
    printf("Stroller\n");

    uint16_t uuid_tmp;

    // Create the stroller service & characteristics
    ble_gatt_service_t * stroller_service = 
      ble_gatt_service_create(stroller_service_uuid, ESP_UUID_LEN_128, stroller_didRequestValue, stroller_didWriteValue);

    uuid_tmp = speed_uuid;
    ble_gatt_char_t * speed = ble_gatt_characteristic_create(stroller_service, &uuid_tmp, ESP_UUID_LEN_16);
    speed->properties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

    uuid_tmp = steering_angle_uuid;
    ble_gatt_char_t * sa = ble_gatt_characteristic_create(stroller_service, &uuid_tmp, ESP_UUID_LEN_16);
    sa->properties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
    
    uuid_tmp = steering_home_uuid;
    ble_gatt_char_t * steering_home = ble_gatt_characteristic_create(stroller_service, &uuid_tmp, ESP_UUID_LEN_16);
    steering_home->perm       = ESP_GATT_PERM_WRITE;
    steering_home->properties = ESP_GATT_CHAR_PROP_BIT_WRITE;
    
    // Start BLE
    gap_start();
    ble_gatt_start(client_didConnect);
    
    // Init steering controller
    steering_init();
    stepper_enable(true);
    stepper_sleep(false);
      
    // Start Motor Monitor
    xTaskCreate(&motor_task, "motor_t", 2048, NULL, 6, &motor_task_h);
    
    //A uart read/write example without event queue;
    xTaskCreate(term_task, "term_task", 2048, NULL, 1, NULL);
    return;
}
