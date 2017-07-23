#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"

#include "pid.h"
#include "motor.h"

#define ECHO_TEST_TXD  (4)
#define ECHO_TEST_RXD  (5)
#define ECHO_TEST_RTS  (18)
#define ECHO_TEST_CTS  (19)

#define BUF_SIZE (128)

// HACK
extern bool debug_pid;
extern bool debug_mot;


//an example of echo test with hardware flow control on UART1
  void term_task(void * unused)
{
    const int uart_num = UART_NUM_0;
  /*
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART1 parameters
    uart_param_config(uart_num, &uart_config);
    //Set UART1 pins(TX: IO4, RX: I05, RTS: IO18, CTS: IO19)
    uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
  */
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t* line = (uint8_t*) malloc(BUF_SIZE);

    while(1) 
    {
      memset(line, '\0', BUF_SIZE);
            
      // Read a line
      uint8_t * ptr = line;
      for(;;)
      {
        uint8_t rx;
        uart_read_bytes(uart_num, &rx, 1, portMAX_DELAY);
        printf("%c", rx);
        
        if (rx != '\n')
        {
          if (line + BUF_SIZE - 1 <= ptr)
          {
            printf("rx buf overflow\n");
            break;
          }

          *ptr++ = rx;
        }
        else
          break;
      }

      // We have a line in +line+ here
      printf("%s\n", line);
      
      // First char is the op code
      switch(line[0])
      {
        case 'p':
        case 'i':
        case 'd':
        {
          uint16_t ms;
          float p, i, d;
          pid_get(&ms, &p, &i, &d);
          
          float var = atof((char*)&line[1]);
          switch(line[0])
          {
            case 'p': p = var; break;
            case 'i': i = var; break;
            case 'd': d = var; break;
          }
          
          pid_set(ms, p, i, d);
          break;
        }
        
        case 'P':
          debug_pid = !debug_pid;
          break;

        case 'M':
          debug_mot = !debug_mot;
          break;

        case 'u':
        {
          int us = atoi((char*)&line[1]);
          printf("set motor to %u us\n", us);
          motor_setManual(true);
          motor_set_us(us);
          break;
        }
        
        
        case 'a':
        {
          printf("Enabling PID\n");
          motor_setManual(false);
          break;
        }
        
        
        case 'r':
        {
          // set RPM
          int rpm = atoi((char*)&line[1]);
          
          printf("set motor to %u RPM\n", rpm);
          // motor_setManual(true);
          motor_setRPM(rpm);
          break;
        }
        
        
        
        default: 
          printf("term error\n");
        break;
        
      }
      
    }
}
