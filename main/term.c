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
#include "steering.h"

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
        uart_write_bytes(uart_num, (const char*)&rx, 1); // echo
        // uart_flush(uart_num);               // don't wait for \n
        
        
        // handle escape commands from pyserial
        // For other terminals THIS WILL BE DIFFERENT
        // But we're only debugging, right?
        //
        // If this doesn't work in +make monitor+: pyserial had a bug in 
        // it that would do a blocking read when there was already data
        // in the buffer. How much life did that cost me? :)
        // 
        //     # [sudo] pip uninstall pyserial
        //     # [sudo] pip install pyserial
        //
        if (rx == 0x1b)
        {
          // Escape sequence
          line[0] = rx;

          // read the next few bytes here and process the command
          uart_read_bytes(uart_num, &line[1], 2, 1);
          // printf("term: got sequence %02x %02x\n", line[1], line[2]);
          break;
        }
        
        // Handle normal text commands
        if (rx == '\n'
           ) 
          break;

        if (line + BUF_SIZE - 1 <= ptr)
        {
          printf("rx buf overflow\n");
          break;
        }

        *ptr++ = rx;

      }

      // We have a command in +line+ here
      
      // First char is the op code
      switch(line[0])
      {
        case 0x1b:
          // this is a command key. The third byte is the command.
          switch(line[2])
          {
            case 0x41: // up arrow
              stepper_set_pos(0);
              printf("term: center\n");
              break;
              
            case 0x44: // left arrow
              stepper_set_pos(50);
              break;
              
            case 0x43: // right arrow
              stepper_set_pos(-50);
              break;
              

            default: 
              printf("term: Unhandled esc %02x %02x %02x\n", line[0], line[1], line[2]);
              break;
          }
          break;
        
        // Steering Commands
        //
        case 'E':
          printf("Enable steering\n");
          stepper_enable(true);
          break;
          
        case 'e':
          printf("Disable steering\n");
          stepper_enable(false);
          break;

        case 's':
        {
          int val = atoi((char*)&line[1]);
          printf("set steering to %d\n", val);
          stepper_set_pos(val);
          break;
        }
        
        case '+': stepper_step_rev(); break;
        case '-': stepper_step_fwd(); break;
        

        // Motor Commands
        //
        case 'p':
        case 'i':
        case 'd':
        {
          uint16_t ms;
          float p, i, d;
          pid_get(&ms, &p, &i, &d);
					printf("setting pid\n");
					printf("orig: %.4f %.4f %.4f\n", p, i, d);
          
          float var = atof((char*)&line[1]);
          switch(line[0])
          {
            case 'p': p = var; break;
            case 'i': i = var; break;
            case 'd': d = var; break;
          }
          
					printf("new:  %.4f %.4f %.4f\n", p, i, d);
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
          printf("term error (%c)\n", line[0]);
        break;
        
      }
      
    }
}
