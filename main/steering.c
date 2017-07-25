#include <stdio.h>

#include <rom/ets_sys.h> // ets_delay_us
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "steering.h"


#define STEPPER_ENABLE_PIN 12
#define STEPPER_SLEEP_PIN  14
#define STEPPER_STEP_PIN   27
#define STEPPER_DIR_PIN    26


static volatile int16_t req_pos = 0; // negative is left, positive is right
static volatile int16_t cur_pos = 0; // Assume homed   

static void steering_task(void *parm);

void steering_init(void)
{
  gpio_set_direction(STEPPER_ENABLE_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(STEPPER_SLEEP_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(STEPPER_STEP_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(STEPPER_DIR_PIN, GPIO_MODE_OUTPUT);
  
  stepper_enable(false);
  stepper_sleep(false);
  
  xTaskCreate(&steering_task, "steering_task", 2048, NULL, 6, NULL);
}

void stepper_enable(bool enable)
{
  // low to enable
  printf("str: setting !enable pin %d\n", !enable);
  gpio_set_level(STEPPER_ENABLE_PIN, !enable);
}

void stepper_sleep(bool sleep)
{
  // low to sleep/reset
  printf("str: setting !sleep/reset pin %d\n", !sleep);
  gpio_set_level(STEPPER_SLEEP_PIN, !sleep);
}

inline void _stepper_pulse(void)
{
  gpio_set_level(STEPPER_STEP_PIN, 1);
  ets_delay_us(1);
  gpio_set_level(STEPPER_STEP_PIN, 0);
  
}
void stepper_step_fwd(void)
{
  gpio_set_level(STEPPER_DIR_PIN, 1);
  _stepper_pulse();
  cur_pos ++;
}

void stepper_step_rev(void)
{
  gpio_set_level(STEPPER_DIR_PIN, 0);
  _stepper_pulse();
  cur_pos --;
}

float _stepper_accel()
{
  // return change * time / duration + start;
  return 0;
}

void stepper_home(void)
{
  cur_pos = 0;
}


void stepper_set_pos(int16_t pos)
{
  printf("str: setting requested pos %d\n", pos);
  req_pos = pos;
}

int16_t stepper_get_pos(void)
{
  return req_pos;
}

static void steering_task(void *parm)
{
  int delay = 1;
  
  #define unrolled_delay 1000
  
  for(;;)
  {
    // printf("str: req % 5d; cur % 5d\n", req_pos, cur_pos);
    if (abs(cur_pos - req_pos) > 4)
    {
      if (cur_pos < req_pos)
      {
        stepper_step_fwd();
      }
      else
      if (cur_pos > req_pos)
      {
        stepper_step_rev();
      }
    }

    // TODO: Handle acceleration
    //
    // This is 100 ms, which is far too slow.
    vTaskDelay(1);
  }
}