#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/dport_reg.h"
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"

#include "driver/gpio.h"

#define wheel_circ_mm (304 * M_PI)
#define wheel_pulley_teeth 190
#define motor_pulley_teeth 14

#define REFRESH_USEC         20000
#define REFRESH_HZ           (1000000 / REFRESH_USEC)

#define signal_pin 5
#define pwm_ch 0
#define timer_bitwidth 12
#define timer_ticks  (1 << 12)
#define signal_lo_us 1250
#define signal_hi_us 1900

xSemaphoreHandle _pwm_lock;
#define PWM_MUTEX_LOCK()    do {} while (xSemaphoreTake(_pwm_lock, portMAX_DELAY) != pdPASS)
#define PWM_MUTEX_UNLOCK()  xSemaphoreGive(_pwm_lock)

#define get_pwm_LS_ch(channel) LEDC.channel[LEDC_LOW_SPEED_MODE].channel[(channel)]
#define get_pwm_LS_tm(timer_no) LEDC.channel[LEDC_LOW_SPEED_MODE].timer[(timer_no)]

//uint32_t frequency = (80MHz or 1MHz)/((div_num / 256.0)*(1 << bit_num));
//                      80 / ((div_num / 256) * (1 << bitwidth))

#define TIMER_LS_PARA_UP_b        (1 << 26)
#define TIMER_LS_SLOW_CLK_b       (1 << 25)
#define TIMER_LS_REFTICK_CLK_b    (0 << 25)
#define TIMER_LS_RESET_b          (1 << 24)
#define TIMER_LS_PAUSE_b          (1 << 23)
#define TIMER_LS_DIV_INTEGRAL_bs  (5 + 8)
#define TIMER_LS_DIV_FRAC_bs      (5)
#define TIMER_LS_DIV_bs           (5)
#define TIMER_LS_RANGE_bs         (0) // 4 bits; "The counter range is (0..2 ** n); the maximum bit width for counter is 20."
                                
#define TIMER_LS_DIV_s(_x_)       (_x_ << TIMER_LS_DIV_bs)
#define TIMER_LS_RANGE_s(_x_)     (_x_ << TIMER_LS_RANGE_bs)

#define PWM_CONF0_PARA_UP_b       (1 << 4) // Strobe to update point and duty values
#define PMW_CONF0_IDLE_HIGH_b     (1 << 3)
#define PMW_CONF0_IDLE_LOW_b      (0 << 3)
#define PWM_CONF0_OUT_EN_b        (1 << 2)
#define PWM_CONF0_TIMER_LS_SEL_bs (0)
                                   
#define PWM_CONF1_DUTY_START_b    (1 << 31)
#define PWM_CONF1_DUTY_INC_b      (1 << 30)
#define PWM_CONF1_DUTY_DEC_b      (0 << 30)
#define PWM_CONF1_DUTY_NUM_bs     (20) // number of times the duty cycle is increased or decreased
#define PWM_CONF1_CYCLE_bs        (10)
#define PWM_CONF1_SCALE_bs        (0)



static void motor_start_pwm(void);
static float motor_spk_to_rpm(int sec);
static int motor_us_to_ticks(int usec);

void motor_task(void *parm)
{
  motor_start_pwm();

  for(;;)
  {
    vTaskDelay(100);    
  }
}

void motor_setPace_KmS(int km_p_sec)
{
  printf("pwm: Updating speed: %d\n", km_p_sec);
  float rpm_setpoint = motor_spk_to_rpm(km_p_sec);
  
  printf("pwm: setpoint %.0f\n", rpm_setpoint);
  
}


static void motor_start_pwm(void)
{
  static bool pwm_timer_started = false;
  if (!pwm_timer_started)
  {
    printf("pwm: enable peri\n");

    // Enable LED PWM peripheral
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_LEDC_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST);
      
    _pwm_lock = xSemaphoreCreateMutex();
    
    pwm_timer_started = true;
  }

  
  // Use APB 80mHz clock
  uint64_t clk_freq = APB_CLK_FREQ;
  clk_freq <<= 8; // LEDC_LSTIMER0_CONF_REG divisor is 8-bit decimal;
  uint32_t div_num = (clk_freq >> timer_bitwidth) / REFRESH_HZ;
  printf("pwm: divisor: %x\n", div_num);

  PWM_MUTEX_LOCK();


  // Configure LS Timer 0
  WRITE_PERI_REG (LEDC_CONF_REG, LEDC_APB_CLK_SEL);
  WRITE_PERI_REG (LEDC_LSTIMER0_CONF_REG, TIMER_LS_PARA_UP_b | TIMER_LS_SLOW_CLK_b | TIMER_LS_DIV_s(div_num) | TIMER_LS_RANGE_s(timer_bitwidth));
  WRITE_PERI_REG (LEDC_LSTIMER0_CONF_REG, TIMER_LS_RESET_b);

  
  // Setup channel
  WRITE_PERI_REG (LEDC_LSCH0_CONF0_REG, PWM_CONF0_PARA_UP_b | PMW_CONF0_IDLE_HIGH_b | PWM_CONF0_OUT_EN_b | (0 << PWM_CONF0_TIMER_LS_SEL_bs));
  WRITE_PERI_REG (LEDC_LSCH0_HPOINT_REG, 0); // Signal pulse starts at reset point
  WRITE_PERI_REG (LEDC_LSCH0_DUTY_REG, motor_us_to_ticks(signal_lo_us)); // TODO: Calculate below; should start at zero
  
  WRITE_PERI_REG (LEDC_LSCH0_CONF0_REG, 1 << 31);
  WRITE_PERI_REG (LEDC_LSCH0_CONF1_REG, PWM_CONF1_DUTY_START_b);
  
  //TODO: LEDC_LS_SIG_OUT0_IDX + pwm_ch seems wrong. See gpio_sig_map.h - LEDC_LS_SIG_OUT0_IDX = 79
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[signal_pin], PIN_FUNC_GPIO);
  gpio_set_direction(signal_pin, GPIO_MODE_OUTPUT);
  gpio_matrix_out(signal_pin, LEDC_LS_SIG_OUT0_IDX + pwm_ch, false /*out invert*/, false /*invert output enable*/);

  PWM_MUTEX_UNLOCK();

  printf("pwm_out\n");
}


float motor_spk_to_rpm(int sec)
{
  // We have an FPU!
  //
  // xs / 1km = 1km / x seconds
  // ÷ 1000 = 1m / x seconds
  // = 1m / (x / 60) minutes
  // 
  // circ * rpm = m/min
  // m/min / circ = rpm

  float meters_p_min = 1000.0 / sec * 60.0;
  float wheel_rpm = meters_p_min / (wheel_circ_mm / 1000);
  printf("mot: %ds/km = %.2f WRPM\n", sec, wheel_rpm);
  
  float motor_rpm = wheel_rpm * (wheel_pulley_teeth / motor_pulley_teeth);
  printf("mot: %.2f RPM\n", motor_rpm);
  
  return motor_rpm;
}


int motor_us_to_ticks(int usec)
{
  // Since we have an FPU it is acceptable to cast to float here.
  return (int)((float)usec / ((float)REFRESH_USEC / (float)timer_bitwidth));   
}