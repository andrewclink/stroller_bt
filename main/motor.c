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
#include "driver/pcnt.h"

#include "motor.h"
#include "pid.h"

#define wheel_circ_mm (304 * M_PI)
#define wheel_pulley_teeth 190
#define motor_pulley_teeth 14
#define SPEED_CHANGE_UNIT 1

// pulsecount
#define RPM_PIN 4
#define PCNT_UNIT PCNT_UNIT_0
#define MOTOR_POLES 14

// PWM<
#define INTV_MS           60
#define REFRESH_USEC         20000
#define REFRESH_HZ           (1000000 / REFRESH_USEC)

#define signal_pin 5 //pwm
#define pwm_ch 0
#define timer_bitwidth 12
#define timer_ticks  (1 << 12)
#define signal_lo_us 1200
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
#define TIMER_LS_RANGE_s(_x_)     ((_x_ << TIMER_LS_RANGE_bs) & 0b11111)

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


static uint32_t motor_us = signal_lo_us;
static uint32_t motor_rpm_setpoint = 0;
static bool motor_is_manual = false;

bool debug_mot = false;

static void motor_start_pwm(void);
static void motor_start_rpm_monitor(void);
static float motor_spk_to_rpm(int sec);
static int motor_us_to_ticks(int usec);

void motor_task(void *parm)
{
  motor_start_pwm();
  motor_start_rpm_monitor();
  
  // Dwell low for a moment to calibrate the speed controller
  // Can't wait to use the three phase controller
  //
  motor_set_us(signal_lo_us);
  vTaskDelay(500 / portTICK_RATE_MS);
  
  pid_set(100, 0.028, 0.025, 0.005);

  for(;;)
  {
    vTaskDelay(60  / portTICK_RATE_MS);
    
    // Debug counted RPM pulses
    int16_t pole_count;
    pcnt_get_counter_value(PCNT_UNIT, &pole_count);
    pcnt_counter_clear(PCNT_UNIT);
    
    // motor
    uint16_t rpm = pole_count * /*1000/60ms=*/(float)(1000.0/INTV_MS) * 60 / MOTOR_POLES;
    
    if (motor_is_manual)
      continue;
    
    if (motor_rpm_setpoint == 0)
    {
      motor_set_us(signal_lo_us);
      continue;
    }

    float pid_out = pid_calc(motor_rpm_setpoint, rpm);

    // How much difference should we make to the output?
    int16_t change_us = round(pid_out * (float)SPEED_CHANGE_UNIT);
    motor_us += change_us;

    // Bound the values out
    if (motor_us > signal_hi_us)
    {
      motor_us = signal_hi_us;
    }
    else
    if (motor_us < signal_lo_us)
    {
      motor_us = signal_lo_us;
    }
    else
    {
      motor_set_us(motor_us);
    }
    
    if (debug_mot)
    {
      uint32_t val = READ_PERI_REG(LEDC_LSTIMER0_VALUE_REG);
      printf("pwm: RPM %4u  SET %4u ### pid %4.2f ### us %4u ### tmr %08x\n", rpm, motor_rpm_setpoint, pid_out, motor_us, val);
    }
  }
}

void motor_setManual(bool manual)
{
  motor_is_manual = manual;

  if (!manual)
  {
    pid_resetstate();
  }
}

void motor_setPace_SpKM(int sec_p_km)
{
  if (sec_p_km == 0)
  {
    printf("pwm: stop\n");
    motor_setRPM(0);
    return;
  }
  
  if (sec_p_km < 210)
  {
    // 3:30/km which is threshold uncomfortable but within capability of a sprint
    // This would be a speed governor.
    sec_p_km = 210;
  }
  
  printf("pwm: Updating speed: %d\n", sec_p_km);
  float rpm_setpoint = motor_spk_to_rpm(sec_p_km);
  
  printf("pwm: setpoint %.0f\n", rpm_setpoint);
  motor_setRPM(rpm_setpoint);
}

void motor_setRPM(int RPM)
{
  // bound upper value for safety
  //
  // mot: 198s/km = 317.29 WRPM
  // mot: 4306.14 RPM
  // 3:30 - 4306.140137

  if (RPM > 4306)
    RPM = 4306;
    
  motor_rpm_setpoint = RPM;
}

void motor_set_us(int us)
{
  motor_us = us;
  
  uint32_t ticks = motor_us_to_ticks(us);
  // printf("pwm: writing %u us; %u ticks\n", us, ticks);
  
  WRITE_PERI_REG(LEDC_LSCH0_DUTY_REG, ticks << 4);
  WRITE_PERI_REG (LEDC_LSCH0_CONF1_REG, PWM_CONF1_DUTY_START_b);
  (*((volatile uint32_t *)ETS_UNCACHED_ADDR(LEDC_LSCH0_CONF0_REG))) |= PWM_CONF0_PARA_UP_b;
}

static void motor_start_pwm(void)
{
  uint32_t val;
  
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
  //WRITE_PERI_REG (LEDC_LSTIMER0_CONF_REG, TIMER_LS_RESET_b);

  
  // Setup channel
  WRITE_PERI_REG (LEDC_LSCH0_CONF0_REG, PWM_CONF0_PARA_UP_b | PMW_CONF0_IDLE_HIGH_b | PWM_CONF0_OUT_EN_b | (0 << PWM_CONF0_TIMER_LS_SEL_bs));
  WRITE_PERI_REG (LEDC_LSCH0_HPOINT_REG, 0); // Signal pulse starts at reset point
  WRITE_PERI_REG (LEDC_LSCH0_DUTY_REG, motor_us_to_ticks(signal_lo_us)); // TODO: Calculate below; should start at zero
  
  WRITE_PERI_REG (LEDC_LSCH0_CONF1_REG, PWM_CONF1_DUTY_START_b);


  val = READ_PERI_REG(LEDC_LSTIMER0_CONF_REG);
  printf("pwm: LEDC_LSTIMER0_CONF_REG: %08x\n", val);
  val = READ_PERI_REG(LEDC_LSCH0_CONF0_REG);
  printf("pwm: LEDC_LSCH0_CONF0_REG: %08x\n", val);
  val = READ_PERI_REG(LEDC_LSCH0_CONF1_REG);
  printf("pwm: LEDC_LSCH0_CONF1_REG: %08x\n", val);
  val = READ_PERI_REG(LEDC_LSCH0_DUTY_REG);
  printf("pwm: LEDC_LSCH0_DUTY_REG:  %08x\n", val);
  

  // Connect to pin
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[signal_pin], PIN_FUNC_GPIO);
  gpio_set_direction(signal_pin, GPIO_MODE_OUTPUT);
  gpio_matrix_out(signal_pin, LEDC_LS_SIG_OUT0_IDX + pwm_ch, false /*out invert*/, false /*invert output enable*/);

  PWM_MUTEX_UNLOCK();

  printf("pwm_out\n");
}


static void motor_start_rpm_monitor(void)
{
  pcnt_config_t pcnt_config = {
    .unit = PCNT_UNIT,
    .pulse_gpio_num = RPM_PIN,

    .ctrl_gpio_num = -1, // No pos/neg control pin
    .channel = PCNT_CHANNEL_0,
    .pos_mode = PCNT_COUNT_INC, // increase on positive edge
    .neg_mode = PCNT_COUNT_INC, // increase on negative edge
    .lctrl_mode = PCNT_MODE_KEEP, // ignore control pin; always count edges
    .hctrl_mode = PCNT_MODE_KEEP, // ignore control pin; always count edges

    .counter_h_lim = 0xffff, // counter limits
    .counter_l_lim = 0, // counter limits
  };

  pcnt_unit_config(&pcnt_config);
  
  pcnt_set_filter_value(PCNT_UNIT, 1); // ignore pulses shorter than x clock edges
  pcnt_filter_enable(PCNT_UNIT);
  
  pcnt_counter_resume(PCNT_UNIT);
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
  return (int)((float)usec / ((float)REFRESH_USEC / (float)pow(2, timer_bitwidth)));   
}