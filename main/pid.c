#include <stdio.h>
#include "esp_system.h"

#include "pid.h"

uint16_t pid_interval_ms = 100;

float pid_integration = 0; // integrated error
float pid_last_error  = 0; // derived error
float pid_last_actual = 0; // derivative on measurement

float pid_kp = 0.05;
float pid_kd = 0.001;
float pid_ki = 0.0001;

bool debug_pid = false;

void pid_get(uint16_t *delta_ms, float * kp, float * ki, float * kd)
{
  float dt_seconds = (float)pid_interval_ms / 1000.0;
	
  if (delta_ms) *delta_ms = pid_interval_ms;
  if (kp) *kp = pid_kp;
  if (ki) *ki = pid_ki / dt_seconds;
  if (kd) *kd = pid_kd * dt_seconds;
}

void pid_set(uint16_t delta_ms, float kp, float ki, float kd)
{
  pid_interval_ms = delta_ms;
  
  float dt_seconds = (float)delta_ms / 1000.0;
  
  // Proportion doesn't care about dt
  pid_kp = kp;

  // do calculation of dt term inside the integral and derivative. 
  pid_ki = ki * dt_seconds;
  pid_kd = kd / dt_seconds;
  
  if (debug_pid)
  {
    printf("Kp= %.5f; Ki= %.5f; Kd= %.5f\n", pid_kp, pid_ki, pid_kd);
  }
  
}

void pid_resetstate(void)
{
  pid_integration = 0; // integrated error
  pid_last_error  = 0; // derived error
  pid_last_actual = 0; // derivative on measurement
}


// A PID algorithm mostly taken from Brett Beauregard's article.
// A few notes:
// - The derivative is taken from the input to avoid derivative kick
// - The integral term is calculated seperately to avoid bumps when 
//   ki changes live
// - dt is baked into the ki and kd constants and thus the algo expects
//   to be called at a consistent interval
//
float pid_calc(float setpoint, float actual)
{
	float error;
	float derivative;
	float output;

	//Caculate P,I,D
	error = setpoint - actual;

	//@TODO: In case of error too small then stop intergration

  
  // Integrate
  pid_integration += (pid_ki * error);
  if (pid_integration > PID_MAX) 
    pid_integration = PID_MAX;
  else
  if (pid_integration < PID_MIN) 
    pid_integration = PID_MIN;
  
  
  // Derive
	//derivative = (error - pid_last_error);
  derivative = (actual - pid_last_actual); // d.o.m.
  pid_last_actual = actual;
  
  float proportion = pid_kp * error;
  float d = pid_kd*derivative;
  
  // static int8_t skip = 10;
  // if(skip-- < 0)
  //{
    printf("p %4.2f i %4.2f d %4.2f\n", proportion, pid_integration, d);
  //  skip = 10;
  // }

  
  // Calculate p,i,d
	output = proportion + pid_integration - d;
  if (output > PID_MAX) output = PID_MAX;
  else
  if (output < PID_MIN) output = PID_MIN;

  return output;
}
