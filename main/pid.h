#ifndef pid_h
#define pid_h

//Define parameter
#define PID_epsilon 100
#define PID_MAX 7                   //For Current Saturation
#define PID_MIN -10

// bad, but working:
// #define Kp  0.05
// #define Ki  0.0001
// #define Kd  0.001


void pid_get(uint16_t *delta_ms, float * kp, float * ki, float * kd);

void pid_set(uint16_t delta_ms, float kp, float ki, float kd);

// Avoid jump on switch from manual override
void pid_resetstate(void);

float pid_calc(float setpoint, float actual);


#endif