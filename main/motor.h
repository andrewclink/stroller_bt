#ifndef motor_h
#define motor_h

void motor_task(void *unused);

// Change setpoint
void motor_setPace_SpKM(int km_p_sec);
uint16_t motor_getPace_SpKM(void);

// Convert values
float motor_rpm_to_spk(double rpm);

// Manually commit to a us value
void motor_set_us(int us);
void motor_setRPM(int RPM);

// Turn on/off the PID
void motor_setManual(bool manual);

#endif