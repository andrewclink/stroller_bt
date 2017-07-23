#ifndef motor_h
#define motor_h

void motor_task(void *unused);

// Change setpoint
void motor_setPace_KmS(int km_p_sec);

#endif