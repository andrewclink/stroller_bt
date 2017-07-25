#ifndef steering_h
#define steering_h

void steering_init(void);
void stepper_home(void);

void stepper_enable(bool enable);
void stepper_sleep(bool sleep);

void stepper_set_pos(int16_t pos);
int16_t stepper_get_pos(void);
void stepper_step_fwd(void);
void stepper_step_rev(void);

#endif