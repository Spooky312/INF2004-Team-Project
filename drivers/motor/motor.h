#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"

// Pin definitions for Cytron RoboPico
#define PWM_M1A 8   
#define PWM_M1B 9
#define PWM_M2A 10
#define PWM_M2B 11

void motor_init(void);
void set_robo_motor(float speed_percent, uint pinA, uint pinB);

#endif // MOTOR_H