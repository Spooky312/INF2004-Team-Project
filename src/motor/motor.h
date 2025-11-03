// ===============================================
//  Module: Motor Driver
//  Description: Controls left and right DC motors using PWM signals.
// ===============================================
#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

void motor_init(void);
void motor_set_speed(float left_speed, float right_speed);
void motor_stop(void);

#endif
