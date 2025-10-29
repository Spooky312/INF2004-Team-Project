// servo.h
#ifndef SERVO_H
#define SERVO_H
#include "pico/stdlib.h"

void servo_init(uint gpio);
void servo_set_angle(uint gpio, float angle);

#endif
