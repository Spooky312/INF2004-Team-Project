// ultrasonic.h
#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include "pico/stdlib.h"

void ultrasonic_init(uint trig, uint echo);
float ultrasonic_get_distance_cm(uint trig, uint echo);

#endif
