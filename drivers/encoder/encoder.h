#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

// Encoder pins from main (2).c
#define ENCODER_PIN_L 27
#define ENCODER_PIN_R 6

// Encoder properties
#define PULSES_PER_REV 20.0f // Adjust for your specific encoders
#define WHEEL_DIAMETER_M 0.065f // Example: 65mm wheel

void encoder_init(void);
float encoder_get_left_rpm(void);
float encoder_get_right_rpm(void);
float encoder_get_distance_m(void);

#endif // ENCODER_H