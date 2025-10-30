// ===============================================
//  Module: Encoder Driver
//  Description: Handles encoder tick interrupts and computes wheel RPM.
// ===============================================
#ifndef ENCODER_H
#define ENCODER_H

#define ENCODER_LEFT_PIN   27
#define ENCODER_RIGHT_PIN  6

#include "pico/stdlib.h"

void encoder_init(void);
float encoder_get_rpm_left(void);
float encoder_get_rpm_right(void);
float encoder_get_distance_m(void);
void encoder_get_ticks(uint32_t *left, uint32_t *right);  // Diagnostic
void encoder_irq_handler(uint gpio, uint32_t events);

#endif