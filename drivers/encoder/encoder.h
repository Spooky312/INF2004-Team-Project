// ===============================================
//  Module: Encoder Driver
//  Description: Handles encoder tick interrupts and computes wheel RPM.
//  Configuration: All pin and timing constants defined in robot_config_demo1.h
// ===============================================
#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

void encoder_init(void);
float encoder_get_rpm_left(void);
float encoder_get_rpm_right(void);
float encoder_get_distance_m(void);
void encoder_get_ticks(uint32_t *left, uint32_t *right);  // Diagnostic
void encoder_irq_handler(uint gpio, uint32_t events);

// Speed calculation functions
float encoder_get_speed_left_mps(void);   // Get left wheel speed in meters per second
float encoder_get_speed_right_mps(void);  // Get right wheel speed in meters per second
float encoder_get_speed_avg_mps(void);    // Get average robot speed in meters per second
float encoder_get_speed_avg_kmh(void);    // Get average robot speed in kilometers per hour

#endif