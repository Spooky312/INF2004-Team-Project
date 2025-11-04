// ===============================================
//  Module: PID Controller
//  Description: Provides PID control logic for speed and heading loops
//               with dynamic motor bias compensation.
// ===============================================
#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"

// ---- PID initialization ----
void pid_init(void);

// ---- Compute corrections ----
float pid_compute_speed(float target_speed, float measured_speed);
float pid_compute_heading(float heading_error);

// ---- Dynamic bias compensation for encoder-based straight line control ----
// Updates bias learning based on speed error and returns motor bias adjustment
void pid_update_bias(float speed_error);
float pid_get_bias_adjustment(void);
void pid_get_bias_stats(float *bias_integral, uint32_t *samples);

// ---- Get PID gains (for debugging/tuning) ----
void pid_get_heading_gains(float *kp, float *ki, float *kd);

#endif
