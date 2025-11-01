// ===============================================
//  Module: PID Controller
//  Description: Provides PID control logic for speed and heading loops.
// ===============================================
#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"

// ---- PID initialization ----
void pid_init(void);

// ---- Compute corrections ----
float pid_compute_speed(float target_speed, float measured_speed);
float pid_compute_heading(float heading_error);

// ---- Get PID gains (for debugging/tuning) ----
void pid_get_heading_gains(float *kp, float *ki, float *kd);

#endif
