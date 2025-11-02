// ===============================================
//  Module: PID Controller
//  Description: Implements discrete PID loops for speed and heading control.
// ===============================================
#include "pid.h"

// ---- Tunable gains ----
// Adjust these empirically for your platform.
static float kp_speed   = 0.40f;  // Reduced slightly for smoother response
static float ki_speed   = 0.04f;  // Reduced to prevent integral windup at lower speeds
static float kd_speed   = 0.03f;  // Increased slightly for better damping

// <-- REDUCED: Gentler heading gains to prevent zig-zagging
// Lower values = slower, smoother corrections
static float kp_heading = 0.30f;  // Reduced from 0.50 - gentler proportional response
static float ki_heading = 0.01f;  // Reduced from 0.02 - slower integral buildup
static float kd_heading = 0.20f;  // Increased from 0.15 - better damping to prevent overshoot

// ---- Internal PID states (separate for left and right wheels) ----
static float speed_integral_left = 0.0f;
static float prev_speed_err_left = 0.0f;

static float speed_integral_right = 0.0f;
static float prev_speed_err_right = 0.0f;

static float heading_integral = 0.0f;
static float prev_heading_err = 0.0f;

// ---- Initialization ----
void pid_init(void)
{
    speed_integral_left = 0.0f;
    prev_speed_err_left = 0.0f;
    speed_integral_right = 0.0f;
    prev_speed_err_right = 0.0f;
    heading_integral = 0.0f;
    prev_heading_err = 0.0f;
}

// ---- Speed PID for LEFT wheel ----
float pid_compute_speed_left(float target_speed, float measured_speed)
{
    float error = target_speed - measured_speed;
    speed_integral_left += error;
    float derivative = error - prev_speed_err_left;
    prev_speed_err_left = error;

    float output = (kp_speed * error) + (ki_speed * speed_integral_left) + (kd_speed * derivative);

    // Anti-windup limit for integral term
    if (speed_integral_left > 200.0f) speed_integral_left = 200.0f;
    if (speed_integral_left < -200.0f) speed_integral_left = -200.0f;

    return output;
}

// ---- Speed PID for RIGHT wheel ----
float pid_compute_speed_right(float target_speed, float measured_speed)
{
    float error = target_speed - measured_speed;
    speed_integral_right += error;
    float derivative = error - prev_speed_err_right;
    prev_speed_err_right = error;

    float output = (kp_speed * error) + (ki_speed * speed_integral_right) + (kd_speed * derivative);

    // Anti-windup limit for integral term
    if (speed_integral_right > 200.0f) speed_integral_right = 200.0f;
    if (speed_integral_right < -200.0f) speed_integral_right = -200.0f;

    return output;
}

// ---- Heading PID ----
float pid_compute_heading(float heading_error)
{
    // Larger deadzone to prevent zig-zagging
    // Only correct if heading error is significant
    if (heading_error > -5.0f && heading_error < 5.0f) {
        heading_error = 0.0f;
        // Also decay the integral when in deadzone to prevent buildup
        heading_integral *= 0.9f;
    }
    
    heading_integral += heading_error;
    float derivative = heading_error - prev_heading_err;
    prev_heading_err = heading_error;

    float output = (kp_heading * heading_error) +
                   (ki_heading * heading_integral) +
                   (kd_heading * derivative);

    // Anti-windup limit for integral term
    if (heading_integral > 100.0f) heading_integral = 100.0f;
    if (heading_integral < -100.0f) heading_integral = -100.0f;

    // Reduced clamp for gentler corrections (from ±50 to ±35)
    if (output > 35.0f) output = 35.0f;
    if (output < -35.0f) output = -35.0f;

    return output;
}
// ---- Get PID gains for debugging/tuning ----
void pid_get_heading_gains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = kp_heading;
    if (ki) *ki = ki_heading;
    if (kd) *kd = kd_heading;
}
