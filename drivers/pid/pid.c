// ===============================================
//  Module: PID Controller
//  Description: Implements discrete PID loops for speed and heading control.
// ===============================================
#include "pid.h"

// ---- Tunable gains ----
// Adjust these empirically for your platform.
static float kp_speed   = 0.45f;
static float ki_speed   = 0.05f;
static float kd_speed   = 0.02f;

static float kp_heading = 1.20f;
static float ki_heading = 0.00f;
static float kd_heading = 0.05f;

// ---- Internal PID states ----
static float speed_integral = 0.0f;
static float prev_speed_err = 0.0f;

static float heading_integral = 0.0f;
static float prev_heading_err = 0.0f;

// ---- Initialization ----
void pid_init(void)
{
    speed_integral = 0.0f;
    prev_speed_err = 0.0f;
    heading_integral = 0.0f;
    prev_heading_err = 0.0f;
}

// ---- Speed PID ----
float pid_compute_speed(float target_speed, float measured_speed)
{
    float error = target_speed - measured_speed;
    speed_integral += error;
    float derivative = error - prev_speed_err;
    prev_speed_err = error;

    float output = (kp_speed * error) + (ki_speed * speed_integral) + (kd_speed * derivative);

    // Anti-windup limit for integral term
    if (speed_integral > 200.0f) speed_integral = 200.0f;
    if (speed_integral < -200.0f) speed_integral = -200.0f;

    return output;
}

// ---- Heading PID ----
float pid_compute_heading(float heading_error)
{
    heading_integral += heading_error;
    float derivative = heading_error - prev_heading_err;
    prev_heading_err = heading_error;

    float output = (kp_heading * heading_error) +
                   (ki_heading * heading_integral) +
                   (kd_heading * derivative);

    // Clamp output for motor correction range
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;

    return output;
}
