// ===============================================
//  Module: PID Controller
//  Description: Implements discrete PID loops for speed and heading control.
// ===============================================
#include "pid.h"
#include "robot_config.h"

// All PID gains now in robot_config.h:
// - PID_KP_SPEED, PID_KI_SPEED, PID_KD_SPEED
// - PID_KP_HEADING, PID_KI_HEADING, PID_KD_HEADING
// - PID_SPEED_I_MAX, PID_HEADING_I_MAX

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

    float output = (PID_KP_SPEED * error) + (PID_KI_SPEED * speed_integral) + (PID_KD_SPEED * derivative);

    // Anti-windup limit for integral term
    if (speed_integral > PID_SPEED_I_MAX) speed_integral = PID_SPEED_I_MAX;
    if (speed_integral < -PID_SPEED_I_MAX) speed_integral = -PID_SPEED_I_MAX;

    return output;
}

// ---- Heading PID ----
float pid_compute_heading(float heading_error)
{
    // Apply 1.0 RPM deadband (ignore small encoder noise)
    if (heading_error > -1.0f && heading_error < 1.0f) {
        heading_error = 0.0f;
    }
    
    // Pure PID control - integral accumulates and becomes feedforward compensation
    heading_integral += heading_error;
    float derivative = heading_error - prev_heading_err;
    prev_heading_err = heading_error;

    float output = (PID_KP_HEADING * heading_error) +
                   (PID_KI_HEADING * heading_integral) +
                   (PID_KD_HEADING * derivative);

    // Anti-windup limit for integral term
    if (heading_integral > PID_HEADING_I_MAX) heading_integral = PID_HEADING_I_MAX;
    if (heading_integral < -PID_HEADING_I_MAX) heading_integral = -PID_HEADING_I_MAX;

    return output;
}
// ---- Get PID gains for debugging/tuning ----
void pid_get_heading_gains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = PID_KP_HEADING;
    if (ki) *ki = PID_KI_HEADING;
    if (kd) *kd = PID_KD_HEADING;
}

// Returns the current PID integral term (for debugging)
float pid_get_heading_integral(void)
{
    return heading_integral;
}

// ---- Legacy bias functions (kept for API compatibility - return dummy values) ----
void pid_update_bias(float speed_error)
{
    // No-op: Bias is now handled by PID integral term
}

float pid_get_bias_adjustment(void)
{
    return 0.0f;  // No separate bias adjustment
}

void pid_get_bias_stats(float *bias_int, uint32_t *samples)
{
    if (bias_int) *bias_int = 0.0f;
    if (samples) *samples = 0;
}
