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

// ---- Dynamic bias learning states ----
static float bias_integral = 0.0f;        // Long-term average of speed error
static float motor_bias_adjustment = 0.0f; // Calculated motor bias
static uint32_t samples_collected = 0;     // Number of samples for bias learning

// ---- Initialization ----
void pid_init(void)
{
    speed_integral = 0.0f;
    prev_speed_err = 0.0f;
    heading_integral = 0.0f;
    prev_heading_err = 0.0f;
    
    // Reset bias learning
    bias_integral = 0.0f;
    motor_bias_adjustment = 0.0f;
    samples_collected = 0;
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
    // Very small deadband for narrow line - we need precision
    if (heading_error > -0.5f && heading_error < 0.5f) {
        heading_error = 0.0f;
    }
    
    heading_integral += heading_error;
    float derivative = heading_error - prev_heading_err;
    prev_heading_err = heading_error;

    float output = (PID_KP_HEADING * heading_error) +
                   (PID_KI_HEADING * heading_integral) +
                   (PID_KD_HEADING * derivative);

    // Anti-windup limit for integral term
    if (heading_integral > PID_HEADING_I_MAX) heading_integral = PID_HEADING_I_MAX;
    if (heading_integral < -PID_HEADING_I_MAX) heading_integral = -PID_HEADING_I_MAX;

    // Clamp output for motor correction range (increased from ±30 to ±50)
    if (output > 50.0f) output = 50.0f;
    if (output < -50.0f) output = -50.0f;

    return output;
}
// ---- Get PID gains for debugging/tuning ----
void pid_get_heading_gains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = PID_KP_HEADING;
    if (ki) *ki = PID_KI_HEADING;
    if (kd) *kd = PID_KD_HEADING;
}

// ---- Dynamic bias compensation functions ----
// Updates bias learning with exponential moving average
void pid_update_bias(float speed_error)
{
    samples_collected++;
    
    // Update bias integral with exponential moving average
    // This learns the long-term tendency (e.g., left motor consistently faster)
    bias_integral = bias_integral * (1.0f - BIAS_LEARNING_RATE) + speed_error * BIAS_LEARNING_RATE;
    
    // Convert bias integral to motor adjustment
    // Positive bias_integral means left wheel is consistently faster -> need to slow it down
    motor_bias_adjustment = bias_integral * 0.015f;  // Scale to motor power range
    
    // Clamp bias adjustment to prevent extreme values
    if (motor_bias_adjustment > BIAS_MAX_ADJUSTMENT) motor_bias_adjustment = BIAS_MAX_ADJUSTMENT;
    if (motor_bias_adjustment < -BIAS_MAX_ADJUSTMENT) motor_bias_adjustment = -BIAS_MAX_ADJUSTMENT;
}

// Returns the current motor bias adjustment value
float pid_get_bias_adjustment(void)
{
    return motor_bias_adjustment;
}

// Returns bias statistics for debugging
void pid_get_bias_stats(float *bias_int, uint32_t *samples)
{
    if (bias_int) *bias_int = bias_integral;
    if (samples) *samples = samples_collected;
}
