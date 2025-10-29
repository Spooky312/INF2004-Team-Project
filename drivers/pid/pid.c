#include "pid.h"
#include "encoder/encoder.h"
#include "motor/motor.h"
#include "imu/imu.h"
#include "pico/stdlib.h"
#include <stdio.h>

// ============================================================================
// !! CRITICAL !! - PID GAIN TUNING
// ============================================================================
// These values are GUESSES and WILL need to be tuned. [cite: 222]
// Start with Kp, leave Ki and Kd at 0.
// Increase Kp until the robot oscillates (shakes).
// Then, set Kp to about 50-60% of that value.
// Then, slowly increase Ki to fix steady-state error.
// Then, slowly increase Kd to reduce overshoot.

// --- Gains for Wheel Speed (RPM) Control ---
#define PID_SPEED_KP 0.02f   // Proportional gain
#define PID_SPEED_KI 0.01f   // Integral gain
#define PID_SPEED_KD 0.001f  // Derivative gain

// --- Gains for Heading (IMU) Control ---
#define PID_HEADING_KP 0.01f // Proportional gain
#define PID_HEADING_KI 0.005f// Integral gain
#define PID_HEADING_KD 0.0f  // Derivative gain

// --- PID Output Clamping ---
#define PID_OUTPUT_MAX 1.0f
#define PID_OUTPUT_MIN -1.0f
#define PID_INTEGRAL_MAX 1.0f
#define PID_INTEGRAL_MIN -1.0f


/**
 * @brief Holds the state for a single PID controller instance.
 */
typedef struct {
    float Kp, Ki, Kd;       // Gains
    float integral;         // Accumulated integral error
    float prev_error;       // Error from the last cycle
    float setpoint;         // The target value (e.g., target RPM)
    uint64_t last_time_us;  // Last time this PID was calculated (in us)
} pid_state_t;

// --- Controller Instances ---
static pid_state_t pid_speed_left;
static pid_state_t pid_speed_right;
static pid_state_t pid_heading;

/**
 * @brief Helper function to initialize a pid_state_t struct
 */
static void pid_state_init(pid_state_t *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_time_us = time_us_64();
}

/**
 * @brief Clamps a value between a min and a max.
 */
static inline float clamp(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief Calculates the PID output for one controller.
 */
static float pid_calculate(pid_state_t *pid, float current_value, float dt_s) {
    // 1. Calculate error
    float error = pid->setpoint - current_value;

    // Special handling for heading wrap-around (e.g., 359 -> 1 degree)
    if (pid == &pid_heading) {
        if (error > 180.0f)      error -= 360.0f;
        else if (error < -180.0f) error += 360.0f;
    }

    // 2. Proportional term
    float P = pid->Kp * error;

    // 3. Integral term (with anti-windup)
    pid->integral += error * dt_s;
    pid->integral = clamp(pid->integral, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);
    float I = pid->Ki * pid->integral;

    // 4. Derivative term
    float derivative = (dt_s > 0) ? (error - pid->prev_error) / dt_s : 0;
    pid->prev_error = error;
    float D = pid->Kd * derivative;

    // 5. Calculate total output
    float output = P + I + D;
    return output;
}


// ============================================================================
// PUBLIC API FUNCTIONS
// ============================================================================

void pid_init(void) {
    pid_state_init(&pid_speed_left, PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, 0.0f);
    pid_state_init(&pid_speed_right, PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, 0.0f);
    pid_state_init(&pid_heading, PID_HEADING_KP, PID_HEADING_KI, PID_HEADING_KD, 0.0f);
}

void pid_set_target_rpm(float left_rpm, float right_rpm) {
    pid_speed_left.setpoint = left_rpm;
    pid_speed_right.setpoint = right_rpm;
}

void pid_set_target_heading(float heading) {
    pid_heading.setpoint = heading;
}

void pid_controller_run(void) {
    // This function is called from pid_task, already inside a mutex
    
    // --- 1. Get current time and calculate deltas (dt) ---
    uint64_t now_us = time_us_64();
    float dt_speed_l = (now_us - pid_speed_left.last_time_us) / 1e6f;
    float dt_speed_r = (now_us - pid_speed_right.last_time_us) / 1e6f;
    float dt_heading = (now_us - pid_heading.last_time_us) / 1e6f;
    pid_speed_left.last_time_us = now_us;
    pid_speed_right.last_time_us = now_us;
    pid_heading.last_time_us = now_us;

    // --- 2. Get Sensor Feedback ---
    float current_rpm_l = encoder_get_left_rpm();
    float current_rpm_r = encoder_get_right_rpm();
    float current_heading = imu_get_filtered_heading(); // From imu.c

    // --- 3. Run PID Calculations ---
    float speed_output_l = pid_calculate(&pid_speed_left, current_rpm_l, dt_speed_l);
    float speed_output_r = pid_calculate(&pid_speed_right, current_rpm_r, dt_speed_r);
    float heading_correction = pid_calculate(&pid_heading, current_heading, dt_heading);

    // --- 4. Combine Outputs and Set Motors ---
    // Left  = Base_L - Correction
    // Right = Base_R + Correction
    float final_output_l = speed_output_l - heading_correction;
    float final_output_r = speed_output_r + heading_correction;

    // --- 5. Clamp Final Outputs ---
    final_output_l = clamp(final_output_l, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    final_output_r = clamp(final_output_r, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    // --- 6. Set Motor Speeds ---
    set_robo_motor(final_output_l, PWM_M1A, PWM_M1B);
    set_robo_motor(final_output_r, PWM_M2A, PWM_M2B);
}