#include "pid.h"
#include "pico/stdlib.h"
#include <math.h> // For fabs

// Define static PID controller instances for left and right motors
static pid_controller_t left_motor_pid;
static pid_controller_t right_motor_pid;

// Initialize a single PID controller's parameters and state
void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->last_time_us = time_us_32();
    pid->first_run = true; // Mark as first run to handle initial derivative term
}

// Compute the PID output based on setpoint and measurement
float pid_compute(pid_controller_t *pid, float setpoint, float measurement) {
    uint32_t now = time_us_32();
    uint64_t diff_us = absolute_time_diff_us(pid->last_time_us, now);
    float dt = (float)diff_us / 1000000.0f;  // Convert microseconds to seconds

    // --- Robustness check for dt ---
    // If dt is zero or negative (timer wrap?), or unreasonably large, use a default or skip derivative
    if (dt <= 0.0f || dt > 0.5f) { // Max plausible interval 0.5s? Tune this.
         dt = 0.01f; // Use a default reasonable dt like 10ms (matching typical task rate)
         // Avoid derivative calculation on the first run or after a bad dt
         pid->first_run = true;
    }

    pid->last_time_us = now;

    // Calculate error
    float error = setpoint - measurement;

    // Proportional term
    float p_term = pid->kp * error;

    // Integral term with anti-windup
    pid->integral += error * dt;

    // Clamp integral term to prevent windup
    // A common method: clamp based on output limits and other terms
    // More advanced: conditional integration, back-calculation
    float max_integral_contribution = (pid->output_max - p_term) / (pid->ki + 1e-6f); // Approximation
    float min_integral_contribution = (pid->output_min - p_term) / (pid->ki + 1e-6f); // Approximation
    if (pid->integral > max_integral_contribution) pid->integral = max_integral_contribution;
    if (pid->integral < min_integral_contribution) pid->integral = min_integral_contribution;
    // Simpler clamping (less effective anti-windup):
    // float max_abs_integral = 10.0f / (pid->ki + 1e-6f); // Tune this limit
    // if (pid->integral > max_abs_integral) pid->integral = max_abs_integral;
    // if (pid->integral < -max_abs_integral) pid->integral = -max_abs_integral;


    float i_term = pid->ki * pid->integral;

    // Derivative term
    float derivative = 0.0f;
    if (!pid->first_run && dt > 0) { // Calculate derivative only if dt is valid and not the first run
        derivative = (error - pid->prev_error) / dt;
    } else {
        pid->first_run = false; // Next run will calculate derivative
    }
    float d_term = pid->kd * derivative;

    // Store error for next derivative calculation
    pid->prev_error = error;

    // Calculate total output
    float output = p_term + i_term + d_term;

    // Clamp output to defined limits
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

// Reset the internal state of a PID controller
void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_time_us = time_us_32();
    pid->first_run = true; // Reset first run flag
}

// --- FIX: Added new function definition ---
// Reset both motor PID controllers
void pid_reset_all(void) {
    pid_reset(&left_motor_pid);
    pid_reset(&right_motor_pid);
}

// Initialize the PID subsystem (specifically the motor PIDs)
void pid_init(void) {
    // --- Tune these PID values based on testing your specific robot ---
    // Factors: motor type, gearbox, wheel size, weight, surface, sensor noise

    // Example starting values (adjust significantly based on observation):
    float common_kp = 0.08f; // Increase P first for responsiveness
    float common_ki = 0.02f; // Increase I slowly to reduce steady-state error
    float common_kd = 0.005f; // Increase D carefully to dampen overshoot/oscillations

    // Safety limits for PWM output (-90% to +90% based on motor.h MAX_SPEED)
    float min_output = -0.9f;
    float max_output = 0.9f;

    // Initialize left motor PID controller
    pid_controller_init(&left_motor_pid,
                       common_kp, common_ki, common_kd,
                       min_output, max_output);

    // Initialize right motor PID controller (might need slightly different tuning)
    pid_controller_init(&right_motor_pid,
                       common_kp, common_ki, common_kd, // Start with same values
                       min_output, max_output);
    printf("PID controllers initialized.\n");
}

// Compute PID output for the left motor
float pid_compute_left(float setpoint_cmps, float measurement_cmps) {
    return pid_compute(&left_motor_pid, setpoint_cmps, measurement_cmps);
}

// Compute PID output for the right motor
float pid_compute_right(float setpoint_cmps, float measurement_cmps) {
    return pid_compute(&right_motor_pid, setpoint_cmps, measurement_cmps);
}