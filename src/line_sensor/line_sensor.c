// ===============================================
//  Module: Line Sensor Driver
//  Description: ADC-based line detection for PID control
// ===============================================
#include "line_sensor.h"
#include <stdio.h>

static uint16_t threshold = LINE_THRESHOLD;

void line_sensor_init(void) {
    adc_init();
    adc_gpio_init(LINE_IR_AO_PIN);
    adc_select_input(LINE_ADC_INPUT);
    printf("[LINE_SENSOR] Initialized on GPIO %d (ADC%d)\n", LINE_IR_AO_PIN, LINE_ADC_INPUT);
}

line_state_t line_sensor_read(void) {
    uint16_t value = adc_read();
    // High ADC = black line, Low ADC = white surface
    return (value > threshold) ? LINE_BLACK : LINE_WHITE;
}

uint16_t line_sensor_read_raw(void) {
    return adc_read();
}

void line_sensor_set_threshold(uint16_t new_threshold) {
    threshold = new_threshold;
    printf("[LINE_SENSOR] Threshold set to %u\n", threshold);
}

float line_sensor_get_error(void) {
    // Simple single-sensor implementation
    // Returns: -1.0 (off line left), 0.0 (on line), +1.0 (off line right)
    line_state_t state = line_sensor_read();
    
    // For a single sensor centered on the line:
    // BLACK (on line) = 0.0 error
    // WHITE (off line) = error depends on last known direction
    
    static float last_error = 0.0f;
    
    if (state == LINE_BLACK) {
        last_error = 0.0f;
        return 0.0f;
    } else {
        // Lost line - maintain last error direction
        // In a real implementation with multiple sensors, 
        // you'd calculate weighted position here
        return last_error;
    }
}

// ===============================================
//  Line Following Control Implementation
// ===============================================

// Smoothing state for sine-wave motion
static float steering_smooth = 0.0f;

// Tuning parameters for responsive yet smooth motion within 2cm line gap
#define SMOOTH_ALPHA 0.20f          // Faster response but still smooth (0.15-0.25)
#define CORRECTION_STRENGTH 0.06f   // Moderate 6% differential - reactive but controlled (0.05-0.08)

void line_sensor_reset_state(void) {
    steering_smooth = 0.0f;
}

motor_commands_t line_sensor_compute_motor_commands(void) {
    motor_commands_t commands;
    
    // Read line sensor
    line_state_t line_state = line_sensor_read();
    
    // Compute target steering value based on sensor reading
    float steering_target;
    
    if (line_state == LINE_BLACK) {
        // BLACK LINE: Want to turn RIGHT
        // Positive steering = turn right (left wheel faster)
        steering_target = +1.0f;
    } else {
        // WHITE SURFACE: Want to turn LEFT
        // Negative steering = turn left (right wheel faster)
        steering_target = -1.0f;
    }
    
    // Apply exponential moving average (low-pass filter) for smooth sine-wave motion
    // This gradually transitions between left and right instead of sharp switches
    steering_smooth = steering_smooth * (1.0f - SMOOTH_ALPHA) + steering_target * SMOOTH_ALPHA;
    
    // Apply normalized steering to motor commands
    // steering_smooth ranges from -1.0 (left) to +1.0 (right)
    float correction = steering_smooth * CORRECTION_STRENGTH;
    
    commands.left_speed = BASE_SPEED + correction;   // Positive correction speeds up left (turn right)
    commands.right_speed = BASE_SPEED - correction;  // Positive correction slows down right (turn right)
    
    // Clamp to safe range
    if (commands.left_speed < 0.1f) commands.left_speed = 0.1f;
    if (commands.left_speed > 1.0f) commands.left_speed = 1.0f;
    if (commands.right_speed < 0.1f) commands.right_speed = 0.1f;
    if (commands.right_speed > 1.0f) commands.right_speed = 1.0f;
    
    return commands;
}
