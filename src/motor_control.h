/**
 * @file motor_control.h
 * @brief Motor control with line following and obstacle avoidance
 * @date October 2025
 * 
 * FIXED VERSION - Uses config.h and removes pin conflicts
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "config.h"  // ✓ ADDED - Use centralized config

// Motor pins - use config values
#define PWM_M1A CFG_PWM_M1A  // GPIO 8
#define PWM_M1B CFG_PWM_M1B  // GPIO 9
#define PWM_M2A CFG_PWM_M2A  // GPIO 10
#define PWM_M2B CFG_PWM_M2B  // GPIO 11

// Line sensor pins - use config values
#define LINE_ADC_LEFT CFG_LINE_ADC_LEFT  // GPIO 26 (ADC0)
// ✓ REMOVED LINE_ADC_RIGHT - was GPIO 27, conflicts with encoder!
// If you need dual sensors later, use GPIO 28 (ADC2)

// Encoder pins - use config values
#define ENCODER_PIN_L CFG_ENCODER_PIN_L  // GPIO 27
#define ENCODER_PIN_R CFG_ENCODER_PIN_R  // GPIO 6

// Motor constants - use config values
#define MAX_SPEED CFG_MAX_SPEED
#define LINE_FOLLOWING_SPEED CFG_LINE_SPEED
#define TURN_SPEED CFG_TURN_SPEED
#define PULSES_PER_REV CFG_PULSES_PER_REV

// Line detection threshold - use config value
#define LINE_THRESHOLD CFG_LINE_THRESHOLD

/**
 * @brief Motor control state
 */
typedef enum {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_TURN_LEFT,
    MOTOR_TURN_RIGHT,
    MOTOR_PIVOT_LEFT,
    MOTOR_PIVOT_RIGHT
} motor_state_t;

/**
 * @brief Line detection result
 */
typedef struct {
    bool on_line;
    int16_t error;  // -100 (far left) to +100 (far right), 0 = centered
    uint16_t left_sensor;
    uint16_t right_sensor;  // Will be 0 if only using single sensor
} line_data_t;

/**
 * @brief Motor telemetry data
 */
typedef struct {
    float speed_left;
    float speed_right;
    float rpm_left;
    float rpm_right;
    uint32_t encoder_left;
    uint32_t encoder_right;
    motor_state_t state;
} motor_telemetry_t;

/**
 * @brief Initialize motor control system
 */
bool motor_control_init(void);

/**
 * @brief Set motor state
 * @param state Desired motor state
 * @param speed Speed percentage (0.0 to 1.0)
 */
void motor_set_state(motor_state_t state, float speed);

/**
 * @brief Stop all motors
 */
void motor_stop(void);

/**
 * @brief Drive forward at specified speed
 */
void motor_forward(float speed);

/**
 * @brief Drive backward at specified speed
 */
void motor_backward(float speed);

/**
 * @brief Turn left (differential speed)
 */
void motor_turn_left(float speed);

/**
 * @brief Turn right (differential speed)
 */
void motor_turn_right(float speed);

/**
 * @brief Pivot turn left (one wheel forward, one back)
 */
void motor_pivot_left(float speed);

/**
 * @brief Pivot turn right (one wheel forward, one back)
 */
void motor_pivot_right(float speed);

/**
 * @brief Read line sensors
 * @return Line detection data
 */
line_data_t motor_read_line_sensors(void);

/**
 * @brief PID line following control
 * @param line_data Current line sensor data
 */
void motor_line_follow(line_data_t *line_data);

/**
 * @brief Get current motor telemetry
 */
motor_telemetry_t motor_get_telemetry(void);

/**
 * @brief Reset encoder counts
 */
void motor_reset_encoders(void);

#endif // MOTOR_CONTROL_H