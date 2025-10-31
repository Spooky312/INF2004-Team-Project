/**
 * @file obstacle_detection.h
 * @brief Ultrasonic sensor with servo scanning for obstacle detection
 * @date October 2025
 * 
 * FIXED VERSION - Uses config.h values to avoid pin conflicts
 */

#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "config.h"  // ✓ ADDED - Use centralized config

// Use config values instead of hardcoded pins
#define TRIG_PIN CFG_ULTRASONIC_TRIG  // GPIO 17 (was 7 - wrong!)
#define ECHO_PIN CFG_ULTRASONIC_ECHO  // GPIO 16 (was 6 - conflict with encoder!)
#define SERVO_PIN CFG_SERVO_PIN       // GPIO 15

// Detection parameters - use config values
#define SAFE_DISTANCE_CM CFG_SAFE_DISTANCE
#define MAX_DISTANCE_CM CFG_MAX_DISTANCE

// Servo angles - use config values
#define SERVO_CENTER CFG_SERVO_CENTER
#define SERVO_LEFT_MAX CFG_SERVO_LEFT_MAX
#define SERVO_RIGHT_MAX CFG_SERVO_RIGHT_MAX
#define SERVO_SCAN_STEP CFG_SERVO_SCAN_STEP

/**
 * @brief Scan direction
 */
typedef enum {
    SCAN_LEFT,
    SCAN_CENTER,
    SCAN_RIGHT
} scan_direction_t;

/**
 * @brief Obstacle detection result
 */
typedef struct {
    bool obstacle_detected;
    float distance_cm;
    scan_direction_t direction;
    uint32_t timestamp_us;
} obstacle_reading_t;

/**
 * @brief Obstacle scan result with clearance info
 */
typedef struct {
    bool obstacle_present;
    float obstacle_width_cm;
    float left_clearance_cm;
    float center_distance_cm;
    float right_clearance_cm;
    bool left_clear;
    bool right_clear;
    scan_direction_t recommended_path;
    uint32_t scan_time_ms;
} obstacle_scan_t;

/**
 * @brief Initialize obstacle detection system
 */
bool obstacle_detection_init(void);

/**
 * @brief Measure distance in current servo direction
 * @return Distance in cm, or MAX_DISTANCE_CM if no echo
 */
float obstacle_measure_distance(void);

/**
 * @brief Set servo angle
 * @param angle Angle in degrees (0-180)
 */
void obstacle_set_servo_angle(uint8_t angle);

/**
 * @brief Point servo in specific direction
 */
void obstacle_look_direction(scan_direction_t direction);

/**
 * @brief Quick check for obstacle straight ahead
 * @return True if obstacle within safe distance
 */
bool obstacle_check_ahead(void);

/**
 * @brief Perform full left-center-right scan
 * @return Complete scan data with recommended avoidance path
 */
obstacle_scan_t obstacle_perform_scan(void);

/**
 * @brief Measure obstacle width by scanning edges
 * @return Width in cm, or 0 if couldn't measure
 */
float obstacle_measure_width(void);

/**
 * @brief Get single reading at current angle
 */
obstacle_reading_t obstacle_get_reading(void);

#endif // OBSTACLE_DETECTION_H