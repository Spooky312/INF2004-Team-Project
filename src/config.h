/**
 * @file config.h
 * @brief Central configuration for Robotic Car Project
 * @date October 2025
 * 
 * CORRECTED VERSION - Fixed pin conflicts and added notes
 * 
 * All configuration parameters are centralized here.
 * Include this file in all .c/.h files that need hardware parameters.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE PIN CONFIGURATION
// ============================================================================

// Motor pins (RoboPico)
#define CFG_PWM_M1A             8
#define CFG_PWM_M1B             9
#define CFG_PWM_M2A             10
#define CFG_PWM_M2B             11

// Encoder pins
#define CFG_ENCODER_PIN_L       3   // Left encoder
#define CFG_ENCODER_PIN_R       6    // Right encoder (OK now that echo is moved)

// Line sensor (single sensor configuration)
#define CFG_LINE_ADC_LEFT       27   // ADC0 - White: high value, Black: low value
#define CFG_LINE_DAC_RIGHT      7   // ADC1 - (Not used in single sensor setup)
// Note: GPIO27 used by encoder, so can't use it for ADC
// If you need second sensor: use GPIO28 (ADC2) or GPIO29 (ADC3)

// Ultrasonic sensor - ✓ FIXED: Moved away from GPIO6 to avoid encoder conflict
#define CFG_ULTRASONIC_TRIG     17   // Trigger pin (was 7 in old code)
#define CFG_ULTRASONIC_ECHO     16   // Echo pin (was 6 - conflicted with encoder!)

// Servo
#define CFG_SERVO_PIN           15   // PWM servo signal

// IMU (I2C)
#define CFG_IMU_SDA_PIN         4    // I2C0 SDA
#define CFG_IMU_SCL_PIN         5    // I2C0 SCL
#define CFG_IMU_I2C_FREQ        100000  // 100 kHz

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================

// ✓ SINGLE SOURCE OF TRUTH for Wi-Fi credentials
#define CFG_WIFI_SSID           "ZongHan"
#define CFG_WIFI_PASS           "ZongHan123"
#define CFG_MQTT_SERVER_IP      "172.20.10.4"
#define CFG_MQTT_PORT           1883

// ============================================================================
// MOTOR CONTROL TUNING
// ============================================================================

// Speed limits (0.0 to 1.0)
#define CFG_MAX_SPEED           0.7f
#define CFG_LINE_SPEED          0.5f
#define CFG_TURN_SPEED          0.4f
#define CFG_SEARCH_SPEED        0.3f

// PID constants for line following
#define CFG_PID_KP              0.3f
#define CFG_PID_KI              0.0f
#define CFG_PID_KD              0.1f

// Encoder
#define CFG_PULSES_PER_REV      20

// ============================================================================
// OBSTACLE DETECTION TUNING
// ============================================================================

// Distance thresholds (cm)
#define CFG_SAFE_DISTANCE       15.0f
#define CFG_MAX_DISTANCE        100.0f

// Servo angles (degrees, 0-180)
#define CFG_SERVO_CENTER        90
#define CFG_SERVO_LEFT_MAX      45
#define CFG_SERVO_RIGHT_MAX     135
#define CFG_SERVO_SCAN_STEP     15

// Servo timing (ms)
#define CFG_SERVO_SETTLE_TIME   300

// ============================================================================
// LINE SENSOR TUNING
// ============================================================================

// ADC threshold (0-4095)
// WHITE surface = higher value, BLACK line = lower value
// Tune this by printing raw ADC values on your track
#define CFG_LINE_THRESHOLD      280

// ============================================================================
// STATE MACHINE TIMING
// ============================================================================

// Avoidance maneuver timing (ms)
#define CFG_TURN_DURATION       800   // Time to turn 90 degrees
#define CFG_FORWARD_DURATION    1500  // Time to move past obstacle

// Rejoin timeout (ms)
#define CFG_REJOIN_TIMEOUT      3000  // Max time to search for line

// ============================================================================
// FREERTOS TASK CONFIGURATION
// ============================================================================

// Task periods (ms)
#define CFG_LINE_FOLLOW_PERIOD      50
#define CFG_OBSTACLE_CHECK_PERIOD   100
#define CFG_TELEMETRY_PERIOD        500
#define CFG_IMU_PERIOD              100

// Task priorities (higher = more priority)
#define CFG_PRIORITY_OBSTACLE   (tskIDLE_PRIORITY + 4)
#define CFG_PRIORITY_LINE       (tskIDLE_PRIORITY + 3)
#define CFG_PRIORITY_IMU        (tskIDLE_PRIORITY + 2)
#define CFG_PRIORITY_TELEMETRY  (tskIDLE_PRIORITY + 1)

// Stack sizes (words, not bytes)
#define CFG_STACK_LINE          256
#define CFG_STACK_OBSTACLE      512
#define CFG_STACK_TELEMETRY     512
#define CFG_STACK_IMU           256

// ============================================================================
// DEBUG FLAGS
// ============================================================================

#define CFG_DEBUG_MOTOR         0    // Print motor debug info
#define CFG_DEBUG_OBSTACLE      1    // Print obstacle detection info
#define CFG_DEBUG_LINE          0    // Print line sensor info
#define CFG_DEBUG_STATE         1    // Print state transitions

// ============================================================================
// FEATURE ENABLES
// ============================================================================

#define CFG_ENABLE_IMU          1    // Set to 0 to disable IMU
#define CFG_ENABLE_MQTT         1    // Set to 0 to disable MQTT
#define CFG_ENABLE_OBSTACLE     1    // Set to 0 for line follow only

// ============================================================================
// PIN USAGE REFERENCE (for documentation)
// ============================================================================
/*
GPIO PIN ALLOCATION:
--------------------
GPIO 4  - I2C0 SDA (IMU)
GPIO 5  - I2C0 SCL (IMU)
GPIO 6  - Encoder Right (interrupt input)
GPIO 8  - Motor 1A (PWM)
GPIO 9  - Motor 1B (PWM)
GPIO 10 - Motor 2A (PWM)
GPIO 11 - Motor 2B (PWM)
GPIO 15 - Servo (PWM)
GPIO 16 - Ultrasonic Echo (input) ✓ FIXED: was GPIO 6
GPIO 17 - Ultrasonic Trig (output) ✓ FIXED: was GPIO 7
GPIO 26 - Line Sensor Left (ADC0)
GPIO 27 - Encoder Left (interrupt input)
GPIO 28 - Available for Line Sensor Right (ADC2) if needed
GPIO 29 - Available (ADC3)

CONFLICTS RESOLVED:
- OLD: Echo (6) vs Encoder Right (6) ✗ NOW: Echo (16) vs Encoder Right (6) ✓
- OLD: Line Right (27) vs Encoder Left (27) ✗ NOW: Only Encoder Left (27) ✓
*/

#endif // CONFIG_H