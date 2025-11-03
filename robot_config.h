// ===============================================
//  Robotic Car - Centralized Configuration
//  Description: All GPIO pins and tunable parameters
//               in one place for easy updates
// ===============================================
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "hardware/i2c.h"

// ============================================================================
// GPIO PIN ASSIGNMENTS - Robo Pico Board
// ============================================================================

// ---- Motor Control Pins ----
#define MOTOR_L_PWM         10      // Left motor PWM (GP10)
#define MOTOR_L_DIR         11      // Left motor direction (GP11)
#define MOTOR_R_PWM         8       // Right motor PWM (GP8)
#define MOTOR_R_DIR         9       // Right motor direction (GP9)

// ---- Encoder Pins ----
#define ENCODER_LEFT_PIN    3       // Left encoder (GP3)
#define ENCODER_RIGHT_PIN   6       // Right encoder (GP6)

// ---- IMU Pins (LSM303DLHC) ----
#define IMU_I2C_INST        i2c0    // I2C instance
#define IMU_I2C_SDA         16      // I2C SDA (GP16)
#define IMU_I2C_SCL         17      // I2C SCL (GP17)
#define IMU_I2C_BAUD_HZ     400000  // I2C clock speed

// ---- Ultrasonic Sensor Pins (HC-SR04) ----
#define ULTRASONIC_TRIG     5       // Trigger pin (GP5)
#define ULTRASONIC_ECHO     4       // Echo pin (GP4)

// ---- IR Sensor Pins ----
// Left IR Sensor (Line Following)
#define LINE_IR_DO_PIN      26      // Left IR digital output (GP26)
#define LINE_IR_AO_PIN      27      // Left IR analog output (GP27)
#define LINE_ADC_INPUT      1       // ADC channel for GP27

// Right IR Sensor (Barcode Scanning)
#define BARCODE_IR_DO_PIN   7       // Right IR digital output (GP7)
#define BARCODE_IR_AO_PIN   28      // Right IR analog output (GP28) - available

// ---- System Control Pins ----
#define EMERGENCY_STOP_PIN  20      // Emergency stop button (GP20)

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================
#define TICKS_PER_REV       360.0f  // Encoder ticks per wheel revolution
#define WHEEL_CIRCUM_M      0.21f   // Wheel circumference in meters
#define RPM_TIMEOUT_US      500000  // 0.5 sec - assume stopped if no ticks

// ============================================================================
// LINE SENSOR CONFIGURATION
// ============================================================================
#define LINE_THRESHOLD      2000    // ADC threshold (white ~0-100, black ~4000-4095)
#define LINE_SAMPLE_COUNT   8       // Number of samples to average

// ============================================================================
// BARCODE SCANNER CONFIGURATION
// ============================================================================
#define BARCODE_SAMPLE_MS   1       // Sampling period in milliseconds
#define BARCODE_VERIFY_MS   2       // Verification delay
#define BARCODE_BAR_IS_LOW  0       // 0: black=HIGH, 1: black=LOW
#define BARCODE_RESET_MS    1000    // Reset timeout if no activity (ms)
#define BARCODE_MAX_LEN     16      // Maximum decoded message length

// ============================================================================
// IMU CONFIGURATION
// ============================================================================
#define IMU_DECLINATION_RAD     0.0f    // Magnetic declination (radians)
#define IMU_HEADING_EMA_ALPHA   0.15f   // EMA filter alpha [0..1], lower = smoother

// ============================================================================
// PID CONTROLLER GAINS
// ============================================================================
// Speed PID
#define PID_KP_SPEED        0.45f   // Proportional gain
#define PID_KI_SPEED        0.05f   // Integral gain
#define PID_KD_SPEED        0.02f   // Derivative gain
#define PID_SPEED_I_MAX     200.0f  // Anti-windup limit

// Heading PID (tuned for narrow 1.5cm line at low speed)
#define PID_KP_HEADING      0.8f    // Proportional gain (higher for faster response)
#define PID_KI_HEADING      0.05f   // Integral gain (eliminate drift)
#define PID_KD_HEADING      0.25f   // Derivative gain (damping)
#define PID_HEADING_I_MAX   50.0f   // Anti-windup limit

// ============================================================================
// ROBOT BEHAVIOR CONFIGURATION
// ============================================================================
// Movement speeds (0.0 to 1.0)
#define BASE_SPEED          0.5f    // Base speed for line following
#define TURN_SPEED          0.20f   // Speed during 90° turns

// Turn parameters
#define TURN_ANGLE_DEG      90.0f   // Target angle for 90° turns
#define TURN_TOLERANCE_DEG  5.0f    // Acceptable angle error for turns
#define TURN_TIMEOUT_MS     5000    // Maximum turn duration (ms)

// Line following behavior
#define SEARCH_INTENSITY    0.4f    // How hard to turn while searching for line
#define RECOVERY_CYCLES     15      // Stabilization cycles after finding line (150ms @ 100Hz)

// Barcode detection timing
#define BARCODE_ENABLE_TIME_MS  500     // Time on line before enabling barcode scan
#define BARCODE_TIMEOUT_MS      2000    // Barcode detection timeout

// ============================================================================
// FREERTOS TASK PRIORITIES
// ============================================================================
#define LINE_FOLLOW_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define BARCODE_TASK_PRIORITY       (tskIDLE_PRIORITY + 2)
#define TELEMETRY_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)
#define TURN_TASK_PRIORITY          (tskIDLE_PRIORITY + 3)
#define STATE_MONITOR_PRIORITY      (tskIDLE_PRIORITY + 2)

// ============================================================================
// FREERTOS TASK STACK SIZES
// ============================================================================
#define LINE_FOLLOW_STACK_SIZE      2048
#define BARCODE_STACK_SIZE          2048
#define TELEMETRY_STACK_SIZE        2048
#define TURN_STACK_SIZE             2048
#define STATE_MONITOR_STACK_SIZE    2048

// ============================================================================
// DEBUG AND TELEMETRY
// ============================================================================
#define TELEMETRY_REPORT_RATE_MS    200     // Telemetry update period (5Hz)
#define LINE_FOLLOW_UPDATE_RATE_MS  10      // Line following loop rate (100Hz)
#define BARCODE_UPDATE_RATE_MS      10      // Barcode polling rate (100Hz)

#endif // ROBOT_CONFIG_H
