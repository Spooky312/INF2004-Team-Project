// ===============================================
//  Robotic Car - Centralized Configuration (Demo 1)
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
#define IMU_I2C_BAUD_HZ     400000  // I2C clock speed (400kHz)

// ---- Debug LED Pins ----
#define STATUS_LED_PIN      25      // Onboard LED (heartbeat)
#define IMU_LED_PIN         19      // IMU status indicator (GP19)
#define ENCODER_LED_PIN     24      // Encoder status indicator (GP24)
#define WIFI_LED_PIN        23      // WiFi status indicator (GP23)

// ---- Button Pins ----
#define CHG_DIRECTION_PIN   21      // Change direction button (GP21)

// ---- Ultrasonic Sensor Pins ----
#define ULTRASONIC_TRIG_PIN 5       // Ultrasonic trigger (GP5)
#define ULTRASONIC_ECHO_PIN 4       // Ultrasonic echo (GP4)

// ---- IR Sensor Pins (Line Detection/Barcode) ----
#define IR_LEFT_DO_PIN      26      // Left IR digital out (GP26)
#define IR_LEFT_AO_PIN      27      // Left IR analog out (GP27)
#define IR_RIGHT_DO_PIN     7       // Right IR digital out (GP7)
#define IR_RIGHT_AO_PIN     28      // Right IR analog out (GP28)

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================
#define TICKS_PER_REV       360.0f  // Encoder ticks per wheel revolution
#define WHEEL_CIRCUM_M      0.21f   // Wheel circumference in meters
#define RPM_TIMEOUT_US      500000  // 0.5 sec - assume stopped if no ticks

// ============================================================================
// PID CONTROLLER TUNING
// ============================================================================

// ---- Speed PID Gains ----
#define PID_KP_SPEED        0.45f   // Proportional gain for speed control
#define PID_KI_SPEED        0.05f   // Integral gain for speed control
#define PID_KD_SPEED        0.02f   // Derivative gain for speed control
#define PID_SPEED_I_MAX     200.0f  // Anti-windup limit for speed integral

// ---- Heading PID Gains ----
// Tuned for smooth heading tracking with LSM303DLHC magnetometer
#define PID_KP_HEADING      0.50f   // Proportional gain for heading control
#define PID_KI_HEADING      0.02f   // Integral gain for heading control
#define PID_KD_HEADING      0.15f   // Derivative gain for heading control
#define PID_HEADING_I_MAX   50.0f   // Anti-windup limit for heading integral

// ============================================================================
// IMU CONFIGURATION
// ============================================================================

// ---- EMA Filter for Heading ----
#define IMU_EMA_ALPHA       0.30f   // EMA smoothing factor (0.0-1.0)
                                    // Lower = smoother but slower response
                                    // Higher = faster but noisier
                                    // Increased from 0.15 to 0.30 for better tracking

// ---- Magnetometer Calibration ----
// Hard-iron offset calibration (adjust based on your environment)
// To calibrate: rotate robot 360° and record min/max values
#define MAG_X_OFFSET        0.0f    // X-axis offset (hard-iron)
#define MAG_Y_OFFSET        0.0f    // Y-axis offset (hard-iron)
#define MAG_Z_OFFSET        0.0f    // Z-axis offset (hard-iron)

// Soft-iron scale factors (for ellipsoid correction)
#define MAG_X_SCALE         1.0f    // X-axis scale
#define MAG_Y_SCALE         1.0f    // Y-axis scale
#define MAG_Z_SCALE         1.0f    // Z-axis scale

// ============================================================================
// MOTOR CONTROL PARAMETERS
// ============================================================================
#define TARGET_SPEED        80.0f   // Default target speed (0-255)
                                    // Adjust based on your motors and battery

#define MAX_MOTOR_SPEED     255.0f  // Maximum PWM speed value
#define MIN_MOTOR_SPEED     0.0f    // Minimum PWM speed value

// ============================================================================
// TASK TIMING CONFIGURATION (FreeRTOS)
// ============================================================================

// ---- Task Priorities ----
#define WIFI_TASK_PRIORITY      (tskIDLE_PRIORITY + 3)
#define PID_TASK_PRIORITY       (tskIDLE_PRIORITY + 2)
#define TELEMETRY_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

// ---- Task Periods ----
#define PID_TASK_PERIOD_MS      20      // 50 Hz control loop
#define TELEMETRY_TASK_PERIOD_MS 1000   // 1 Hz telemetry reporting
#define TELEMETRY_PERIOD_MS     1000    // Alias for compatibility

// ---- Task Stack Sizes ----
#define PID_TASK_STACK_SIZE     2048
#define TELEMETRY_TASK_STACK_SIZE 2048
#define WIFI_TASK_STACK_SIZE    2048

// ============================================================================
// STATE MACHINE CONFIGURATION
// ============================================================================

// ---- Transition Timing ----
#define TRANSITION_RAMP_DOWN_TICKS  25  // 0.5 seconds ramp down
#define TRANSITION_STOP_HOLD_TICKS  40  // 0.3 seconds hold at stop
#define TRANSITION_RAMP_UP_TICKS    65  // 0.5 seconds ramp up

// ============================================================================
// FEATURE FLAGS
// ============================================================================

// ---- Enable/Disable Features ----
#define HAVE_CHG_DIRECTION  1       // Set to 1 to enable direction change button
#define ENABLE_WIFI_MQTT    1       // Set to 1 to enable WiFi/MQTT telemetry
#define ENABLE_DEBUG_LEDS   1       // Set to 1 to enable debug LED indicators

// ============================================================================
// DEBUGGING OPTIONS
// ============================================================================

// ---- Verbose Logging ----
#define DEBUG_PID_VERBOSE   0       // Set to 1 for detailed PID debug output
#define DEBUG_IMU_VERBOSE   0       // Set to 1 for detailed IMU debug output
#define DEBUG_ENCODER_VERBOSE 0     // Set to 1 for detailed encoder debug output

// ---- Debug Print Intervals ----
#define PID_DEBUG_INTERVAL  50      // Print PID status every N loops (1 sec @ 50Hz)
#define TELEMETRY_DEBUG_INTERVAL 1  // Print telemetry every N loops (1 sec @ 1Hz)

#endif // ROBOT_CONFIG_H
