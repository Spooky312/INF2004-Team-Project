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
// ---- Speed PID Gains (TUNED: slightly softer) ----
// Slightly reduced KP and KI to avoid sudden large throttle changes
// when large heading corrections are momentarily applied.
#define PID_KP_SPEED        0.35f   // Proportional gain for speed control (reduced)
#define PID_KI_SPEED        0.03f   // Integral gain for speed control (reduced)
#define PID_KD_SPEED        0.02f   // Derivative gain for speed control (unchanged)
// Lower integral cap to limit long-term speed correction magnitude
#define PID_SPEED_I_MAX     100.0f  // Anti-windup limit for speed integral (reduced)

// ---- Heading PID Gains ----
// Tuned for smooth heading tracking with LSM303DLHC magnetometer
// ---- Heading PID Gains (TUNED: less aggressive, more damping) ----
// These conservative values reduce overshoot and improve stability.
// - Lower KP to reduce aggressive responses to heading error
// - Much smaller KI to avoid wind-up on temporary large errors
// - Increase KD to provide stronger damping of oscillations
#define PID_KP_HEADING      0.25f   // Proportional gain for heading control (reduced)
#define PID_KI_HEADING      0.005f  // Integral gain for heading control (reduced)
#define PID_KD_HEADING      0.30f   // Derivative gain for heading control (increased)
// Reduce integral max to limit long-term buildup
#define PID_HEADING_I_MAX   20.0f   // Anti-windup limit for heading integral (reduced)

// ============================================================================
// IMU CONFIGURATION
// ============================================================================

// ---- EMA Filter for Heading ----
#define IMU_EMA_ALPHA       0.15f   // EMA smoothing factor (0.0-1.0)
                                    // Lower = smoother but slower response
                                    // Higher = faster but noisier
                                    // Reduced to 0.15 for maximum stability
                                    
#define IMU_OUTLIER_THRESHOLD 30.0f // Reject heading changes > this value (degrees)
                                    // Helps filter EMI from motors/WiFi
                                    // Reduced to 30° for stricter rejection

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
#define TARGET_SPEED        100.0f   // Default target speed (0-255)
                                    // Adjust based on your motors and battery

#define MAX_MOTOR_SPEED     255.0f  // Maximum PWM speed value
#define MIN_MOTOR_SPEED     0.0f    // Minimum PWM speed value

// ---- Directional / motor trims (use these to correct asymmetry between
//     left and right motors or to bias heading corrections per-side).
//    Default values are 1.0 (no change). If your robot turns faster to the
//    right than left, reduce the right trim slightly (e.g. 0.95) or increase
//    the left trim (e.g. 1.05) until behavior is symmetric.
#define MOTOR_LEFT_TRIM     1.0f    // Multiply left motor output
#define MOTOR_RIGHT_TRIM    1.0f   // Multiply right motor output (reduced to correct faster right turns)

// ---- Heading correction per-side scale. Sometimes you may prefer to scale
//     the heading correction applied to each motor independently. Default
//     is 1.0 (symmetric). Values >1 amplify the correction on that side.
#define HEADING_CORR_LEFT_SCALE  1.10f
#define HEADING_CORR_RIGHT_SCALE 1.0f

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
