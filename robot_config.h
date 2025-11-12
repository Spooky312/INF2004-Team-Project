// ===============================================
//  robot_config.h - User-editable configuration
//
// Intention: place all frequently-edited parameters in the clearly marked
// "USER EDITABLE SECTION" below. Keep the remainder of the file stable.
// Edit values in the USER EDITABLE SECTION only. Save and rebuild.
// ===============================================

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "hardware/i2c.h"

// ---------------------------------------------------------------------------
// USER EDITABLE SECTION
// Modify only the values in this section while tuning or testing. Keep names
// and units as shown. Comments include guidance and sensible starting values.
// ---------------------------------------------------------------------------

// --- WiFi & MQTT credentials (edit here for convenience)
// Note: for CI/build automation you can override these using CMake -D flags.
#ifndef WIFI_SSID
#define WIFI_SSID "ssid-here"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "pass-here"
#endif

#ifndef MQTT_SERVER_IP
// Broker IP or hostname (default: local PC). Example: "192.168.1.100"
#define MQTT_SERVER_IP "ip-here"
#endif

// --- Hardware pinout (change only if wiring changes) -----------------------
// Motors
#define MOTOR_L_PWM         10      // Left motor PWM (GP10)
#define MOTOR_L_DIR         11      // Left motor direction (GP11)
#define MOTOR_R_PWM         8       // Right motor PWM (GP8)
#define MOTOR_R_DIR         9       // Right motor direction (GP9)

// Encoders
#define ENCODER_LEFT_PIN    3       // Left encoder (GP3)
#define ENCODER_RIGHT_PIN   6       // Right encoder (GP6)

// IMU (I2C)
#define IMU_I2C_INST        i2c0    // I2C instance
#define IMU_I2C_SDA         16      // I2C SDA (GP16)
#define IMU_I2C_SCL         17      // I2C SCL (GP17)
#define IMU_I2C_BAUD_HZ     400000  // I2C speed in Hz

// LEDs & buttons
#define STATUS_LED_PIN      25      // Heartbeat LED
#define IMU_LED_PIN         19      // IMU status
#define ENCODER_LED_PIN     24      // Encoder status
#define WIFI_LED_PIN        23      // WiFi status
#define CHG_DIRECTION_PIN   21      // Change-direction button (if present)

// Ultrasonic / IR sensors (if present)
#define ULTRASONIC_TRIG_PIN 5
#define ULTRASONIC_ECHO_PIN 4
#define IR_LEFT_DO_PIN      26
#define IR_LEFT_AO_PIN      27
#define IR_RIGHT_DO_PIN     7
#define IR_RIGHT_AO_PIN     28

// --- Motion & encoder geometry -------------------------------------------
#define TICKS_PER_REV       360.0f  // Encoder ticks per wheel revolution
#define WHEEL_CIRCUM_M      0.21f   // Wheel circumference (meters)

// --- Default motion setpoints & limits -----------------------------------
#define TARGET_SPEED        80.0f   // Default target speed (units: rpm-ish / scaled)
#define MAX_MOTOR_SPEED     255.0f  // PWM max
#define MIN_MOTOR_SPEED     0.0f

// --- PID tuning: start here when adjusting behavior ----------------------
// Speed PID (controls throttle to maintain target speed)
#define PID_KP_SPEED        0.35f   // Proportional
#define PID_KI_SPEED        0.03f   // Integral
#define PID_KD_SPEED        0.02f   // Derivative
#define PID_SPEED_I_MAX     100.0f  // Integral windup cap

// Heading PID (controls differential between left/right motors)
#define PID_KP_HEADING      0.25f   // Proportional
#define PID_KI_HEADING      0.005f  // Integral
#define PID_KD_HEADING      0.30f   // Derivative
#define PID_HEADING_I_MAX   20.0f   // Integral windup cap

// Heading deadband and EMA smoothing
#define IMU_EMA_ALPHA       0.15f   // EMA alpha for heading filter (0.0-1.0)
#define IMU_OUTLIER_THRESHOLD 30.0f // Ignore sudden jumps > this (degrees)

// Magnetic declination: adjust this for your local magnetic declination so
// compass headings align to true north. Edit in degrees (positive east).
#ifndef IMU_DECLINATION_DEG
#define IMU_DECLINATION_DEG 0.0f
#endif

// Declination in radians used by code. This expands to an expression using
// M_PI; the translation unit that uses this macro includes <math.h> so M_PI
// should be available there.
#ifndef IMU_DECLINATION_RAD
#define IMU_DECLINATION_RAD (IMU_DECLINATION_DEG * (float)M_PI / 180.0f)
#endif

// --- Motor trims & heading scales ----------------------------------------
// Use these to correct physical asymmetry or tune correction aggressiveness
// without touching PID gains.
#define MOTOR_LEFT_TRIM     1.00f   // Multiply left motor output (1.0 = unchanged)
#define MOTOR_RIGHT_TRIM    1.00f   // Multiply right motor output (reduced to correct bias)

// Per-side multiplier applied to heading correction term only
#define HEADING_CORR_LEFT_SCALE  1.00f
#define HEADING_CORR_RIGHT_SCALE 1.00f

// ---------------------------------------------------------------------------
// Compatibility aliases for older driver macros
// Some driver headers (older variants) expect different macro names. Provide
// conservative aliases here so drivers that include the old names continue to
// work while keeping `robot_config.h` as the single source of truth.
#ifndef IMU_I2C
#define IMU_I2C IMU_I2C_INST
#endif

#ifndef IMU_HEADING_EMA_ALPHA
#define IMU_HEADING_EMA_ALPHA IMU_EMA_ALPHA
#endif

// --- Tasks & timing (tune if you change control frequency) --------------
#define PID_TASK_PERIOD_MS      20      // Control loop period (ms) - 50Hz
#define TELEMETRY_TASK_PERIOD_MS 1000   // Telemetry period (ms)

// --- Features & debug toggles -------------------------------------------
#define HAVE_CHG_DIRECTION  1       // 1 = enable change-direction button
#define ENABLE_WIFI_MQTT    1       // 1 = enable WiFi + MQTT
#define ENABLE_DEBUG_LEDS   1

#define DEBUG_PID_VERBOSE   0
#define DEBUG_IMU_VERBOSE   0
#define DEBUG_ENCODER_VERBOSE 0

// ---------------------------------------------------------------------------
// END USER EDITABLE SECTION
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Implementation details and compatibility shims below. You usually do not
// need to edit anything below this line.
// ---------------------------------------------------------------------------

// Encoder timeout (how long without ticks before considered stopped)
#define RPM_TIMEOUT_US      500000  // microseconds

// Default task priorities and stacks (FreeRTOS conventions)
#define WIFI_TASK_PRIORITY      (tskIDLE_PRIORITY + 3)
#define PID_TASK_PRIORITY       (tskIDLE_PRIORITY + 2)
#define TELEMETRY_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

#define PID_TASK_STACK_SIZE     2048
#define TELEMETRY_TASK_STACK_SIZE 2048
#define WIFI_TASK_STACK_SIZE    2048

// Transition timing for state machine
#define TRANSITION_RAMP_DOWN_TICKS  25
#define TRANSITION_STOP_HOLD_TICKS  40
#define TRANSITION_RAMP_UP_TICKS    65

// Telemetry/diagnostics intervals
#define PID_DEBUG_INTERVAL  50
#define TELEMETRY_DEBUG_INTERVAL 1

// Magnetometer calibration defaults (override if you calibrate)
#define MAG_X_OFFSET        0.0f
#define MAG_Y_OFFSET        0.0f
#define MAG_Z_OFFSET        0.0f
#define MAG_X_SCALE         1.0f
#define MAG_Y_SCALE         1.0f
#define MAG_Z_SCALE         1.0f

#endif // ROBOT_CONFIG_H
