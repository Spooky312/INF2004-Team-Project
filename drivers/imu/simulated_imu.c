/**
 * @file simulated_imu.c
 * @brief Simulated IMU Implementation
 */

#include "simulated_imu.h"
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

// Simulation parameters
#define TURN_RATE_DEGPS     90.0f   // Degrees per second when turning
#define TURN_TOLERANCE_DEG  5.0f    // Tolerance for turn completion

// IMU state
static imu_data_t current_imu;
static imu_robot_state_t robot_state = IMU_STATE_IDLE;
static uint32_t last_update_ms = 0;
static uint32_t state_entry_time_ms = 0;

// Latest sensor data
static line_data_t last_line_data;
static barcode_result_t last_barcode;
static bool line_data_valid = false;
static bool barcode_pending = false;

void simulated_imu_init(void) {
    current_imu.yaw_deg = 0.0f;
    current_imu.yaw_rate_degps = 0.0f;
    current_imu.accel_x = 0.0f;
    current_imu.accel_y = 0.0f;
    current_imu.target_yaw_deg = 0.0f;
    current_imu.is_turning = false;
    current_imu.timestamp_ms = to_ms_since_boot(get_absolute_time());
    
    robot_state = IMU_STATE_IDLE;
    last_update_ms = current_imu.timestamp_ms;
    state_entry_time_ms = current_imu.timestamp_ms;
    
    line_data_valid = false;
    barcode_pending = false;
    
    printf("[SIM_IMU] Initialized\n");
}

void simulated_imu_process_barcode(const barcode_result_t *barcode) {
    if (!barcode || !barcode->valid) return;
    
    last_barcode = *barcode;
    barcode_pending = true;
    
    printf("[SIM_IMU] Barcode received: %s -> %s\n", 
           barcode->decoded_string,
           barcode_command_to_string(barcode->command));
}

void simulated_imu_process_line(const line_data_t *line) {
    if (!line) return;
    
    last_line_data = *line;
    line_data_valid = true;
}

// Normalize angle to 0-360 range
static float normalize_angle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Calculate angle difference (shortest path)
static float angle_difference(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// State transition
static void transition_state(imu_robot_state_t new_state) {
    if (new_state == robot_state) return;
    
    printf("[SIM_IMU] State: %d -> %d\n", robot_state, new_state);
    robot_state = new_state;
    state_entry_time_ms = to_ms_since_boot(get_absolute_time());
    
    // State entry actions
    switch (new_state) {
        case IMU_STATE_LINE_FOLLOWING:
            current_imu.is_turning = false;
            current_imu.yaw_rate_degps = 0.0f;
            break;
            
        case IMU_STATE_TURNING_LEFT:
            current_imu.target_yaw_deg = normalize_angle(current_imu.yaw_deg - 90.0f);
            current_imu.is_turning = true;
            printf("[SIM_IMU] Turning LEFT: %.1f -> %.1f degrees\n", 
                   current_imu.yaw_deg, current_imu.target_yaw_deg);
            break;
            
        case IMU_STATE_TURNING_RIGHT:
            current_imu.target_yaw_deg = normalize_angle(current_imu.yaw_deg + 90.0f);
            current_imu.is_turning = true;
            printf("[SIM_IMU] Turning RIGHT: %.1f -> %.1f degrees\n", 
                   current_imu.yaw_deg, current_imu.target_yaw_deg);
            break;
            
        case IMU_STATE_U_TURN:
            current_imu.target_yaw_deg = normalize_angle(current_imu.yaw_deg + 180.0f);
            current_imu.is_turning = true;
            printf("[SIM_IMU] U-TURN: %.1f -> %.1f degrees\n", 
                   current_imu.yaw_deg, current_imu.target_yaw_deg);
            break;
            
        case IMU_STATE_STOPPED:
            current_imu.is_turning = false;
            current_imu.yaw_rate_degps = 0.0f;
            printf("[SIM_IMU] STOPPED\n");
            break;
            
        default:
            break;
    }
}

void simulated_imu_update(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    float dt = (now_ms - last_update_ms) / 1000.0f; // seconds
    
    if (dt <= 0.0f) return;
    
    last_update_ms = now_ms;
    current_imu.timestamp_ms = now_ms;
    
    // State machine
    switch (robot_state) {
        case IMU_STATE_IDLE:
            // Automatically start line following
            transition_state(IMU_STATE_LINE_FOLLOWING);
            break;
            
        case IMU_STATE_LINE_FOLLOWING:
            // Simulate slight heading changes based on line position
            if (line_data_valid) {
                // Line position affects yaw rate
                float line_correction = last_line_data.position * 0.01f; // deg/s
                current_imu.yaw_rate_degps = line_correction;
                current_imu.yaw_deg = normalize_angle(current_imu.yaw_deg + 
                                                      current_imu.yaw_rate_degps * dt);
            }
            
            // Check for barcode command
            if (barcode_pending) {
                barcode_pending = false;
                
                switch (last_barcode.command) {
                    case BARCODE_CMD_LEFT:
                        transition_state(IMU_STATE_TURNING_LEFT);
                        break;
                    case BARCODE_CMD_RIGHT:
                        transition_state(IMU_STATE_TURNING_RIGHT);
                        break;
                    case BARCODE_CMD_UTURN:
                        transition_state(IMU_STATE_U_TURN);
                        break;
                    case BARCODE_CMD_STOP:
                        transition_state(IMU_STATE_STOPPED);
                        break;
                    default:
                        printf("[SIM_IMU] Unknown barcode command\n");
                        break;
                }
            }
            break;
            
        case IMU_STATE_TURNING_LEFT:
        case IMU_STATE_TURNING_RIGHT:
        case IMU_STATE_U_TURN:
            // Simulate turning motion
            {
                float error = angle_difference(current_imu.target_yaw_deg, 
                                               current_imu.yaw_deg);
                
                if (fabsf(error) < TURN_TOLERANCE_DEG) {
                    // Turn complete
                    current_imu.yaw_deg = current_imu.target_yaw_deg;
                    current_imu.yaw_rate_degps = 0.0f;
                    printf("[SIM_IMU] Turn complete at %.1f degrees\n", 
                           current_imu.yaw_deg);
                    transition_state(IMU_STATE_LINE_FOLLOWING);
                } else {
                    // Continue turning
                    float turn_direction = (error > 0) ? 1.0f : -1.0f;
                    current_imu.yaw_rate_degps = turn_direction * TURN_RATE_DEGPS;
                    current_imu.yaw_deg = normalize_angle(current_imu.yaw_deg + 
                                                          current_imu.yaw_rate_degps * dt);
                }
            }
            break;
            
        case IMU_STATE_STOPPED:
            // Do nothing
            current_imu.yaw_rate_degps = 0.0f;
            break;
    }
    
    // Simulate accelerometer (very basic)
    // When turning, there's lateral acceleration
    if (current_imu.is_turning) {
        current_imu.accel_x = 0.0f;
        current_imu.accel_y = current_imu.yaw_rate_degps * 0.01f; // Simplified
    } else {
        current_imu.accel_x = 0.2f; // Moving forward
        current_imu.accel_y = 0.0f;
    }
}

void simulated_imu_get_data(imu_data_t *data) {
    if (data) {
        *data = current_imu;
    }
}

imu_robot_state_t simulated_imu_get_state(void) {
    return robot_state;
}

void simulated_imu_reset(void) {
    simulated_imu_init();
}

void simulated_imu_print_status(void) {
    printf("[SIM_IMU] State=%d, Yaw=%.1f°, Rate=%.1f°/s, Target=%.1f°, Turning=%d\n",
           robot_state,
           current_imu.yaw_deg,
           current_imu.yaw_rate_degps,
           current_imu.target_yaw_deg,
           current_imu.is_turning);
    
    if (line_data_valid) {
        printf("         Line: ADC=%u, Pos=%d, Detected=%d\n",
               last_line_data.raw_adc,
               last_line_data.position,
               last_line_data.line_detected);
    }
}