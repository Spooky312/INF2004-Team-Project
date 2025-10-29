/**
 * @file simulated_imu.h
 * @brief Simulated IMU for Demo 2 Testing
 * 
 * Receives barcode and line sensor data, simulates robot behavior
 */

#ifndef SIMULATED_IMU_H
#define SIMULATED_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "barcode_decoder.h"
#include "line_detection.h"

// IMU data structure
typedef struct {
    float yaw_deg;              // Current heading (0-360)
    float yaw_rate_degps;       // Angular velocity
    float accel_x;              // X acceleration (m/s²)
    float accel_y;              // Y acceleration
    float target_yaw_deg;       // Target heading for turns
    bool is_turning;            // Currently executing turn
    uint32_t timestamp_ms;
} imu_data_t;

// Robot state tracked by simulated IMU
typedef enum {
    IMU_STATE_IDLE = 0,
    IMU_STATE_LINE_FOLLOWING,
    IMU_STATE_TURNING_LEFT,
    IMU_STATE_TURNING_RIGHT,
    IMU_STATE_U_TURN,
    IMU_STATE_STOPPED
} imu_robot_state_t;

/**
 * @brief Initialize simulated IMU
 */
void simulated_imu_init(void);

/**
 * @brief Update IMU with barcode data
 * @param barcode Barcode result from decoder
 */
void simulated_imu_process_barcode(const barcode_result_t *barcode);

/**
 * @brief Update IMU with line sensor data
 * @param line Line sensor data
 */
void simulated_imu_process_line(const line_data_t *line);

/**
 * @brief Update IMU state (call periodically in task)
 * Simulates robot motion and updates heading
 */
void simulated_imu_update(void);

/**
 * @brief Get current IMU data
 * @param data Pointer to store IMU data
 */
void simulated_imu_get_data(imu_data_t *data);

/**
 * @brief Get current robot state
 * @return Current state
 */
imu_robot_state_t simulated_imu_get_state(void);

/**
 * @brief Reset IMU to initial state
 */
void simulated_imu_reset(void);

/**
 * @brief Print IMU status (for debugging)
 */
void simulated_imu_print_status(void);

#endif // SIMULATED_IMU_H