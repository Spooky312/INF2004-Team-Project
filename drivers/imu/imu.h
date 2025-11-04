// ===============================================
//  Module: IMU (LSM303DLHC) Driver
//  Description: I2C driver for accelerometer + magnetometer,
//               provides raw vectors and a filtered heading.
// ===============================================
#ifndef IMU_H
#define IMU_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// NOTE: Pin and parameter configuration moved to robot_config.h
// Include that file in your .c files for:
// - IMU_I2C_INST, IMU_I2C_SDA, IMU_I2C_SCL
// - IMU_I2C_BAUD_HZ
// - IMU_EMA_ALPHA

// Magnetic declination in radians (set for your location; 0 for demo)
#define IMU_DECLINATION_RAD  0.0f

// ---- Public API ----
void  imu_init(void);

// Raw sensor reads (LSB units converted to g for accel, gauss for mag)
bool  imu_read_accel(float *ax_g, float *ay_g, float *az_g);
bool  imu_read_mag(float *mx_g, float *my_g, float *mz_g);

// Heading (degrees). Raw = instantaneous from mag; Filtered = EMA-smoothed.
bool  imu_get_heading_deg(float *heading_raw_deg, float *heading_filt_deg);

// Optional: get last computed pitch/roll (degrees) from accel
void  imu_get_pitch_roll_deg(float *pitch_deg, float *roll_deg);

#endif
