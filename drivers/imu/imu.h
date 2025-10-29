// ===============================================
//  Module: IMU (LSM303DLHC) Driver
//  Description: I2C driver for accelerometer + magnetometer,
//               provides raw vectors and a filtered heading.
// ===============================================
#ifndef IMU_H
#define IMU_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ---- Public configuration ----
#define IMU_I2C           i2c0
#define IMU_I2C_SDA       4         // GP4  (adjust if wired differently)
#define IMU_I2C_SCL       5         // GP5
#define IMU_I2C_BAUD_HZ   400000

// Magnetic declination in radians (set for your location; 0 for demo)
#define IMU_DECLINATION_RAD  0.0f

// Filter alpha in [0..1]; lower is smoother
#define IMU_HEADING_EMA_ALPHA  0.15f

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
