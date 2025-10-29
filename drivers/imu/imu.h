/**
 * @file imu.h
 * @brief IMU driver header for LSM303DLHC
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h" // Ensures I2C types are known

/* ============================================================================
 * I2C PINS AND PORT (PROJECT-SPECIFIC)
 * ============================================================================ */
#define IMU_I2C_PORT    i2c0
#define IMU_SDA_PIN     4  // Verify this matches your wiring
#define IMU_SCL_PIN     5  // Verify this matches your wiring
#define IMU_I2C_FREQ    (400 * 1000) // 400kHz Fast Mode

/* ============================================================================
 * I2C ADDRESSES
 * ============================================================================ */
#define IMU_ACCEL_ADDR       0x19
#define IMU_MAG_ADDR         0x1E

/* ============================================================================
 * ACCELEROMETER REGISTERS
 * ============================================================================ */
#define IMU_CTRL_REG1_A      0x20
#define IMU_CTRL_REG4_A      0x23
#define IMU_STATUS_REG_A     0x27
#define IMU_OUT_X_L_A        0x28
#define IMU_OUT_X_H_A        0x29
#define IMU_OUT_Y_L_A        0x2A
#define IMU_OUT_Y_H_A        0x2B
#define IMU_OUT_Z_L_A        0x2C
#define IMU_OUT_Z_H_A        0x2D
#define IMU_ODR_50HZ         0x40
#define IMU_NORMAL_MODE      0x00
#define IMU_XYZ_ENABLE       0x07
#define IMU_FS_2G            0x00
#define IMU_FS_4G            0x10
#define IMU_FS_8G            0x20
#define IMU_FS_16G           0x30
#define IMU_HR_ENABLE        0x08
#define IMU_ZYXDA            0x08
#define IMU_AUTO_INCREMENT   0x80

/* ============================================================================
 * MAGNETOMETER REGISTERS
 * ============================================================================ */
#define IMU_CRA_REG_M        0x00
#define IMU_CRB_REG_M        0x01
#define IMU_MR_REG_M         0x02
#define IMU_OUT_X_H_M        0x03
#define IMU_OUT_X_L_M        0x04
#define IMU_OUT_Z_H_M        0x05
#define IMU_OUT_Z_L_M        0x06
#define IMU_OUT_Y_H_M        0x07
#define IMU_OUT_Y_L_M        0x08
#define IMU_SR_REG_M         0x09
#define IMU_MAG_ODR_15HZ     0x10
#define IMU_MAG_CONTINUOUS   0x00
#define IMU_MAG_GAIN_1_3     0x20
#define IMU_MAG_GAIN_1_9     0x40
#define IMU_MAG_GAIN_2_5     0x60
#define IMU_MAG_GAIN_4_0     0x80
#define IMU_MAG_GAIN_4_7     0xA0
#define IMU_MAG_GAIN_5_6     0xC0
#define IMU_MAG_GAIN_8_1     0xE0
#define IMU_MAG_DRDY         0x01

/* ============================================================================
 * ENUMS AND STRUCTS
 * ============================================================================ */
typedef enum { IMU_SCALE_2G=0, IMU_SCALE_4G=1, IMU_SCALE_8G=2, IMU_SCALE_16G=3 } imu_scale_t;
typedef enum { IMU_MAG_GAIN_1_3_GAUSS=0, IMU_MAG_GAIN_1_9_GAUSS=1, IMU_MAG_GAIN_2_5_GAUSS=2, IMU_MAG_GAIN_4_0_GAUSS=3, IMU_MAG_GAIN_4_7_GAUSS=4, IMU_MAG_GAIN_5_6_GAUSS=5, IMU_MAG_GAIN_8_1_GAUSS=6 } imu_mag_gain_t;
typedef struct { int16_t x, y, z; } imu_accel_raw_t;
typedef struct { int16_t x, y, z; } imu_mag_raw_t;
typedef struct { i2c_inst_t *i2c_port; uint8_t sda_pin; uint8_t scl_pin; uint32_t i2c_freq; imu_scale_t scale; imu_mag_gain_t mag_gain; } imu_config_t;

/* ============================================================================
 * FUNCTION PROTOTYPES (Ensure all these are present)
 * ============================================================================ */
// Accelerometer Low-Level
bool imu_init(imu_config_t const * const p_config);
bool imu_read_accel_raw(imu_config_t const * const p_config, imu_accel_raw_t * const p_data);
bool imu_data_available(imu_config_t const * const p_config);
bool imu_write_register(imu_config_t const * const p_config, uint8_t reg, uint8_t value);
bool imu_read_register(imu_config_t const * const p_config, uint8_t reg, uint8_t * const p_value);

// Magnetometer Low-Level *** Ensure these prototypes exist ***
bool imu_mag_init(imu_config_t const * const p_config);
bool imu_mag_write_register(imu_config_t const * const p_config, uint8_t reg, uint8_t value);
bool imu_mag_read_register(imu_config_t const * const p_config, uint8_t reg, uint8_t * const p_value);
bool imu_mag_data_available(imu_config_t const * const p_config);
bool imu_mag_read_raw(imu_config_t const * const p_config, imu_mag_raw_t * const p_data);

// High-Level API for Demo 1
typedef struct { imu_accel_raw_t accel; imu_mag_raw_t mag; float heading_deg; } imu_data_t;
bool imu_driver_init(void);
bool imu_get_all_data(imu_data_t *p_data);
float imu_calculate_heading(imu_data_t *p_data);
float imu_get_filtered_heading(void);

#endif /* IMU_H */