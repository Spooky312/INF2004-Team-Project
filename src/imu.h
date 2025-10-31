/**
 * @file imu.h
 * @brief IMU driver header for accelerometer and magnetometer functionality
 * @author Muhammad Ridwan Putra Jasni
 * @date 2/10/2025
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* ============================================================================
 * I2C ADDRESSES
 * ============================================================================
 * The LSM303DLHC has TWO separate I2C addresses - one for each sensor
 */
#define IMU_ACCEL_ADDR       0x19  /* 7-bit I2C address for accelerometer */
#define IMU_MAG_ADDR         0x1E  /* 7-bit I2C address for magnetometer */

/* ============================================================================
 * ACCELEROMETER REGISTER ADDRESSES
 * ============================================================================ */
#define IMU_CTRL_REG1_A      0x20  /* Control register 1 - sets power mode, data rate */
#define IMU_CTRL_REG4_A      0x23  /* Control register 4 - sets full-scale range */
#define IMU_STATUS_REG_A     0x27  /* Status register - check if data is ready */
#define IMU_OUT_X_L_A        0x28  /* X-axis LOW byte output register */
#define IMU_OUT_X_H_A        0x29  /* X-axis HIGH byte output register */
#define IMU_OUT_Y_L_A        0x2A  /* Y-axis LOW byte output register */
#define IMU_OUT_Y_H_A        0x2B  /* Y-axis HIGH byte output register */
#define IMU_OUT_Z_L_A        0x2C  /* Z-axis LOW byte output register */
#define IMU_OUT_Z_H_A        0x2D  /* Z-axis HIGH byte output register */

/* ============================================================================
 * MAGNETOMETER REGISTER ADDRESSES
 * ============================================================================ */
#define IMU_CRA_REG_M        0x00  /* Config register A - sets data rate */
#define IMU_CRB_REG_M        0x01  /* Config register B - sets gain/range */
#define IMU_MR_REG_M         0x02  /* Mode register - sets operating mode */
#define IMU_OUT_X_H_M        0x03  /* X-axis HIGH byte (note: mag is H then L!) */
#define IMU_OUT_X_L_M        0x04  /* X-axis LOW byte */
#define IMU_OUT_Z_H_M        0x05  /* Z-axis HIGH byte */
#define IMU_OUT_Z_L_M        0x06  /* Z-axis LOW byte */
#define IMU_OUT_Y_H_M        0x07  /* Y-axis HIGH byte */
#define IMU_OUT_Y_L_M        0x08  /* Y-axis LOW byte */
#define IMU_SR_REG_M         0x09  /* Status register - check if data ready */

/* ============================================================================
 * ACCELEROMETER CONTROL REGISTER BIT DEFINITIONS
 * ============================================================================ */
#define IMU_ODR_50HZ         0x40  /* Output Data Rate = 50 Hz */
#define IMU_NORMAL_MODE      0x00  /* Normal power mode */
#define IMU_XYZ_ENABLE       0x07  /* Enable all three axes */
#define IMU_FS_2G            0x00  /* Full scale = ±2g */
#define IMU_FS_4G            0x10  /* Full scale = ±4g */
#define IMU_FS_8G            0x20  /* Full scale = ±8g */
#define IMU_FS_16G           0x30  /* Full scale = ±16g */
#define IMU_HR_ENABLE        0x08  /* High resolution mode */
#define IMU_ZYXDA            0x08  /* Bit 3: X, Y, Z data available flag */
#define IMU_AUTO_INCREMENT   0x80  /* MSB = 1 enables auto-increment */

/* ============================================================================
 * MAGNETOMETER CONTROL REGISTER BIT DEFINITIONS
 * ============================================================================ */
#define IMU_MAG_ODR_15HZ     0x10  /* Magnetometer data rate = 15 Hz */
#define IMU_MAG_CONTINUOUS   0x00  /* Continuous conversion mode */
#define IMU_MAG_GAIN_1_3     0x20  /* Gain = ±1.3 gauss range */
#define IMU_MAG_GAIN_1_9     0x40  /* Gain = ±1.9 gauss range */
#define IMU_MAG_GAIN_2_5     0x60  /* Gain = ±2.5 gauss range */
#define IMU_MAG_GAIN_4_0     0x80  /* Gain = ±4.0 gauss range */
#define IMU_MAG_GAIN_4_7     0xA0  /* Gain = ±4.7 gauss range */
#define IMU_MAG_GAIN_5_6     0xC0  /* Gain = ±5.6 gauss range */
#define IMU_MAG_GAIN_8_1     0xE0  /* Gain = ±8.1 gauss range */
#define IMU_MAG_DRDY         0x01  /* Bit 0: Data ready flag */

/**
 * @brief Accelerometer full-scale range enumeration
 */
typedef enum
{
    IMU_SCALE_2G  = 0,  /* ±2g range */
    IMU_SCALE_4G  = 1,  /* ±4g range */
    IMU_SCALE_8G  = 2,  /* ±8g range */
    IMU_SCALE_16G = 3   /* ±16g range */
} imu_scale_t;

/**
 * @brief Magnetometer gain/range enumeration
 */
typedef enum
{
    IMU_MAG_GAIN_1_3_GAUSS = 0,  /* ±1.3 gauss - good for most environments */
    IMU_MAG_GAIN_1_9_GAUSS = 1,  /* ±1.9 gauss */
    IMU_MAG_GAIN_2_5_GAUSS = 2,  /* ±2.5 gauss */
    IMU_MAG_GAIN_4_0_GAUSS = 3,  /* ±4.0 gauss */
    IMU_MAG_GAIN_4_7_GAUSS = 4,  /* ±4.7 gauss */
    IMU_MAG_GAIN_5_6_GAUSS = 5,  /* ±5.6 gauss */
    IMU_MAG_GAIN_8_1_GAUSS = 6   /* ±8.1 gauss - for high magnetic fields */
} imu_mag_gain_t;

/**
 * @brief Raw accelerometer data structure
 */
typedef struct
{
    int16_t x;  /* X-axis raw value */
    int16_t y;  /* Y-axis raw value */
    int16_t z;  /* Z-axis raw value */
} imu_accel_raw_t;

/**
 * @brief Raw magnetometer data structure
 * 
 * These are the raw magnetic field readings.
 * Positive X typically points to magnetic North (when flat)
 * Positive Y points East
 * Positive Z points down
 */
typedef struct
{
    int16_t x;  /* X-axis raw magnetic field value */
    int16_t y;  /* Y-axis raw magnetic field value */
    int16_t z;  /* Z-axis raw magnetic field value */
} imu_mag_raw_t;

/**
 * @brief IMU configuration structure
 */
typedef struct
{
    i2c_inst_t *i2c_port;      /* I2C instance (use i2c0 or i2c1) */
    uint8_t sda_pin;           /* GPIO pin for I2C data line (SDA) */
    uint8_t scl_pin;           /* GPIO pin for I2C clock line (SCL) */
    uint32_t i2c_freq;         /* I2C bus frequency in Hz */
    imu_scale_t scale;         /* Accelerometer range */
    imu_mag_gain_t mag_gain;   /* Magnetometer gain/range */
} imu_config_t;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

/**
 * @brief Initialize the IMU (accelerometer and magnetometer)
 */
bool imu_init(imu_config_t const * const p_config);

/**
 * @brief Read raw accelerometer data
 */
bool imu_read_accel_raw(imu_config_t const * const p_config,
                        imu_accel_raw_t * const p_data);

/**
 * @brief Check if new accelerometer data is available
 */
bool imu_data_available(imu_config_t const * const p_config);

/**
 * @brief Write to an IMU accelerometer register
 */
bool imu_write_register(imu_config_t const * const p_config,
                        uint8_t reg,
                        uint8_t value);

/**
 * @brief Read from an IMU accelerometer register
 */
bool imu_read_register(imu_config_t const * const p_config,
                       uint8_t reg,
                       uint8_t * const p_value);

/**
 * @brief Write to magnetometer register
 */
bool imu_mag_write_register(imu_config_t const * const p_config,
                            uint8_t reg,
                            uint8_t value);

/**
 * @brief Read from magnetometer register
 */
bool imu_mag_read_register(imu_config_t const * const p_config,
                           uint8_t reg,
                           uint8_t * const p_value);

/**
 * @brief Initialize the magnetometer
 */
bool imu_mag_init(imu_config_t const * const p_config);

/**
 * @brief Check if new magnetometer data is available
 */
bool imu_mag_data_available(imu_config_t const * const p_config);

/**
 * @brief Read raw magnetometer data
 */
bool imu_mag_read_raw(imu_config_t const * const p_config,
                      imu_mag_raw_t * const p_data);

#endif /* IMU_H */