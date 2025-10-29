/**
 * @file imu.h
 * @brief IMU driver header for LSM303DLHC
 * Based on user-provided imu.h
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* ============================================================================
 * I2C PINS AND PORT (PROJECT-SPECIFIC)
 * ============================================================================ */
// NOTE: These pins are for the MPU-9250 on the RoboPico board
// The RoboPico schematic shows I2C0 is on GP4 (SDA) and GP5 (SCL)
#define IMU_I2C_PORT    i2c0
#define IMU_SDA_PIN     4  
#define IMU_SCL_PIN     5  
#define IMU_I2C_FREQ    (400 * 1000) // 400kHz Fast Mode

/* ============================================================================
 * I2C ADDRESSES (from user imu.h)
 * ============================================================================ */
#define IMU_ACCEL_ADDR       0x19  /* 7-bit I2C address for accelerometer */
#define IMU_MAG_ADDR         0x1E  /* 7-bit I2C address for magnetometer */

/* ============================================================================
 * ACCELEROMETER REGISTERS (from user imu.h)
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
#define IMU_ZYXDA            0x08  /* Bit 3: X, Y, Z data available flag */
#define IMU_AUTO_INCREMENT   0x80  /* MSB = 1 enables auto-increment */
/* ... (other accel defines from user imu.h) ... */
#define IMU_ODR_50HZ         0x40  /* Output Data Rate = 50 Hz */
#define IMU_NORMAL_MODE      0x00  /* Normal power mode */
#define IMU_XYZ_ENABLE       0x07  /* Enable all three axes */
#define IMU_FS_2G            0x00  /* Full scale = ±2g */
#define IMU_FS_4G            0x10  /* Full scale = ±4g */
#define IMU_FS_8G            0x20  /* Full scale = ±8g */
#define IMU_FS_16G           0x30  /* Full scale = ±16g */
#define IMU_HR_ENABLE        0x08  /* High resolution mode */

/* ============================================================================
 * MAGNETOMETER REGISTERS (from user imu.h)
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
#define IMU_MAG_DRDY         0x01  /* Bit 0: Data ready flag */
/* ... (other mag defines from user imu.h) ... */
#define IMU_MAG_ODR_15HZ     0x10  /* Magnetometer data rate = 15 Hz */
#define IMU_MAG_CONTINUOUS   0x00  /* Continuous conversion mode */
#define IMU_MAG_GAIN_1_3     0x20  /* Gain = ±1.3 gauss range */
#define IMU_MAG_GAIN_1_9     0x40  /* Gain = ±1.9 gauss range */
#define IMU_MAG_GAIN_2_5     0x60  /* Gain = ±2.5 gauss range */
#define IMU_MAG_GAIN_4_0     0x80  /* Gain = ±4.0 gauss range */
#define IMU_MAG_GAIN_4_7     0xA0  /* Gain = ±4.7 gauss range */
#define IMU_MAG_GAIN_5_6     0xC0  /* Gain = ±5.6 gauss range */
#define IMU_MAG_GAIN_8_1     0xE0  /* Gain = ±8.1 gauss range */


/* ============================================================================
 * ENUMS AND STRUCTS (from user imu.h)
 * ============================================================================ */

typedef enum
{
    IMU_SCALE_2G  = 0,  /* ±2g range */
    IMU_SCALE_4G  = 1,  /* ±4g range */
    IMU_SCALE_8G  = 2,  /* ±8g range */
    IMU_SCALE_16G = 3   /* ±16g range */
} imu_scale_t;

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

typedef struct
{
    int16_t x;  /* X-axis raw value */
    int16_t y;  /* Y-axis raw value */
    int16_t z;  /* Z-axis raw value */
} imu_accel_raw_t;

typedef struct
{
    int16_t x;  /* X-axis raw magnetic field value */
    int16_t y;  /* Y-axis raw magnetic field value */
    int16_t z;  /* Z-axis raw magnetic field value */
} imu_mag_raw_t;

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
 * HIGH-LEVEL API FOR DEMO 1
 * ============================================================================ */

/**
 * @brief Holds all processed data from the IMU
 */
typedef struct {
    imu_accel_raw_t accel;
    imu_mag_raw_t mag;
    float heading_deg; // Calculated heading
} imu_data_t;


/**
 * @brief High-level init. Call this from main.c
 * This will configure and initialize the IMU with project defaults.
 * @return true on success, false on failure
 */
bool imu_driver_init(void);

/**
 * @brief Fetches the latest raw data from both sensors.
 * @param p_data Pointer to the imu_data_t struct to fill.
 * @return true on success, false if data could not be read.
 */
bool imu_get_all_data(imu_data_t *p_data);

/**
 * @brief Calculates the tilt-compensated heading.
 * (For Demo 1, we start with a simple atan2)
 *
 * @param p_data Pointer to the imu_data_t struct containing mag data.
 * @return float Heading in degrees (0-360).
 */
float imu_calculate_heading(imu_data_t *p_data);

/**
 * @brief Gets the filtered heading (stub for Demo 1).
 * @return float Filtered heading in degrees.
 */
float imu_get_filtered_heading(void);

#endif /* IMU_H */