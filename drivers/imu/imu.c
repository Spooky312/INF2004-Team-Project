/**
 * @file imu_driver.c
 * @brief IMU driver implementation for LSM303DLHC
 */

#include "imu.h"
#include "pico/stdlib.h"
#include <string.h>
#include <math.h> // For atan2
#include <stdio.h> // <-- ADDED THIS LINE

#define I2C_TIMEOUT_US  100000
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Global config object for the driver
static imu_config_t _imu_cfg;
static imu_data_t _imu_data_cache; // Cache for holding last good data

/* ============================================================================
 * ALL LOW-LEVEL ACCELEROMETER FUNCTIONS (Copy from your original imu.c)
 * ============================================================================ */

bool
imu_write_register(imu_config_t const * const p_config,
                   uint8_t const reg,
                   uint8_t const value)
{
    uint8_t buffer[2];
    int result;

    if (NULL == p_config || NULL == p_config->i2c_port) return false;

    buffer[0] = reg;
    buffer[1] = value;

    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_ACCEL_ADDR,
                                  buffer,
                                  2,
                                  false,
                                  I2C_TIMEOUT_US);
    return (result == 2);
}

bool
imu_read_register(imu_config_t const * const p_config,
                  uint8_t const reg,
                  uint8_t * const p_value)
{
    int result;

    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_value) return false;

    result = i2c_write_timeout_us(p_config->i2c_port, IMU_ACCEL_ADDR, &reg, 1, true, I2C_TIMEOUT_US);
    if (result != 1) return false;

    result = i2c_read_timeout_us(p_config->i2c_port, IMU_ACCEL_ADDR, p_value, 1, false, I2C_TIMEOUT_US);
    return (result == 1);
}

bool
imu_init(imu_config_t const * const p_config)
{
    uint8_t ctrl_reg1_val;
    uint8_t ctrl_reg4_val;
    bool success;

    if (NULL == p_config || NULL == p_config->i2c_port) return false;

    i2c_init(p_config->i2c_port, p_config->i2c_freq);
    gpio_set_function(p_config->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(p_config->scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(p_config->sda_pin);
    gpio_pull_up(p_config->scl_pin);

    sleep_ms(10);

    ctrl_reg1_val = IMU_ODR_50HZ | IMU_NORMAL_MODE | IMU_XYZ_ENABLE;
    success = imu_write_register(p_config, IMU_CTRL_REG1_A, ctrl_reg1_val);
    if (!success) return false;

    switch (p_config->scale)
    {
        case IMU_SCALE_2G:  ctrl_reg4_val = IMU_FS_2G;  break;
        case IMU_SCALE_4G:  ctrl_reg4_val = IMU_FS_4G;  break;
        case IMU_SCALE_8G:  ctrl_reg4_val = IMU_FS_8G;  break;
        case IMU_SCALE_16G: ctrl_reg4_val = IMU_FS_16G; break;
        default:            ctrl_reg4_val = IMU_FS_2G;  break;
    }
    ctrl_reg4_val |= IMU_HR_ENABLE;

    success = imu_write_register(p_config, IMU_CTRL_REG4_A, ctrl_reg4_val);
    if (!success) return false;

    /* NOW INITIALIZE MAGNETOMETER */
    success = imu_mag_init(p_config); // This call requires imu_mag_init prototype in imu.h

    return success;
}

bool
imu_data_available(imu_config_t const * const p_config)
{
    uint8_t status;
    bool success;

    if (NULL == p_config) return false;

    success = imu_read_register(p_config, IMU_STATUS_REG_A, &status);
    if (!success) return false;

    return ((status & IMU_ZYXDA) != 0);
}

bool
imu_read_accel_raw(imu_config_t const * const p_config,
                   imu_accel_raw_t * const p_data)
{
    uint8_t buffer[6];
    uint8_t reg_addr;
    int result;

    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_data) return false;

    reg_addr = IMU_OUT_X_L_A | IMU_AUTO_INCREMENT;

    result = i2c_write_timeout_us(p_config->i2c_port, IMU_ACCEL_ADDR, &reg_addr, 1, true, I2C_TIMEOUT_US);
    if (result != 1) return false;

    result = i2c_read_timeout_us(p_config->i2c_port, IMU_ACCEL_ADDR, buffer, 6, false, I2C_TIMEOUT_US);
    if (result != 6) return false;

    p_data->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    p_data->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    p_data->z = (int16_t)(buffer[4] | (buffer[5] << 8));

    return true;
}


/* ============================================================================
 * ALL LOW-LEVEL MAGNETOMETER FUNCTIONS (Copy from your original imu.c)
 * ============================================================================ */

bool
imu_mag_write_register(imu_config_t const * const p_config,
                       uint8_t const reg,
                       uint8_t const value)
{
    uint8_t buffer[2];
    int result;

    if (NULL == p_config || NULL == p_config->i2c_port) return false;

    buffer[0] = reg;
    buffer[1] = value;

    /* NOTE: Using IMU_MAG_ADDR instead of IMU_ACCEL_ADDR */
    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_MAG_ADDR,
                                  buffer,
                                  2,
                                  false,
                                  I2C_TIMEOUT_US);

    return (result == 2);
}


bool
imu_mag_read_register(imu_config_t const * const p_config,
                      uint8_t const reg,
                      uint8_t * const p_value)
{
    int result;

    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_value) return false;

    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_MAG_ADDR,
                                  &reg,
                                  1,
                                  true,
                                  I2C_TIMEOUT_US);

    if (result != 1) return false;

    result = i2c_read_timeout_us(p_config->i2c_port,
                                 IMU_MAG_ADDR,
                                 p_value,
                                 1,
                                 false,
                                 I2C_TIMEOUT_US);

    return (result == 1);
}


bool
imu_mag_init(imu_config_t const * const p_config)
{
    uint8_t cra_val;   /* Config Register A value */
    uint8_t crb_val;   /* Config Register B value */
    uint8_t mr_val;    /* Mode Register value */
    bool success;

    if (NULL == p_config) return false;

    /* STEP 1: Configure data rate in CRA register */
    cra_val = IMU_MAG_ODR_15HZ;  /* 15 Hz update rate */
    success = imu_mag_write_register(p_config, IMU_CRA_REG_M, cra_val);
    if (!success) return false;

    /* STEP 2: Configure gain/range in CRB register */
    switch (p_config->mag_gain)
    {
        case IMU_MAG_GAIN_1_3_GAUSS: crb_val = IMU_MAG_GAIN_1_3; break;
        case IMU_MAG_GAIN_1_9_GAUSS: crb_val = IMU_MAG_GAIN_1_9; break;
        case IMU_MAG_GAIN_2_5_GAUSS: crb_val = IMU_MAG_GAIN_2_5; break;
        case IMU_MAG_GAIN_4_0_GAUSS: crb_val = IMU_MAG_GAIN_4_0; break;
        case IMU_MAG_GAIN_4_7_GAUSS: crb_val = IMU_MAG_GAIN_4_7; break;
        case IMU_MAG_GAIN_5_6_GAUSS: crb_val = IMU_MAG_GAIN_5_6; break;
        case IMU_MAG_GAIN_8_1_GAUSS: crb_val = IMU_MAG_GAIN_8_1; break;
        default:                     crb_val = IMU_MAG_GAIN_1_3; break;
    }
    success = imu_mag_write_register(p_config, IMU_CRB_REG_M, crb_val);
    if (!success) return false;

    /* STEP 3: Set continuous conversion mode */
    mr_val = IMU_MAG_CONTINUOUS;  /* Keep reading automatically */
    success = imu_mag_write_register(p_config, IMU_MR_REG_M, mr_val);

    return success;
}

bool
imu_mag_data_available(imu_config_t const * const p_config)
{
    uint8_t status;
    bool success;

    if (NULL == p_config) return false;

    success = imu_mag_read_register(p_config,
                                    IMU_SR_REG_M,
                                    &status);

    if (!success) return false;

    return ((status & IMU_MAG_DRDY) != 0);
}


bool
imu_mag_read_raw(imu_config_t const * const p_config,
                 imu_mag_raw_t * const p_data)
{
    uint8_t buffer[6];
    uint8_t reg_addr;
    int result;

    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_data) return false;

    reg_addr = IMU_OUT_X_H_M;

    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_MAG_ADDR,
                                  &reg_addr,
                                  1,
                                  true,
                                  I2C_TIMEOUT_US);

    if (result != 1) return false;

    result = i2c_read_timeout_us(p_config->i2c_port,
                                 IMU_MAG_ADDR,
                                 buffer,
                                 6,
                                 false,
                                 I2C_TIMEOUT_US);

    if (result != 6) return false;

    p_data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    p_data->z = (int16_t)((buffer[2] << 8) | buffer[3]);
    p_data->y = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}


/* ============================================================================
 * NEW HIGH-LEVEL API FUNCTIONS
 * ============================================================================ */

bool imu_driver_init(void) {
    _imu_cfg.i2c_port = IMU_I2C_PORT;
    _imu_cfg.sda_pin = IMU_SDA_PIN;
    _imu_cfg.scl_pin = IMU_SCL_PIN;
    _imu_cfg.i2c_freq = IMU_I2C_FREQ;
    _imu_cfg.scale = IMU_SCALE_2G;
    _imu_cfg.mag_gain = IMU_MAG_GAIN_1_3_GAUSS;

    return imu_init(&_imu_cfg);
}

bool imu_get_all_data(imu_data_t *p_data) {
    if (NULL == p_data) return false;

    if (!imu_read_accel_raw(&_imu_cfg, &p_data->accel)) {
        printf("Failed to read accel\n"); // Now printf is valid
        return false;
    }

    if (!imu_mag_read_raw(&_imu_cfg, &p_data->mag)) {
        printf("Failed to read mag\n"); // Now printf is valid
        return false;
    }

    p_data->heading_deg = imu_calculate_heading(p_data);
    return true;
}

float imu_calculate_heading(imu_data_t *p_data) {
    // Basic atan2 calculation, does not account for tilt
    float mag_x = (float)p_data->mag.x;
    float mag_y = (float)p_data->mag.y;

    // TODO: Add magnetometer calibration offsets if needed
    // mag_x -= X_OFFSET;
    // mag_y -= Y_OFFSET;

    float heading_rad = atan2(mag_y, mag_x);
    float heading_deg = heading_rad * 180.0 / M_PI;

    // Normalize to 0-360 degrees
    if (heading_deg < 0) {
        heading_deg += 360.0;
    }
    // You might need to add an offset depending on sensor orientation
    // heading_deg += DECLINATION_OFFSET;
    // if (heading_deg >= 360.0) heading_deg -= 360.0;

    return heading_deg;
}

float imu_get_filtered_heading(void) {
    // This function is called from within a mutex in main.c,
    // so it's generally safe to update the cache here.
    if (imu_get_all_data(&_imu_data_cache)) {
        // For Demo 1, "filtered" is just the raw calculated heading.
        // For Week 10 demo, implement a Complementary or Kalman filter here
        // using _imu_data_cache.accel data for tilt compensation.
        return _imu_data_cache.heading_deg;
    }

    // Return last known heading on failure
    return _imu_data_cache.heading_deg;
}