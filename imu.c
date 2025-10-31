/**
 * @file imu.c
 * @brief IMU driver implementation for LSM303DLHC accelerometer and magnetometer
 * @author Muhammad Ridwan Putra
 * @date 2/10/2025
 */

#include "imu.h"
#include "pico/stdlib.h"
#include <string.h>
#include "config.h"

#define I2C_TIMEOUT_US  100000

/* ============================================================================
 * ACCELEROMETER FUNCTIONS (unchanged from original)
 * ============================================================================ */

bool
imu_write_register(imu_config_t const * const p_config,
                   uint8_t const reg,
                   uint8_t const value)
{
    uint8_t buffer[2];
    int result;
    
    if (NULL == p_config || NULL == p_config->i2c_port)
    {
        return false;
    }
    
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
    
    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_value)
    {
        return false;
    }
    
    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_ACCEL_ADDR,
                                  &reg,
                                  1,
                                  true,
                                  I2C_TIMEOUT_US);
    
    if (result != 1)
    {
        return false;
    }
    
    result = i2c_read_timeout_us(p_config->i2c_port,
                                 IMU_ACCEL_ADDR,
                                 p_value,
                                 1,
                                 false,
                                 I2C_TIMEOUT_US);
    
    return (result == 1);
}

bool
imu_init(imu_config_t const * const p_config)
{
    uint8_t ctrl_reg1_val;
    uint8_t ctrl_reg4_val;
    bool success;
    
    if (NULL == p_config || NULL == p_config->i2c_port)
    {
        return false;
    }
    
    i2c_init(p_config->i2c_port, p_config->i2c_freq);
    gpio_set_function(p_config->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(p_config->scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(p_config->sda_pin);
    gpio_pull_up(p_config->scl_pin);
    
    sleep_ms(10);
    
    ctrl_reg1_val = IMU_ODR_50HZ | 
                    IMU_NORMAL_MODE | 
                    IMU_XYZ_ENABLE;
    
    success = imu_write_register(p_config,
                                 IMU_CTRL_REG1_A,
                                 ctrl_reg1_val);
    
    if (!success)
    {
        return false;
    }
    
    switch (p_config->scale)
    {
        case IMU_SCALE_2G:
            ctrl_reg4_val = IMU_FS_2G;
            break;
        case IMU_SCALE_4G:
            ctrl_reg4_val = IMU_FS_4G;
            break;
        case IMU_SCALE_8G:
            ctrl_reg4_val = IMU_FS_8G;
            break;
        case IMU_SCALE_16G:
            ctrl_reg4_val = IMU_FS_16G;
            break;
        default:
            ctrl_reg4_val = IMU_FS_2G;
            break;
    }
    
    ctrl_reg4_val |= IMU_HR_ENABLE;
    
    success = imu_write_register(p_config,
                                 IMU_CTRL_REG4_A,
                                 ctrl_reg4_val);
    
    if (!success)
    {
        return false;
    }
    
    /* NOW INITIALIZE MAGNETOMETER */
    success = imu_mag_init(p_config);
    
    return success;
}

bool
imu_data_available(imu_config_t const * const p_config)
{
    uint8_t status;
    bool success;
    
    if (NULL == p_config)
    {
        return false;
    }
    
    success = imu_read_register(p_config,
                                IMU_STATUS_REG_A,
                                &status);
    
    if (!success)
    {
        return false;
    }
    
    return ((status & IMU_ZYXDA) != 0);
}

bool
imu_read_accel_raw(imu_config_t const * const p_config,
                   imu_accel_raw_t * const p_data)
{
    uint8_t buffer[6];
    uint8_t reg_addr;
    int result;
    
    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_data)
    {
        return false;
    }
    
    reg_addr = IMU_OUT_X_L_A | IMU_AUTO_INCREMENT;
    
    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_ACCEL_ADDR,
                                  &reg_addr,
                                  1,
                                  true,
                                  I2C_TIMEOUT_US);
    
    if (result != 1)
    {
        return false;
    }
    
    result = i2c_read_timeout_us(p_config->i2c_port,
                                 IMU_ACCEL_ADDR,
                                 buffer,
                                 6,
                                 false,
                                 I2C_TIMEOUT_US);
    
    if (result != 6)
    {
        return false;
    }
    
    p_data->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    p_data->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    p_data->z = (int16_t)(buffer[4] | (buffer[5] << 8));
    
    return true;
}

/* ============================================================================
 * MAGNETOMETER FUNCTIONS (NEW!)
 * ============================================================================ */

/**
 * @brief Write to magnetometer register
 * 
 * IMPORTANT DIFFERENCE FROM ACCELEROMETER:
 * The magnetometer has a DIFFERENT I2C address (0x1E instead of 0x19)
 * Everything else works the same way
 */
bool
imu_mag_write_register(imu_config_t const * const p_config,
                       uint8_t const reg,
                       uint8_t const value)
{
    uint8_t buffer[2];
    int result;
    
    if (NULL == p_config || NULL == p_config->i2c_port)
    {
        return false;
    }
    
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

/**
 * @brief Read from magnetometer register
 */
bool
imu_mag_read_register(imu_config_t const * const p_config,
                      uint8_t const reg,
                      uint8_t * const p_value)
{
    int result;
    
    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_value)
    {
        return false;
    }
    
    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_MAG_ADDR,
                                  &reg,
                                  1,
                                  true,
                                  I2C_TIMEOUT_US);
    
    if (result != 1)
    {
        return false;
    }
    
    result = i2c_read_timeout_us(p_config->i2c_port,
                                 IMU_MAG_ADDR,
                                 p_value,
                                 1,
                                 false,
                                 I2C_TIMEOUT_US);
    
    return (result == 1);
}

/**
 * @brief Initialize magnetometer
 * 
 * WHAT THIS DOES:
 * 1. Sets output data rate (how fast sensor updates)
 * 2. Sets gain/range (sensitivity to magnetic fields)
 * 3. Sets continuous mode (keeps reading automatically)
 * 
 * WHY THESE SETTINGS?
 * - 15 Hz data rate: fast enough for compass, not too power hungry
 * - Gain 1.3: good for Earth's magnetic field (~0.25 to 0.65 gauss)
 * - Continuous mode: no need to trigger each reading
 */
bool
imu_mag_init(imu_config_t const * const p_config)
{
    uint8_t cra_val;   /* Config Register A value */
    uint8_t crb_val;   /* Config Register B value */
    uint8_t mr_val;    /* Mode Register value */
    bool success;
    
    if (NULL == p_config)
    {
        return false;
    }
    
    /* STEP 1: Configure data rate in CRA register */
    cra_val = IMU_MAG_ODR_15HZ;  /* 15 Hz update rate */
    success = imu_mag_write_register(p_config,
                                     IMU_CRA_REG_M,
                                     cra_val);
    
    if (!success)
    {
        return false;
    }
    
    /* STEP 2: Configure gain/range in CRB register */
    /* Convert enum to register value */
    switch (p_config->mag_gain)
    {
        case IMU_MAG_GAIN_1_3_GAUSS:
            crb_val = IMU_MAG_GAIN_1_3;
            break;
        case IMU_MAG_GAIN_1_9_GAUSS:
            crb_val = IMU_MAG_GAIN_1_9;
            break;
        case IMU_MAG_GAIN_2_5_GAUSS:
            crb_val = IMU_MAG_GAIN_2_5;
            break;
        case IMU_MAG_GAIN_4_0_GAUSS:
            crb_val = IMU_MAG_GAIN_4_0;
            break;
        case IMU_MAG_GAIN_4_7_GAUSS:
            crb_val = IMU_MAG_GAIN_4_7;
            break;
        case IMU_MAG_GAIN_5_6_GAUSS:
            crb_val = IMU_MAG_GAIN_5_6;
            break;
        case IMU_MAG_GAIN_8_1_GAUSS:
            crb_val = IMU_MAG_GAIN_8_1;
            break;
        default:
            crb_val = IMU_MAG_GAIN_1_3;  /* Default to 1.3 gauss */
            break;
    }
    
    success = imu_mag_write_register(p_config,
                                     IMU_CRB_REG_M,
                                     crb_val);
    
    if (!success)
    {
        return false;
    }
    
    /* STEP 3: Set continuous conversion mode */
    mr_val = IMU_MAG_CONTINUOUS;  /* Keep reading automatically */
    success = imu_mag_write_register(p_config,
                                     IMU_MR_REG_M,
                                     mr_val);
    
    return success;
}

/**
 * @brief Check if new magnetometer data is available
 * 
 * Reads the status register and checks the DRDY (Data Ready) bit
 */
bool
imu_mag_data_available(imu_config_t const * const p_config)
{
    uint8_t status;
    bool success;
    
    if (NULL == p_config)
    {
        return false;
    }
    
    success = imu_mag_read_register(p_config,
                                    IMU_SR_REG_M,
                                    &status);
    
    if (!success)
    {
        return false;
    }
    
    return ((status & IMU_MAG_DRDY) != 0);
}

/**
 * @brief Read raw magnetometer data
 * 
 * IMPORTANT DIFFERENCE FROM ACCELEROMETER:
 * Magnetometer stores data as HIGH byte first, then LOW byte!
 * Also, the axis order is X, Z, Y (not X, Y, Z)
 * 
 * Register layout:
 * 0x03: OUT_X_H (X high byte)
 * 0x04: OUT_X_L (X low byte)
 * 0x05: OUT_Z_H (Z high byte)
 * 0x06: OUT_Z_L (Z low byte)
 * 0x07: OUT_Y_H (Y high byte)
 * 0x08: OUT_Y_L (Y low byte)
 * 
 * WHY IS ORDER DIFFERENT?
 * STMicroelectronics designed it this way. Nobody knows why!
 * We just have to deal with it.
 */
bool
imu_mag_read_raw(imu_config_t const * const p_config,
                 imu_mag_raw_t * const p_data)
{
    uint8_t buffer[6];
    uint8_t reg_addr;
    int result;
    
    if (NULL == p_config || NULL == p_config->i2c_port || NULL == p_data)
    {
        return false;
    }
    
    /* Start reading from X high byte register */
    reg_addr = IMU_OUT_X_H_M;
    
    result = i2c_write_timeout_us(p_config->i2c_port,
                                  IMU_MAG_ADDR,
                                  &reg_addr,
                                  1,
                                  true,
                                  I2C_TIMEOUT_US);
    
    if (result != 1)
    {
        return false;
    }
    
    /* Read 6 bytes: X_H, X_L, Z_H, Z_L, Y_H, Y_L */
    result = i2c_read_timeout_us(p_config->i2c_port,
                                 IMU_MAG_ADDR,
                                 buffer,
                                 6,
                                 false,
                                 I2C_TIMEOUT_US);
    
    if (result != 6)
    {
        return false;
    }
    
    /* Combine bytes - NOTE: HIGH byte comes first! */
    /* buffer[0] = X_H, buffer[1] = X_L */
    p_data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    /* buffer[2] = Z_H, buffer[3] = Z_L */
    p_data->z = (int16_t)((buffer[2] << 8) | buffer[3]);
    
    /* buffer[4] = Y_H, buffer[5] = Y_L */
    p_data->y = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    return true;
}

/* ============================================================================
 * WHAT TO SAY IF PROFESSOR ASKS ABOUT MAGNETOMETER
 * ============================================================================
 * 
 * Q: "Why does the magnetometer have a different I2C address?"
 * A: "The LSM303DLHC is actually two separate sensors in one package - an
 *    accelerometer and a magnetometer. Each has its own I2C address so they
 *    can be accessed independently. Accelerometer is 0x19, magnetometer is
 *    0x1E. This allows you to use just one sensor if needed."
 * 
 * Q: "Why is the magnetometer axis order X, Z, Y instead of X, Y, Z?"
 * A: "That's how ST designed the register layout. The physical axes are still
 *    X, Y, Z, but the registers are arranged differently. We handle this in
 *    software by reading buffer[0-1] for X, buffer[2-3] for Z, and buffer[4-5]
 *    for Y, then storing them in the correct order in our structure."
 * 
 * Q: "Why is magnetometer data HIGH byte first while accelerometer is LOW byte first?"
 * A: "Different engineers designed each sensor core. The accelerometer follows
 *    'little-endian' convention (LSB first), while the magnetometer follows
 *    'big-endian' convention (MSB first). Both are valid, just different
 *    design choices. We handle this by changing how we combine the bytes."
 * 
 * Q: "What is gain and why set it to 1.3 gauss?"
 * A: "Gain sets the measurement range. Earth's magnetic field is about 0.25
 *    to 0.65 gauss depending on location. Setting gain to ±1.3 gauss gives
 *    us the best resolution for compass applications. Higher gains (like 8.1)
 *    would work but with less precision."
 * 
 * Q: "What does continuous mode mean?"
 * A: "There are three modes: continuous, single-shot, and sleep. In continuous
 *    mode, the sensor automatically takes new measurements at the configured
 *    data rate (15 Hz). In single-shot, you have to trigger each measurement.
 *    Continuous is easier for our compass application."
 * 
 * Q: "How do you calculate heading from raw magnetometer data?"
 * A: "We use atan2(Y, X) to get the angle in the horizontal plane. This gives
 *    the angle from magnetic North. We have to account for the sensor's tilt
 *    using accelerometer data for a true compass, but for a simple heading
 *    display, just atan2(Y, X) works when the sensor is flat."
 * 
 * ============================================================================
 */