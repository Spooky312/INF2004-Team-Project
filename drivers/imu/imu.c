// ===============================================
//  Module: IMU (LSM303DLHC) Driver
//  Description: I2C bring-up, raw accel/mag reads,
//               tilt/heading helpers, EMA-filtered heading.
// ===============================================
#include "imu.h"
#include <math.h>
#include <string.h>

// ---- LSM303DLHC I2C addresses ----
#define LSM_ACC_ADDR         0x19   // SA0_A = 1 typical
#define LSM_MAG_ADDR         0x1E

// ---- Accelerometer registers ----
#define A_CTRL_REG1          0x20   // ODR, enable axes
#define A_CTRL_REG4          0x23   // full-scale, high-res
#define A_OUT_X_L            0x28   // auto-increment from here

// ---- Magnetometer registers ----
#define M_CRA_REG            0x00   // data rate
#define M_CRB_REG            0x01   // gain
#define M_MR_REG             0x02   // mode
#define M_OUT_X_H            0x03   // X H, X L, Z H, Z L, Y H, Y L

// ---- Helpers ----
static inline int i2c_w1(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(IMU_I2C, addr, buf, 2, false);
}
static inline int i2c_rn(uint8_t addr, uint8_t reg, uint8_t *dst, size_t n) {
    i2c_write_blocking(IMU_I2C, addr, &reg, 1, true);
    return i2c_read_blocking(IMU_I2C, addr, dst, n, false);
}

// ---- State ----
static float ema_heading_deg = 0.0f;
static bool  ema_inited = false;
static float last_pitch_deg = 0.0f, last_roll_deg = 0.0f;

// ---- Simple clamp ----
static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }

// ---- Bring-up ----
void imu_init(void)
{
    // I2C pins + speed
    i2c_init(IMU_I2C, IMU_I2C_BAUD_HZ);
    gpio_set_function(IMU_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA);
    gpio_pull_up(IMU_I2C_SCL);

    // Accelerometer: 100 Hz, all axes on
    // CTRL_REG1_A: ODR=100Hz (0101), LPen=0, XYZ enable = 111 -> 0b01010111 = 0x57
    i2c_w1(LSM_ACC_ADDR, A_CTRL_REG1, 0x57);
    // CTRL_REG4_A: High-resolution, ±2g (00), BDU=0 -> 0x08
    i2c_w1(LSM_ACC_ADDR, A_CTRL_REG4, 0x08);

    // Magnetometer: 75 Hz, gain default, continuous-conversion
    // CRA: DO=75Hz -> 0x18
    i2c_w1(LSM_MAG_ADDR, M_CRA_REG, 0x18);
    // CRB: Gain = default (1.3 gauss) -> 0x20
    i2c_w1(LSM_MAG_ADDR, M_CRB_REG, 0x20);
    // MR: Continuous convert -> 0x00
    i2c_w1(LSM_MAG_ADDR, M_MR_REG, 0x00);

    ema_heading_deg = 0.0f;
    ema_inited = false;
    last_pitch_deg = last_roll_deg = 0.0f;
}

// ---- Raw reads ----
bool imu_read_accel(float *ax_g, float *ay_g, float *az_g)
{
    uint8_t out[6];
    // Enable auto-increment by setting MSB of sub-address
    if (i2c_rn(LSM_ACC_ADDR, (A_OUT_X_L | 0x80), out, 6) != 6) return false;

    // 12-bit right-aligned in high-res mode: combine and sign-extend
    int16_t x = (int16_t)((out[1] << 8) | out[0]) >> 4;
    int16_t y = (int16_t)((out[3] << 8) | out[2]) >> 4;
    int16_t z = (int16_t)((out[5] << 8) | out[4]) >> 4;

    // Sensitivity: 1 mg/LSB at ±2g in HR mode -> 0.001 g/LSB
    const float sens_g = 0.001f;
    if (ax_g) *ax_g = x * sens_g;
    if (ay_g) *ay_g = y * sens_g;
    if (az_g) *az_g = z * sens_g;
    return true;
}

bool imu_read_mag(float *mx_g, float *my_g, float *mz_g)
{
    uint8_t out[6];
    if (i2c_rn(LSM_MAG_ADDR, M_OUT_X_H, out, 6) != 6) return false;

    // Note: order is X, Z, Y in this device!
    int16_t x = (int16_t)((out[0] << 8) | out[1]);
    int16_t z = (int16_t)((out[2] << 8) | out[3]);
    int16_t y = (int16_t)((out[4] << 8) | out[5]);

    // Sensitivity for 1.3 gauss: 1100 LSB/gauss (X,Y), 980 LSB/gauss (Z)
    const float sx = 1.0f / 1100.0f;
    const float sy = 1.0f / 1100.0f;
    const float sz = 1.0f / 980.0f;

    if (mx_g) *mx_g = x * sx;
    if (my_g) *my_g = y * sy;
    if (mz_g) *mz_g = z * sz;
    return true;
}

// ---- Pitch / Roll (from accel) ----
static void compute_pitch_roll(float ax, float ay, float az, float *pitch_deg, float *roll_deg)
{
    // Protect against divide-by-zero
    ax = clampf(ax, -1.9f, 1.9f);
    ay = clampf(ay, -1.9f, 1.9f);
    az = clampf(az, -1.9f, 1.9f);

    // Roll around X, Pitch around Y
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    if (pitch_deg) *pitch_deg = pitch * 180.0f / (float)M_PI;
    if (roll_deg)  *roll_deg  = roll  * 180.0f / (float)M_PI;

    last_pitch_deg = pitch * 180.0f / (float)M_PI;
    last_roll_deg  = roll  * 180.0f / (float)M_PI;
}

void imu_get_pitch_roll_deg(float *pitch_deg, float *roll_deg)
{
    if (pitch_deg) *pitch_deg = last_pitch_deg;
    if (roll_deg)  *roll_deg  = last_roll_deg;
}

// ---- Heading (raw + filtered) ----
bool imu_get_heading_deg(float *heading_raw_deg, float *heading_filt_deg)
{
    float ax, ay, az, mx, my, mz;
    if (!imu_read_accel(&ax, &ay, &az)) return false;
    if (!imu_read_mag(&mx, &my, &mz))   return false;

    // Tilt compensation (simple form using pitch/roll from accel)
    float pitch_rad = 0.0f, roll_rad = 0.0f;
    {
        float pd, rd;
        compute_pitch_roll(ax, ay, az, &pd, &rd);
        pitch_rad = pd * (float)M_PI / 180.0f;
        roll_rad  = rd * (float)M_PI / 180.0f;
    }

    // Compensate mag readings
    float mx_comp = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
    float my_comp = mx * sinf(roll_rad)  * sinf(pitch_rad)
                  + my * cosf(roll_rad)
                  - mz * sinf(roll_rad)  * cosf(pitch_rad);

    // Heading in radians, add declination
    float heading = atan2f(-my_comp, mx_comp) + IMU_DECLINATION_RAD;

    // Normalize to [0, 2π)
    if (heading < 0) heading += 2.0f * (float)M_PI;
    if (heading >= 2.0f * (float)M_PI) heading -= 2.0f * (float)M_PI;

    float heading_deg = heading * 180.0f / (float)M_PI;

    // EMA filter
    if (!ema_inited) {
        ema_heading_deg = heading_deg;
        ema_inited = true;
    } else {
        ema_heading_deg = (IMU_HEADING_EMA_ALPHA * heading_deg)
                        + (1.0f - IMU_HEADING_EMA_ALPHA) * ema_heading_deg;
    }

    if (heading_raw_deg)  *heading_raw_deg  = heading_deg;
    if (heading_filt_deg) *heading_filt_deg = ema_heading_deg;
    return true;
}
