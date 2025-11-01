// ===============================================
//  Module: Motor Driver
//  Description: PWM control for left and right DC motors.
// ===============================================
#include "motor.h"

// <-- CORRECTED: Pin assignments to match actual wiring
// Motor A (left): GP10 (PWM), GP11 (DIR)
// Motor B (right): GP8 (PWM), GP9 (DIR)
#define MOTOR_L_PWM 10
#define MOTOR_L_DIR 11
#define MOTOR_R_PWM 8
#define MOTOR_R_DIR 9

static uint slice_l, chan_l, slice_r, chan_r;

static uint16_t speed_to_pwm(float speed)
{
    // Accept speed in range 0.0 to 1.0
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;
    return (uint16_t)(speed * 65535.0f);
}

void motor_init(void)
{
    // Initialize PWM pins
    gpio_set_function(MOTOR_L_PWM, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_PWM, GPIO_FUNC_PWM);

    // Initialize direction pins
    gpio_init(MOTOR_L_DIR);
    gpio_set_dir(MOTOR_L_DIR, GPIO_OUT);
    gpio_put(MOTOR_L_DIR, 0);  // Start with forward direction
    
    gpio_init(MOTOR_R_DIR);
    gpio_set_dir(MOTOR_R_DIR, GPIO_OUT);
    gpio_put(MOTOR_R_DIR, 0);  // Start with forward direction

    slice_l = pwm_gpio_to_slice_num(MOTOR_L_PWM);
    chan_l  = pwm_gpio_to_channel(MOTOR_L_PWM);
    slice_r = pwm_gpio_to_slice_num(MOTOR_R_PWM);
    chan_r  = pwm_gpio_to_channel(MOTOR_R_PWM);

    pwm_set_wrap(slice_l, 65535);
    pwm_set_wrap(slice_r, 65535);
    pwm_set_enabled(slice_l, true);
    pwm_set_enabled(slice_r, true);
}

void motor_set_speed(float left_speed, float right_speed)
{
    // Handle left motor direction and speed
    if (left_speed < 0) {
        gpio_put(MOTOR_L_DIR, 1);  // Reverse
        left_speed = -left_speed;
    } else {
        gpio_put(MOTOR_L_DIR, 0);  // Forward
    }
    
    // Handle right motor direction and speed
    if (right_speed < 0) {
        gpio_put(MOTOR_R_DIR, 1);  // Reverse
        right_speed = -right_speed;
    } else {
        gpio_put(MOTOR_R_DIR, 0);  // Forward
    }
    
    pwm_set_chan_level(slice_l, chan_l, speed_to_pwm(left_speed));
    pwm_set_chan_level(slice_r, chan_r, speed_to_pwm(right_speed));
}

void motor_stop(void)
{
    pwm_set_chan_level(slice_l, chan_l, 0);
    pwm_set_chan_level(slice_r, chan_r, 0);
}