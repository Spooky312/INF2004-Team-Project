// ===============================================
//  Module: Motor Driver
//  Description: PWM control for left and right DC motors.
// ===============================================
#include "motor.h"

#define MOTOR_L_PWM 14
#define MOTOR_R_PWM 15

static uint slice_l, chan_l, slice_r, chan_r;

static uint16_t speed_to_pwm(float speed)
{
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;
    return (uint16_t)((speed / 255.0f) * 65535);
}

void motor_init(void)
{
    gpio_set_function(MOTOR_L_PWM, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_PWM, GPIO_FUNC_PWM);

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
    pwm_set_chan_level(slice_l, chan_l, speed_to_pwm(left_speed));
    pwm_set_chan_level(slice_r, chan_r, speed_to_pwm(right_speed));
}

void motor_stop(void)
{
    pwm_set_chan_level(slice_l, chan_l, 0);
    pwm_set_chan_level(slice_r, chan_r, 0);
}
