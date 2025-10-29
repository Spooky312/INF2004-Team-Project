// servo.c
#include "servo.h"
#include "hardware/pwm.h"

void servo_init(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 20000);
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_enabled(slice, true);
}

void servo_set_angle(uint gpio, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    float pulse_us = 1000 + (angle / 180.0f) * 1000;
    pwm_set_gpio_level(gpio, (uint16_t)pulse_us);
}
