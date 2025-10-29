#include "motor.h"
#include "hardware/pwm.h"

// Robo Pico motor pins
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

// ---- Helper: initialise PWM channel ----
static void pwm_init_pin(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 6249);       // 20 kHz PWM
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_enabled(slice, true);
}

// ---- Local re-implementation of set_robo_motor ----
static void set_robo_motor(float speed_percent, uint pinA, uint pinB) {
    uint sliceA = pwm_gpio_to_slice_num(pinA);
    uint chanA  = pwm_gpio_to_channel(pinA);
    uint sliceB = pwm_gpio_to_slice_num(pinB);
    uint chanB  = pwm_gpio_to_channel(pinB);
    uint16_t top = pwm_hw->slice[sliceA].top;

    if (speed_percent > 0) {
        // Forward
        pwm_set_chan_level(sliceA, chanA, (uint16_t)(speed_percent * top));
        pwm_set_chan_level(sliceB, chanB, 0);
    } else if (speed_percent < 0) {
        // Reverse
        pwm_set_chan_level(sliceA, chanA, 0);
        pwm_set_chan_level(sliceB, chanB, (uint16_t)(-speed_percent * top));
    } else {
        // Stop
        pwm_set_chan_level(sliceA, chanA, 0);
        pwm_set_chan_level(sliceB, chanB, 0);
    }
}

// ---- Public API ----
void motor_init(void) {
    pwm_init_pin(M1A);
    pwm_init_pin(M1B);
    pwm_init_pin(M2A);
    pwm_init_pin(M2B);
}

void motor_set_speed(float left_speed, float right_speed) {
    set_robo_motor(left_speed,  M1A, M1B);
    set_robo_motor(right_speed, M2A, M2B);
}

void motor_stop(void) {
    motor_set_speed(0, 0);
}
