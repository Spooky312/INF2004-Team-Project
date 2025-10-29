#include "motor.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdlib.h> // for abs()

// Helper function to initialize one PWM slice
static void init_pwm_pin(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    static bool slice_initialized[NUM_PWM_SLICES] = {false};

    if (!slice_initialized[slice]) {
        // 20 kHz, ~13-bit resolution
        pwm_set_wrap(slice, 6249); 
        pwm_set_clkdiv(slice, 1.0f);
        pwm_set_enabled(slice, true);
        slice_initialized[slice] = true;
    }
}

void motor_init(void) {
    init_pwm_pin(PWM_M1A);
    init_pwm_pin(PWM_M1B);
    init_pwm_pin(PWM_M2A);
    init_pwm_pin(PWM_M2B);
}

void set_robo_motor(float speed_percent, uint pinA, uint pinB) {
    uint sliceA = pwm_gpio_to_slice_num(pinA);
    uint chanA  = pwm_gpio_to_channel(pinA);
    uint sliceB = pwm_gpio_to_slice_num(pinB);
    uint chanB  = pwm_gpio_to_channel(pinB);

    uint16_t top = pwm_hw->slice[sliceA].top;

    if (speed_percent > 1.0f) speed_percent = 1.0f;
    if (speed_percent < -1.0f) speed_percent = -1.0f;

    if (speed_percent > 0) { // Forward
        pwm_set_chan_level(sliceA, chanA, (uint16_t)(speed_percent * top));
        pwm_set_chan_level(sliceB, chanB, 0);
    } else if (speed_percent < 0) { // Reverse
        pwm_set_chan_level(sliceA, chanA, 0);
        pwm_set_chan_level(sliceB, chanB, (uint16_t)(-speed_percent * top));
    } else { // Stop
        pwm_set_chan_level(sliceA, chanA, 0);
        pwm_set_chan_level(sliceB, chanB, 0);
    }
}