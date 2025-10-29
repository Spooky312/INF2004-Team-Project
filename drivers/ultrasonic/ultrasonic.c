// ultrasonic.c
#include "ultrasonic.h"

void ultrasonic_init(uint trig, uint echo) {
    gpio_init(trig); gpio_set_dir(trig, GPIO_OUT); gpio_put(trig, 0);
    gpio_init(echo); gpio_set_dir(echo, GPIO_IN);
}

static void trig_pulse(uint trig) {
    gpio_put(trig, 0); sleep_us(2);
    gpio_put(trig, 1); sleep_us(10);
    gpio_put(trig, 0);
}

float ultrasonic_get_distance_cm(uint trig, uint echo) {
    trig_pulse(trig);
    absolute_time_t start = get_absolute_time();
    while (!gpio_get(echo))
        if (absolute_time_diff_us(start, get_absolute_time()) > 30000) return -1;
    absolute_time_t echo_start = get_absolute_time();
    while (gpio_get(echo))
        if (absolute_time_diff_us(echo_start, get_absolute_time()) > 30000) return -1;
    uint32_t echo_time = absolute_time_diff_us(echo_start, get_absolute_time());
    return echo_time / 58.0f;
}
