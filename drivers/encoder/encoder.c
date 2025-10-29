// ===============================================
//  Module: Encoder Driver
//  Description: GPIO interrupt-based encoder tick counting
//               and live RPM computation for PID and telemetry.
// ===============================================
#include "encoder.h"
#include "hardware/timer.h"

// ---- User configuration ----
#define ENCODER_LEFT_PIN   6
#define ENCODER_RIGHT_PIN  7
#define TICKS_PER_REV      360.0f
#define WHEEL_CIRCUM_M     0.21f

// ---- Internal state ----
static volatile uint32_t left_ticks = 0;
static volatile uint32_t right_ticks = 0;
static absolute_time_t last_left_time, last_right_time;
static float rpm_left = 0.0f, rpm_right = 0.0f;
static float dist_left_m = 0.0f, dist_right_m = 0.0f;

// ---- Interrupt handler ----
static void encoder_irq_handler(uint gpio, uint32_t events)
{
    absolute_time_t now = get_absolute_time();

    if (gpio == ENCODER_LEFT_PIN) {
        left_ticks++;
        dist_left_m = (left_ticks / TICKS_PER_REV) * WHEEL_CIRCUM_M;

        // compute instantaneous rpm
        int64_t dt_us = absolute_time_diff_us(last_left_time, now);
        if (dt_us > 0) rpm_left = 60.0f * 1e6f / (TICKS_PER_REV * dt_us);
        last_left_time = now;
    }

    if (gpio == ENCODER_RIGHT_PIN) {
        right_ticks++;
        dist_right_m = (right_ticks / TICKS_PER_REV) * WHEEL_CIRCUM_M;

        int64_t dt_us = absolute_time_diff_us(last_right_time, now);
        if (dt_us > 0) rpm_right = 60.0f * 1e6f / (TICKS_PER_REV * dt_us);
        last_right_time = now;
    }
}

// ---- Initialization ----
void encoder_init(void)
{
    gpio_init(ENCODER_LEFT_PIN);
    gpio_set_dir(ENCODER_LEFT_PIN, GPIO_IN);
    gpio_pull_down(ENCODER_LEFT_PIN);

    gpio_init(ENCODER_RIGHT_PIN);
    gpio_set_dir(ENCODER_RIGHT_PIN, GPIO_IN);
    gpio_pull_down(ENCODER_RIGHT_PIN);

    // attach interrupts (rising edge)
    gpio_set_irq_enabled_with_callback(
        ENCODER_LEFT_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_handler);
    gpio_set_irq_enabled(ENCODER_RIGHT_PIN, GPIO_IRQ_EDGE_RISE, true);

    last_left_time = last_right_time = get_absolute_time();
}

// ---- Accessors ----
float encoder_get_rpm_left(void)  { return rpm_left; }
float encoder_get_rpm_right(void) { return rpm_right; }
float encoder_get_distance_m(void)
{
    return (dist_left_m + dist_right_m) / 2.0f;
}
