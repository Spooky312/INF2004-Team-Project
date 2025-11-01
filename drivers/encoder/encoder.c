// ===============================================
//  Module: Encoder Driver
//  Description: GPIO interrupt-based encoder tick counting
//               and live RPM computation for PID and telemetry.
// ===============================================
#include "encoder.h"
#include "hardware/timer.h"
#include <stdio.h>

// ---- User configuration ----
#define TICKS_PER_REV      360.0f
#define WHEEL_CIRCUM_M     0.21f
#define RPM_TIMEOUT_US     500000  // 0.5 seconds - if no ticks, assume stopped

// ---- Internal state ----
static volatile uint32_t left_ticks = 0;
static volatile uint32_t right_ticks = 0;
static absolute_time_t last_left_time, last_right_time;
static float rpm_left = 0.0f, rpm_right = 0.0f;
static float dist_left_m = 0.0f, dist_right_m = 0.0f;

// ---- Interrupt handler ----
void encoder_irq_handler(uint gpio, uint32_t events)
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

void encoder_init(void)
{
    gpio_init(ENCODER_LEFT_PIN);
    gpio_set_dir(ENCODER_LEFT_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_LEFT_PIN);

    gpio_init(ENCODER_RIGHT_PIN);
    gpio_set_dir(ENCODER_RIGHT_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_RIGHT_PIN);

    // DO NOT enable interrupts here - that's done by gpio_router_init() in main.c
    // gpio_set_irq_enabled(ENCODER_LEFT_PIN, GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_enabled(ENCODER_RIGHT_PIN, GPIO_IRQ_EDGE_FALL, true);

    last_left_time = last_right_time = get_absolute_time();
    printf("[ENC] Encoder GPIOs initialized (interrupts will be enabled by router).\n");
}


// ---- Accessors ----
float encoder_get_rpm_left(void)
{
    int64_t time_since_tick = absolute_time_diff_us(last_left_time, get_absolute_time());
    if (time_since_tick > RPM_TIMEOUT_US) {
        rpm_left = 0.0f;
    }
    return rpm_left;
}

float encoder_get_rpm_right(void)
{
    int64_t time_since_tick = absolute_time_diff_us(last_right_time, get_absolute_time());
    if (time_since_tick > RPM_TIMEOUT_US) {
        rpm_right = 0.0f;
    }
    return rpm_right;
}

float encoder_get_distance_m(void)
{
    return (dist_left_m + dist_right_m) / 2.0f;
}

void encoder_get_ticks(uint32_t *left, uint32_t *right)
{
    if (left) *left = left_ticks;
    if (right) *right = right_ticks;
}
