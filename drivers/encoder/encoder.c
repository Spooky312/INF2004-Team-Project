#include "encoder.h"
#include "hardware/gpio.h"
#include "hardware/irq.h" // <-- ADDED THIS LINE
#include <math.h> // for M_PI

// Volatile variables for ISR
volatile absolute_time_t rise_time_left, fall_time_left;
volatile absolute_time_t rise_time_right, fall_time_right;

volatile uint32_t pulse_width_high_left = 0;
volatile uint32_t pulse_width_low_left  = 0;
volatile uint32_t pulse_width_high_right = 0;
volatile uint32_t pulse_width_low_right  = 0;

volatile int32_t total_pulse_count_left = 0;
volatile int32_t total_pulse_count_right = 0;

// RPM is read more often than it's updated, so cache it
static volatile float cached_rpm_left = 0.0f;
static volatile float cached_rpm_right = 0.0f;

// Combined ISR for both encoders
void encoder_isr(uint gpio, uint32_t events) {
    absolute_time_t now = get_absolute_time();

    if (gpio == ENCODER_PIN_L) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            pulse_width_low_left = absolute_time_diff_us(fall_time_left, now);
            rise_time_left = now;
            total_pulse_count_left++;
        }
        else if (events & GPIO_IRQ_EDGE_FALL) {
            pulse_width_high_left = absolute_time_diff_us(rise_time_left, now);
            fall_time_left = now;
        }
    }
    else if (gpio == ENCODER_PIN_R) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            pulse_width_low_right = absolute_time_diff_us(fall_time_right, now);
            rise_time_right = now;
            total_pulse_count_right++;
        }
        else if (events & GPIO_IRQ_EDGE_FALL) {
            pulse_width_high_right = absolute_time_diff_us(rise_time_right, now);
            fall_time_right = now;
        }
    }
}

void encoder_init(void) {
    gpio_init(ENCODER_PIN_L);
    gpio_set_dir(ENCODER_PIN_L, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_L);

    gpio_init(ENCODER_PIN_R);
    gpio_set_dir(ENCODER_PIN_R, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_R);

    gpio_set_irq_enabled_with_callback(ENCODER_PIN_L, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    gpio_set_irq_enabled(ENCODER_PIN_R, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// This function is "thread-safe" for reads because it only reads volatile vars
float encoder_get_left_rpm(void) {
    // Disable interrupts to get an atomic read of the pulse widths
    uint32_t irq_status = save_and_disable_interrupts(); // Now valid
    uint32_t highL = pulse_width_high_left;
    uint32_t lowL  = pulse_width_low_left;
    restore_interrupts(irq_status); // Now valid

    float periodL = highL + lowL;

    if (periodL > 0) {
        float freqL = 1e6f / periodL;
        // Reads/writes of float are generally atomic on ARM Cortex-M0+
        cached_rpm_left = (freqL * 60.0f) / PULSES_PER_REV;
    } else {
        // Add timeout logic if needed
        // ...
    }
    return cached_rpm_left;
}

float encoder_get_right_rpm(void) {
    // Disable interrupts to get an atomic read of the pulse widths
    uint32_t irq_status = save_and_disable_interrupts(); // Now valid
    uint32_t highR = pulse_width_high_right;
    uint32_t lowR  = pulse_width_low_right;
    restore_interrupts(irq_status); // Now valid

    float periodR = highR + lowR;

    if (periodR > 0) {
        float freqR = 1e6f / periodR;
        cached_rpm_right = (freqR * 60.0f) / PULSES_PER_REV;
    } else {
         // Add timeout logic if needed
         // ...
    }
    return cached_rpm_right;
}

float encoder_get_distance_m(void) {
    // Read volatile counts atomically
    uint32_t irq_status = save_and_disable_interrupts(); // Now valid
    long count_l = total_pulse_count_left;
    long count_r = total_pulse_count_right;
    restore_interrupts(irq_status); // Now valid

    long avg_pulse_count = (count_l + count_r) / 2;
    float revs = (float)avg_pulse_count / PULSES_PER_REV;
    float distance = revs * (M_PI * WHEEL_DIAMETER_M);
    return distance;
}