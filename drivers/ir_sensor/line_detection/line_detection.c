/**
 * @file line_detection.c
 * @brief IR Line Sensor Implementation
 * Extracted and adapted from uploaded line_detection.c
 */

#include "line_detection.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// Configuration
#define ADC_INPUT        0
#define GPIO_PIN         26
#define DEFAULT_THRESH   280

static uint16_t threshold = DEFAULT_THRESH;

void line_detection_init(void) {
    adc_init();
    adc_gpio_init(GPIO_PIN);
    adc_select_input(ADC_INPUT);
    threshold = DEFAULT_THRESH;
}

void line_detection_read(line_data_t *data) {
    if (!data) return;
    
    // Read ADC
    uint16_t raw = adc_read();
    
    // Determine if on line (black)
    bool on_line = (raw >= threshold);
    
    // Calculate position
    // Simple mapping: below threshold = white (right), above = black (left)
    // You can enhance this with multi-sensor array
    int16_t position;
    if (on_line) {
        // On black line - calculate position based on how dark
        // For single sensor, we approximate position
        // Darker = more left, lighter = more right
        float normalized = (float)(raw - threshold) / (4095.0f - threshold);
        position = (int16_t)(-50 + normalized * 100); // Range: -50 to +50
    } else {
        // On white - assume line to the right
        float normalized = (float)raw / (float)threshold;
        position = (int16_t)(normalized * 100); // Range: 0 to +100
    }
    
    // Clamp position
    if (position < -100) position = -100;
    if (position > 100) position = 100;
    
    // Fill data structure
    data->raw_adc = raw;
    data->position = position;
    data->line_detected = on_line;
    data->timestamp_ms = to_ms_since_boot(get_absolute_time());
}

void line_detection_set_threshold(uint16_t thresh) {
    threshold = thresh;
}

uint16_t line_detection_get_threshold(void) {
    return threshold;
}