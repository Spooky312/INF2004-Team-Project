// ===============================================
//  Module: Line Sensor Driver
//  Description: ADC-based line detection for PID control
// ===============================================
#include "line_sensor/line_sensor.h"
#include <stdio.h>

static uint16_t threshold = LINE_THRESHOLD;

void line_sensor_init(void) {
    adc_init();
    adc_gpio_init(LINE_ADC_PIN);
    adc_select_input(LINE_ADC_INPUT);
    printf("[LINE_SENSOR] Initialized on GPIO %d (ADC%d)\n", LINE_ADC_PIN, LINE_ADC_INPUT);
}

line_state_t line_sensor_read(void) {
    uint16_t value = adc_read();
    // High ADC = black line, Low ADC = white surface
    return (value > threshold) ? LINE_BLACK : LINE_WHITE;
}

uint16_t line_sensor_read_raw(void) {
    return adc_read();
}

void line_sensor_set_threshold(uint16_t new_threshold) {
    threshold = new_threshold;
    printf("[LINE_SENSOR] Threshold set to %u\n", threshold);
}

float line_sensor_get_error(void) {
    // Simple single-sensor implementation
    // Returns: -1.0 (off line left), 0.0 (on line), +1.0 (off line right)
    line_state_t state = line_sensor_read();
    
    // For a single sensor centered on the line:
    // BLACK (on line) = 0.0 error
    // WHITE (off line) = error depends on last known direction
    
    static float last_error = 0.0f;
    
    if (state == LINE_BLACK) {
        last_error = 0.0f;
        return 0.0f;
    } else {
        // Lost line - maintain last error direction
        // In a real implementation with multiple sensors, 
        // you'd calculate weighted position here
        return last_error;
    }
}
