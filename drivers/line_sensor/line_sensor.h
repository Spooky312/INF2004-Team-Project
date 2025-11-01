// ===============================================
//  Module: Line Sensor Driver
//  Description: ADC-based line detection for PID control
// ===============================================
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

// Configuration
#define LINE_ADC_PIN        27
#define LINE_ADC_INPUT      1     // GPIO 27 = ADC1 (not ADC0!)
#define LINE_THRESHOLD      2000  // Midpoint: white ~0-100, black ~4000-4095
#define LINE_SAMPLE_COUNT   8     // Averaging samples

// Line state
typedef enum {
    LINE_WHITE = 0,
    LINE_BLACK = 1
} line_state_t;

// ---- Initialization ----
void line_sensor_init(void);

// ---- Read current line state ----
line_state_t line_sensor_read(void);

// ---- Get raw ADC value ----
uint16_t line_sensor_read_raw(void);

// ---- Set threshold (for calibration) ----
void line_sensor_set_threshold(uint16_t threshold);

// ---- Get error for PID (0 = centered, -1 = left, +1 = right) ----
// For multi-sensor array, this would return weighted position
float line_sensor_get_error(void);

#endif
