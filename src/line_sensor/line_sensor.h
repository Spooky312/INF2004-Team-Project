// ===============================================
//  Module: Line Sensor Driver
//  Description: ADC-based line detection for PID control
// ===============================================
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "robot_config.h"

// Configuration now in robot_config.h:
// - LINE_IR_AO_PIN (GPIO 27)
// - LINE_ADC_INPUT (ADC1)
// - LINE_THRESHOLD
// - LINE_SAMPLE_COUNT

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
