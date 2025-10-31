// ===============================================
//  Line Sensor Driver - Header
//  Description: Analog IR sensor for black line detection
//  Hardware: MH-series IR sensor on GP26 (ADC0)
// ===============================================

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------
// Configuration
// -----------------------------------------------
#define LINE_SENSOR_PIN         27      // GP27 = ADC1 (changed from GP26 to avoid encoder conflict)
#define LINE_SENSOR_ADC_CHAN    1       // ADC channel 1
#define LINE_THRESHOLD          280     // Your original threshold: < 280 = WHITE, >= 280 = BLACK

// -----------------------------------------------
// API Functions
// -----------------------------------------------

/**
 * Initialize line sensor (ADC)
 */
void line_sensor_init(void);

/**
 * Read raw ADC value (0-4095)
 * Returns filtered value using moving average
 */
uint16_t line_sensor_read_raw(void);

/**
 * Check if sensor is over black line
 * Returns true if ADC value > LINE_THRESHOLD
 */
bool line_sensor_on_line(void);

#endif // LINE_SENSOR_H