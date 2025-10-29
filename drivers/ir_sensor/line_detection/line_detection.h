/**
 * @file line_detection.h
 * @brief IR Line Sensor Module
 */

#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include <stdint.h>
#include <stdbool.h>

// Line sensor data
typedef struct {
    uint16_t raw_adc;           // Raw ADC value (0-4095)
    int16_t position;           // -100 (left) to +100 (right), 0 = center
    bool line_detected;         // true if line detected
    uint32_t timestamp_ms;      // Timestamp
} line_data_t;

/**
 * @brief Initialize line detection
 */
void line_detection_init(void);

/**
 * @brief Read line sensor
 * @param data Pointer to store result
 */
void line_detection_read(line_data_t *data);

/**
 * @brief Set detection threshold
 * @param threshold ADC threshold value (0-4095)
 */
void line_detection_set_threshold(uint16_t threshold);

/**
 * @brief Get current threshold
 * @return Current threshold value
 */
uint16_t line_detection_get_threshold(void);

#endif // LINE_DETECTION_H