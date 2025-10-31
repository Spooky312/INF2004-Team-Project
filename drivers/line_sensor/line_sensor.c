// ===============================================
//  Line Sensor Driver - Implementation
//  Description: Analog IR sensor for black line detection
//  With persistent moving average filter for motor EMI rejection
// ===============================================

#include "line_sensor.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// -----------------------------------------------
// Moving Average Filter State
// -----------------------------------------------
#define FILTER_SIZE 8
static uint16_t filter_buffer[FILTER_SIZE] = {0};
static uint8_t filter_index = 0;
static bool filter_initialized = false;

// -----------------------------------------------
// Initialization
// -----------------------------------------------
void line_sensor_init(void)
{
    // Initialize ADC
    adc_init();
    
    // Configure GP26 as ADC input
    adc_gpio_init(LINE_SENSOR_PIN);
    
    // Select the ADC channel
    adc_select_input(LINE_SENSOR_ADC_CHAN);
    
    // Initialize filter with initial readings
    for (int i = 0; i < FILTER_SIZE; i++) {
        adc_select_input(LINE_SENSOR_ADC_CHAN);
        filter_buffer[i] = adc_read();
        sleep_us(100);
    }
    filter_initialized = true;
    filter_index = 0;
}

// -----------------------------------------------
// Read Raw ADC Value with persistent filtering
// -----------------------------------------------
uint16_t line_sensor_read_raw(void)
{
    // Select ADC channel (in case it was changed)
    adc_select_input(LINE_SENSOR_ADC_CHAN);
    
    // Take multiple samples and average (reduces single-read noise)
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) {
        sum += adc_read();
        sleep_us(50);  // ADC settling time
    }
    uint16_t raw_sample = (uint16_t)(sum / 4);
    
    // Update moving average buffer (persists across calls)
    filter_buffer[filter_index] = raw_sample;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    // Calculate filtered average
    uint32_t filtered_sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        filtered_sum += filter_buffer[i];
    }
    
    return (uint16_t)(filtered_sum / FILTER_SIZE);
}

// -----------------------------------------------
// Check if on BLACK line
// Your logic: value < threshold = WHITE, value >= threshold = BLACK
// -----------------------------------------------
bool line_sensor_on_line(void)
{
    uint16_t value = line_sensor_read_raw();
    // BLACK line has higher ADC values
    return (value >= LINE_THRESHOLD);
}