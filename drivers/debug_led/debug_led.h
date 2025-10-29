// ===============================================
//  Module: Debug LED Indicators
//  Description: Provides simple LED control for live system diagnostics.
// ===============================================
#ifndef DEBUG_LED_H
#define DEBUG_LED_H

#include "pico/stdlib.h"

void debug_led_init(void);
void debug_led_set(uint gpio, bool on);
void debug_led_blink(uint gpio, uint ms);
void debug_led_success(uint gpio);
void debug_led_error(uint gpio);

#endif
