// ===============================================
//  Module: Debug LED Indicators
//  Description: Implements helper functions for LED feedback.
// ===============================================
#include "debug_led.h"
#include "hardware/timer.h"

#define STATUS_LED_PIN   25
#define IMU_LED_PIN      11
#define ENCODER_LED_PIN  12
#define WIFI_LED_PIN     13

void debug_led_init(void)
{
    const uint leds[] = { STATUS_LED_PIN, IMU_LED_PIN, ENCODER_LED_PIN, WIFI_LED_PIN };
    for (int i = 0; i < 4; i++) {
        gpio_init(leds[i]);
        gpio_set_dir(leds[i], GPIO_OUT);
        gpio_put(leds[i], 0);
    }
}

void debug_led_set(uint gpio, bool on)
{
    gpio_put(gpio, on);
}

void debug_led_blink(uint gpio, uint ms)
{
    gpio_put(gpio, 1);
    sleep_ms(ms);
    gpio_put(gpio, 0);
}

void debug_led_success(uint gpio)
{
    gpio_put(gpio, 1);
    sleep_ms(50);
    gpio_put(gpio, 0);
}

void debug_led_error(uint gpio)
{
    for (int i = 0; i < 3; i++) {
        gpio_put(gpio, 1);
        sleep_ms(100);
        gpio_put(gpio, 0);
        sleep_ms(100);
    }
}
