// ===============================================
//  Module: Change Direction Button
//  Description: Detects button press on GPIO and sets a flag
//               to indicate the robot should reverse direction.
// ===============================================
#include "chg_direction.h"

#define CHG_DIRECTION_PIN 21   // adjust as wired

static volatile bool pressed_flag = false;

void chg_direction_irq_handler(uint gpio, uint32_t events)
{
    if (gpio == CHG_DIRECTION_PIN)
        pressed_flag = true;
}

void chg_direction_init(void)
{
    gpio_init(CHG_DIRECTION_PIN);
    gpio_set_dir(CHG_DIRECTION_PIN, GPIO_IN);
    gpio_pull_up(CHG_DIRECTION_PIN);

    // DO NOT enable interrupts here - that's done by gpio_router_init() in main.c
    // gpio_set_irq_enabled(CHG_DIRECTION_PIN, GPIO_IRQ_EDGE_FALL, true);
}

bool chg_direction_was_pressed(void)
{
    bool was = pressed_flag;
    pressed_flag = false;
    return was;
}

