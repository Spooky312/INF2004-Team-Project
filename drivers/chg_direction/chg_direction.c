// ===============================================
//  Module: Change Direction Button
//  Description: Detects button press on GPIO and sets a flag
//               to indicate the robot should reverse direction.
// ===============================================
#include "chg_direction.h"

#define CHG_DIRECTION_PIN 10   // adjust pin as wired

static volatile bool pressed_flag = false;

static void chg_direction_irq(uint gpio, uint32_t events)
{
    if (gpio == CHG_DIRECTION_PIN)
        pressed_flag = true;  // flag set; cleared when read
}

void chg_direction_init(void)
{
    gpio_init(CHG_DIRECTION_PIN);
    gpio_set_dir(CHG_DIRECTION_PIN, GPIO_IN);
    gpio_pull_up(CHG_DIRECTION_PIN);

    gpio_set_irq_enabled_with_callback(
        CHG_DIRECTION_PIN, GPIO_IRQ_EDGE_FALL, true, &chg_direction_irq);
}

bool chg_direction_was_pressed(void)
{
    bool was = pressed_flag;
    pressed_flag = false;
    return was;
}
