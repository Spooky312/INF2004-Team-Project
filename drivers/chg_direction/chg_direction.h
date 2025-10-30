// ===============================================
//  Module: Change Direction Button
//  Description: Handles push-button press to toggle motor direction.
// ===============================================
#ifndef CHG_DIRECTION_H
#define CHG_DIRECTION_H

#define CHG_DIRECTION_PIN  21


#include "pico/stdlib.h"

void chg_direction_init(void);
bool chg_direction_was_pressed(void);
void chg_direction_irq_handler(uint gpio, uint32_t events);


#endif
