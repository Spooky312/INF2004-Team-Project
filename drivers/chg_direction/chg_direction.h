// ===============================================
//  Module: Change Direction Button
//  Description: Handles push-button press to toggle motor direction.
// ===============================================
#ifndef CHG_DIRECTION_H
#define CHG_DIRECTION_H

#include "pico/stdlib.h"

void chg_direction_init(void);
bool chg_direction_was_pressed(void);

#endif
