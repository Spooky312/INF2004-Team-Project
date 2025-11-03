// ===============================================
//  Module: Barcode Decoder (Code-39)
//  Description: Edge-based barcode scanning for junction commands
// ===============================================
#ifndef BARCODE_H
#define BARCODE_H

#include "pico/stdlib.h"
#include <stdbool.h>
#include "robot_config.h"

// Configuration now in robot_config.h:
// - BARCODE_IR_DO_PIN
// - BARCODE_SAMPLE_MS
// - BARCODE_VERIFY_MS
// - BARCODE_BAR_IS_LOW
// - BARCODE_RESET_MS
// - BARCODE_MAX_LEN

// Barcode commands (expected decoded strings)
typedef enum {
    CMD_NONE = 0,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP,
    CMD_FORWARD
} barcode_command_t;

// Callback signature for barcode detection
typedef void (*barcode_callback_t)(const char* decoded_str, barcode_command_t cmd);

// ---- Initialization ----
void barcode_init(void);

// ---- Start/Stop scanning ----
void barcode_start_scanning(void);
void barcode_stop_scanning(void);

// ---- Register callback ----
void barcode_set_callback(barcode_callback_t callback);

// ---- Get last decoded barcode ----
const char* barcode_get_last_decoded(void);

// ---- Parse command from decoded string ----
barcode_command_t barcode_parse_command(const char* str);

// ---- Manual polling (if not using timer interrupt) ----
void barcode_update(void);

#endif
