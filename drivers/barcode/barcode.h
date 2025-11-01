// ===============================================
//  Module: Barcode Decoder (Code-39)
//  Description: Edge-based barcode scanning for junction commands
// ===============================================
#ifndef BARCODE_H
#define BARCODE_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Configuration
#define BARCODE_IR_PIN      28
#define BARCODE_SAMPLE_MS   1
#define BARCODE_VERIFY_MS   2
#define BARCODE_BAR_IS_LOW  0      // 0: black = HIGH, 1: black = LOW
#define BARCODE_RESET_MS    1000   // Reset if no activity (ms)
#define BARCODE_MAX_LEN     16     // Max decoded message length

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
