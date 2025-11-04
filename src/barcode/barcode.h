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
    CMD_FORWARD,
    CMD_UTURN
} barcode_command_t;

// Result structure for barcode detection
typedef struct {
    char decoded_string[64];
    barcode_command_t command;
    bool valid;
    uint32_t timestamp_ms;
} barcode_result_t;

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

// ---- Get barcode result ----
bool barcode_get_result(barcode_result_t *result);

// ---- Reset decoder ----
void barcode_reset(void);

// ---- Speed Control API (Autonomous Speed Adjustment) ----
// Get average narrow duration from recent successful scans (ms)
float barcode_get_avg_narrow_duration_ms(void);

// Set calibrated module width (m) - call once during initial calibration
void barcode_set_module_width_m(float width_m);

// Get current module width setting (m)
float barcode_get_module_width_m(void);

// Compute target RPM for desired narrow duration (ms)
// Returns 0 if not calibrated
float barcode_compute_target_rpm(float desired_narrow_ms);

// Clear narrow duration history (useful when changing speed significantly)
void barcode_reset_narrow_history(void);

// Get number of narrow samples collected
int barcode_get_narrow_sample_count(void);

#endif
