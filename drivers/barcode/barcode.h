// ===============================================
//  Barcode Scanner Driver - Header
//  Description: Code-39 barcode decoder
//  Hardware: MH-series digital IR sensor on GP7
// ===============================================

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------
// Configuration
// -----------------------------------------------
#define BARCODE_PIN             28       // GP28 for digital IR sensor
#define BARCODE_DEBOUNCE_US     2000    // 2ms debounce

// -----------------------------------------------
// API Functions
// -----------------------------------------------

/**
 * Initialize barcode scanner GPIO
 */
void barcode_init(void);

/**
 * Process barcode scanner (call frequently, e.g., 1kHz)
 * This is non-blocking and updates internal state
 */
void barcode_process(void);

/**
 * Check if a barcode has been successfully decoded
 * Returns true if valid barcode is available
 */
bool barcode_scan_available(void);

/**
 * Get the decoded barcode character
 * Returns the last decoded character (A-Z, 0-9, etc.)
 */
char barcode_get_result(void);

/**
 * Clear the current scan result
 */
void barcode_clear(void);

#endif // BARCODE_H