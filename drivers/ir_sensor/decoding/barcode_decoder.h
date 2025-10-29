/**
 * @file barcode_decoder.h
 * @brief Code-39 Barcode Decoder Module
 */

#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H

#include <stdint.h>
#include <stdbool.h>

#define BARCODE_MAX_STRING 32

// Command types
typedef enum {
    BARCODE_CMD_NONE = 0,
    BARCODE_CMD_LEFT,
    BARCODE_CMD_RIGHT,
    BARCODE_CMD_STOP,
    BARCODE_CMD_UTURN,
    BARCODE_CMD_UNKNOWN
} barcode_command_t;

// Result structure
typedef struct {
    char decoded_string[BARCODE_MAX_STRING];
    barcode_command_t command;
    bool valid;
    uint32_t timestamp_ms;
} barcode_result_t;

/**
 * @brief Initialize barcode decoder
 */
void barcode_decoder_init(void);

/**
 * @brief Update decoder (call every 1ms in task)
 */
void barcode_decoder_update(void);

/**
 * @brief Get decoded barcode result
 * @return true if new result available
 */
bool barcode_decoder_get_result(barcode_result_t *result);

/**
 * @brief Reset decoder state
 */
void barcode_decoder_reset(void);

/**
 * @brief Convert string to command
 */
barcode_command_t barcode_string_to_command(const char *str);

/**
 * @brief Convert command to string
 */
const char* barcode_command_to_string(barcode_command_t cmd);

#endif // BARCODE_DECODER_H