/**
 * @file barcode_decoder.c
 * @brief Code-39 Barcode Decoder Implementation
 * Extracted and adapted from uploaded barcode_decoder.c
 */

#include "barcode_decoder.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Configuration
#define IR_PIN             7
#define VERIFY_MS          2
#define BAR_IS_LOW         0
#define SCAN_RESET_MS      1000

// Edge detection state
static volatile uint8_t raw_level = 0;
static volatile uint8_t stable_level = 0;
static volatile uint8_t prev_stable_level = 0;
static volatile uint32_t last_change_ms = 0;
static volatile uint32_t last_activity_ms = 0;

// Segment storage
typedef struct { uint16_t dur_ms; uint8_t is_bar; } seg_t;
static seg_t win[9];
static int win_len = 0;

// Decoder state
typedef enum { SCAN_IDLE=0, SCAN_READ } scan_state_t;
static scan_state_t scan_state = SCAN_IDLE;
static char decoded_msg[BARCODE_MAX_STRING];
static bool new_result_available = false;
static barcode_result_t last_result;

// Code-39 pattern table
typedef struct { char ch; const char *pat; } c39_pat_t;
static const c39_pat_t C39[] = {
    {'0',"nnnwwnwnn"}, {'1',"wnnwnnnnw"}, {'2',"nnwwnnnnw"}, {'3',"wnwwnnnnn"},
    {'4',"nnnwwnnnw"}, {'5',"wnnwwnnnn"}, {'6',"nnwwwnnnn"}, {'7',"nnnwnnwnw"},
    {'8',"wnnwnnwnn"}, {'9',"nnwwnnwnn"},
    {'A',"wnnnnwnnw"}, {'B',"nnwnnwnnw"}, {'C',"wnwnnwnnn"}, {'D',"nnnnwwnnw"},
    {'E',"wnnnwwnnn"}, {'F',"nnwnwwnnn"}, {'G',"nnnnnwwnw"}, {'H',"wnnnnwwnn"},
    {'I',"nnwnnwwnn"}, {'J',"nnnnwwwnn"},
    {'K',"wnnnnnnww"}, {'L',"nnwnnnnww"}, {'M',"wnwnnnnwn"}, {'N',"nnnnwnnww"},
    {'O',"wnnnwnnwn"}, {'P',"nnwnwnnwn"}, {'Q',"nnnnnnwww"}, {'R',"wnnnnnwwn"},
    {'S',"nnwnnnwwn"}, {'T',"nnnnwnwwn"},
    {'U',"wwnnnnnnw"}, {'V',"nwwnnnnnw"}, {'W',"wwwnnnnnn"}, {'X',"nwnnwnnnw"},
    {'Y',"wwnnwnnnn"}, {'Z',"nwwnwnnnn"},
    {'-',"nwnnnnwnw"}, {'.',"wwnnnnwnn"}, {' ',"nwwnnnwnn"},
    {'*',"nwnnwnwnn"},
    {0, NULL}
};

// Convert pattern to mask
static uint16_t pat_to_mask(const char *p) {
    uint16_t m = 0;
    for (int i=0; i<9 && p[i]; ++i) 
        if (p[i]=='w'||p[i]=='W') 
            m |= (1u << (8 - i));
    return m;
}

// Build mask from top-3 widest segments
static uint16_t build_mask_top3(const seg_t a[9]) {
    int idx[9]; 
    uint16_t d[9];
    for (int k=0; k<9; k++){ 
        idx[k]=k; 
        d[k]=a[k].dur_ms; 
    }
    
    // Partial selection sort - bring 3 largest to front
    for (int pos=0; pos<3; ++pos) {
        int max_i = pos;
        for (int j=pos+1; j<9; ++j) 
            if (d[j] > d[max_i]) 
                max_i = j;
        if (max_i != pos) {
            uint16_t td=d[pos]; d[pos]=d[max_i]; d[max_i]=td;
            int ti=idx[pos]; idx[pos]=idx[max_i]; idx[max_i]=ti;
        }
    }
    
    // Build 9-bit mask
    uint16_t mask9 = 0;
    mask9 |= (1u << (8 - idx[0]));
    mask9 |= (1u << (8 - idx[1]));
    mask9 |= (1u << (8 - idx[2]));
    return mask9;
}

// Lookup character from mask
static bool lookup_mask(uint16_t mask9, char *out) {
    for (int i=0; C39[i].ch; ++i) {
        if (pat_to_mask(C39[i].pat) == mask9) { 
            *out = C39[i].ch; 
            return true; 
        }
    }
    return false;
}

// Reset window
static void reset_window(void) { 
    win_len = 0; 
}

// Reset scan state
static void reset_scan_state(void) {
    reset_window();
    scan_state = SCAN_IDLE;
    decoded_msg[0] = '\0';
}

// Process segment
static void push_segment(uint16_t dur_ms, bool ended_is_bar) {
    if (dur_ms == 0) return;

    // Start only on a BAR
    if (win_len == 0) {
        if (!ended_is_bar) return;
    } else {
        // Enforce alternation
        bool expect_bar = ((win_len % 2) == 0);
        if (expect_bar != ended_is_bar) {
            reset_window();
            if (ended_is_bar) { 
                win[0] = (seg_t){dur_ms, 1}; 
                win_len = 1; 
            }
            return;
        }
    }

    // Append segment
    win[win_len++] = (seg_t){dur_ms, (uint8_t)ended_is_bar};
    
    if (win_len < 9) return;

    // 9 collected - decode
    char ch = 0;
    uint16_t mask = build_mask_top3(win);
    bool ok = lookup_mask(mask, &ch);
    reset_window();
    
    if (!ok) return;

    if (scan_state == SCAN_IDLE) {
        if (ch == '*') {
            decoded_msg[0] = '\0';
            scan_state = SCAN_READ;
        }
        return;
    }

    // SCAN_READ state
    if (ch == '*') {
        // Stop character - barcode complete
        scan_state = SCAN_IDLE;
        
        // Store result
        strncpy(last_result.decoded_string, decoded_msg, BARCODE_MAX_STRING-1);
        last_result.decoded_string[BARCODE_MAX_STRING-1] = '\0';
        last_result.command = barcode_string_to_command(decoded_msg);
        last_result.valid = (last_result.command != BARCODE_CMD_UNKNOWN);
        last_result.timestamp_ms = to_ms_since_boot(get_absolute_time());
        new_result_available = true;
        
        return;
    }

    // Append payload character
    size_t L = strlen(decoded_msg);
    if (L + 2 < BARCODE_MAX_STRING) { 
        decoded_msg[L] = ch; 
        decoded_msg[L+1] = '\0'; 
    }
}

// Public API Implementation

void barcode_decoder_init(void) {
    gpio_init(IR_PIN);
    gpio_set_dir(IR_PIN, GPIO_IN);
    gpio_pull_up(IR_PIN);

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    raw_level = stable_level = prev_stable_level = gpio_get(IR_PIN);
    last_change_ms = now_ms;
    last_activity_ms = now_ms;
    
    reset_scan_state();
    new_result_available = false;
}

void barcode_decoder_update(void) {
    raw_level = gpio_get(IR_PIN);
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    static uint8_t pending_level = 0xFF;
    static uint32_t pending_since = 0;

    // Quiet edge detection
    if (raw_level == stable_level) { 
        pending_level = 0xFF; 
        return; 
    }
    
    if (pending_level != raw_level) { 
        pending_level = raw_level; 
        pending_since = now_ms; 
        return; 
    }
    
    if ((now_ms - pending_since) < VERIFY_MS) return;

    // Accept change
    prev_stable_level = stable_level;
    stable_level = pending_level;

    uint32_t width_ms = (pending_since > last_change_ms) ? 
                        (pending_since - last_change_ms) : 0;

    // Determine if ended segment was bar or space
    bool ended_was_low = (stable_level > prev_stable_level);
    bool ended_is_bar = BAR_IS_LOW ? ended_was_low : !ended_was_low;

    if (width_ms > 0 && width_ms < 65535) {
        push_segment((uint16_t)width_ms, ended_is_bar);
        last_activity_ms = pending_since;
    }

    last_change_ms = pending_since;
    
    // Inactivity timeout
    if ((now_ms - last_activity_ms) > SCAN_RESET_MS) {
        if (scan_state != SCAN_IDLE || win_len > 0) {
            reset_scan_state();
        }
        last_activity_ms = now_ms;
    }
}

bool barcode_decoder_get_result(barcode_result_t *result) {
    if (!new_result_available || !result) return false;
    
    *result = last_result;
    new_result_available = false;
    return true;
}

void barcode_decoder_reset(void) {
    reset_scan_state();
    new_result_available = false;
}

barcode_command_t barcode_string_to_command(const char *str) {
    if (!str) return BARCODE_CMD_UNKNOWN;
    
    char upper[32];
    int i;
    for (i = 0; i < 31 && str[i]; i++) {
        upper[i] = toupper((unsigned char)str[i]);
    }
    upper[i] = '\0';
    
    // Trim whitespace
    while (i > 0 && isspace((unsigned char)upper[i-1])) {
        upper[--i] = '\0';
    }
    
    if (strcmp(upper, "LEFT") == 0) return BARCODE_CMD_LEFT;
    if (strcmp(upper, "RIGHT") == 0) return BARCODE_CMD_RIGHT;
    if (strcmp(upper, "STOP") == 0) return BARCODE_CMD_STOP;
    if (strcmp(upper, "UTURN") == 0) return BARCODE_CMD_UTURN;
    if (strcmp(upper, "U-TURN") == 0) return BARCODE_CMD_UTURN;
    if (strcmp(upper, "U TURN") == 0) return BARCODE_CMD_UTURN;
    
    return BARCODE_CMD_UNKNOWN;
}

const char* barcode_command_to_string(barcode_command_t cmd) {
    switch (cmd) {
        case BARCODE_CMD_NONE: return "NONE";
        case BARCODE_CMD_LEFT: return "LEFT";
        case BARCODE_CMD_RIGHT: return "RIGHT";
        case BARCODE_CMD_STOP: return "STOP";
        case BARCODE_CMD_UTURN: return "U-TURN";
        case BARCODE_CMD_UNKNOWN: return "UNKNOWN";
        default: return "INVALID";
    }
}