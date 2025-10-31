// ===============================================
//  Barcode Scanner Driver - Implementation
//  Description: Code-39 barcode decoder (FORWARD ONLY)
//  Based on working standalone decoder with debug output
// ===============================================

#include "barcode.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>

// -----------------------------------------------
// Configuration
// -----------------------------------------------
#define VERIFY_MS          2       // Debounce time
#define BAR_IS_LOW         0       // 0: black = HIGH, 1: black = LOW
#define SCAN_RESET_MS      1000    // Reset if no segments for this long
#define DEBUG_SEGMENTS     1       // Print each segment
#define DEBUG_WINDOW       1       // Print window decode attempts

// -----------------------------------------------
// Edge Tracker State
// -----------------------------------------------
static volatile uint8_t  raw_level, stable_level, prev_stable_level;
static volatile uint32_t last_change_ms;
static uint32_t last_activity_ms = 0;

// -----------------------------------------------
// Segment Window (9 elements per character)
// -----------------------------------------------
typedef struct { uint16_t dur_ms; uint8_t is_bar; } seg_t;
static seg_t win[9];
static int win_len = 0;

// -----------------------------------------------
// Scan State
// -----------------------------------------------
typedef enum { SCAN_IDLE = 0, SCAN_READ } scan_state_t;
static scan_state_t scan_state = SCAN_IDLE;
static char msg[128];
static bool result_ready = false;

// -----------------------------------------------
// Code-39 Pattern Table
// -----------------------------------------------
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
    {'*',"nwnnwnwnn"}, {'$',"nwnwnwnnn"}, {'/',"nwnwnnnwn"},
    {'+',"nwnnnwnwn"}, {'%',"nnnwnwnwn"},
    {0, NULL}
};

// -----------------------------------------------
// Pattern Conversion
// -----------------------------------------------
static inline uint16_t pat_to_mask(const char *p) {
    uint16_t m = 0;
    for (int i = 0; i < 9 && p[i]; ++i)
        if (p[i] == 'w' || p[i] == 'W') m |= (1u << (8 - i));
    return m;
}

// -----------------------------------------------
// Top-3 Longest Classification
// -----------------------------------------------
static inline uint16_t build_mask_top3(const seg_t a[9])
{
    int idx[9];
    uint16_t d[9];
    for (int k = 0; k < 9; k++) { idx[k] = k; d[k] = a[k].dur_ms; }
    
    // Bring 3 largest to front
    for (int pos = 0; pos < 3; ++pos) {
        int max_i = pos;
        for (int j = pos + 1; j < 9; ++j)
            if (d[j] > d[max_i]) max_i = j;
        if (max_i != pos) {
            uint16_t td = d[pos]; d[pos] = d[max_i]; d[max_i] = td;
            int ti = idx[pos]; idx[pos] = idx[max_i]; idx[max_i] = ti;
        }
    }
    
    // Build 9-bit mask (MSB = first element, 1 = wide)
    uint16_t mask9 = 0;
    mask9 |= (1u << (8 - idx[0]));
    mask9 |= (1u << (8 - idx[1]));
    mask9 |= (1u << (8 - idx[2]));
    
#if DEBUG_WINDOW
    printf("[BARCODE] Top3 idx: %d %d %d | mask=0x%03X\n", idx[0], idx[1], idx[2], mask9);
#endif
    return mask9;
}

static bool lookup_mask(uint16_t mask9, char *out)
{
    for (int i = 0; C39[i].ch; ++i) {
        if (pat_to_mask(C39[i].pat) == mask9) {
            *out = C39[i].ch;
            return true;
        }
    }
    return false;
}

// -----------------------------------------------
// Window Reset
// -----------------------------------------------
static inline void reset_window(void) {
    win_len = 0;
}

static inline void reset_scan_state(const char *why)
{
    reset_window();
    scan_state = SCAN_IDLE;
    msg[0] = '\0';
    if (why) printf("[BARCODE] Reset: %s\n", why);
}

// -----------------------------------------------
// Segment Processing
// -----------------------------------------------
static void push_segment(uint16_t dur_ms, bool ended_is_bar)
{
    if (dur_ms == 0) return;
    
    // Start only on BAR
    if (win_len == 0) {
        if (!ended_is_bar) return;
    } else {
        // Enforce alternation: B,S,B,S,...,B
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
    
#if DEBUG_SEGMENTS
    printf("[BARCODE] SEG %d: %c %u ms\n", win_len, ended_is_bar ? 'B' : 'S', (unsigned)dur_ms);
#endif
    
    if (win_len < 9) return;
    
    // 9 collected -> decode
    char ch = 0;
    uint16_t mask = build_mask_top3(win);
    
#if DEBUG_WINDOW
    printf("[BARCODE] WIN9: ");
    for (int k = 0; k < 9; k++)
        printf("%c%u ", win[k].is_bar ? 'B' : 'S', win[k].dur_ms);
    printf("\n");
#endif
    
    bool ok = lookup_mask(mask, &ch);
    reset_window();
    
    if (!ok) {
        printf("[BARCODE] Failed to decode mask 0x%03X\n", mask);
        return;
    }
    
    if (scan_state == SCAN_IDLE) {
        if (ch == '*') {
            msg[0] = '\0';
            scan_state = SCAN_READ;
            printf("[BARCODE] *** START (*) ***\n");
        }
        return;
    }
    
    // SCAN_READ
    if (ch == '*') {
        printf("[BARCODE] *** STOP (*) ***\n");
        printf("[BARCODE] *** DECODED: \"%s\" ***\n", msg);
        result_ready = true;
        scan_state = SCAN_IDLE;
        return;
    }
    
    // Append payload char
    size_t L = strlen(msg);
    if (L + 2 < sizeof(msg)) {
        msg[L] = ch;
        msg[L + 1] = '\0';
    }
    
#if DEBUG_WINDOW
    printf("[BARCODE] CHAR: '%c' | buffer=\"%s\"\n", ch, msg);
#endif
}

// -----------------------------------------------
// Initialization
// -----------------------------------------------
void barcode_init(void)
{
    gpio_init(BARCODE_PIN);
    gpio_set_dir(BARCODE_PIN, GPIO_IN);
    gpio_pull_up(BARCODE_PIN);
    
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    raw_level = stable_level = prev_stable_level = gpio_get(BARCODE_PIN);
    last_change_ms = now_ms;
    last_activity_ms = now_ms;
    
    scan_state = SCAN_IDLE;
    msg[0] = '\0';
    result_ready = false;
    win_len = 0;
    
    printf("[BARCODE] Initialized on GP%d (waiting for *...*)\n", BARCODE_PIN);
}

// -----------------------------------------------
// Process (call at 1kHz from FreeRTOS task)
// -----------------------------------------------
void barcode_process(void)
{
    static uint8_t pending_level = 0xFF;
    static uint32_t pending_since = 0;
    
    raw_level = gpio_get(BARCODE_PIN);
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    
    // Debounce logic
    if (raw_level == stable_level) {
        pending_level = 0xFF;
    } else if (pending_level != raw_level) {
        pending_level = raw_level;
        pending_since = now_ms;
    } else if ((now_ms - pending_since) >= VERIFY_MS) {
        // Accept change
        prev_stable_level = stable_level;
        stable_level = pending_level;
        
        uint32_t width_ms = (pending_since > last_change_ms) ? (pending_since - last_change_ms) : 0;
        
        // Map ended level to bar/space
        bool ended_was_low = (stable_level > prev_stable_level);  // rising => LOW ended
        bool ended_is_bar = BAR_IS_LOW ? ended_was_low : !ended_was_low;
        
        if (width_ms > 0 && width_ms < 65535) {
            push_segment((uint16_t)width_ms, ended_is_bar);
            last_activity_ms = pending_since;
        }
        
        last_change_ms = pending_since;
        pending_level = 0xFF;
    }
    
    // Inactivity timeout
    if ((now_ms - last_activity_ms) > SCAN_RESET_MS) {
        if (scan_state != SCAN_IDLE || win_len > 0) {
            reset_scan_state("inactivity timeout");
        }
        last_activity_ms = now_ms;
    }
}

// -----------------------------------------------
// API Functions
// -----------------------------------------------
bool barcode_scan_available(void)
{
    return result_ready;
}

char barcode_get_result(void)
{
    if (strlen(msg) > 0) {
        return msg[0];  // Return first character
    }
    return 0;
}

void barcode_clear(void)
{
    result_ready = false;
    msg[0] = '\0';
}