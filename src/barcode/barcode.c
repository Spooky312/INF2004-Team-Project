/**
 * @file barcode.c
 * @brief Code-39 Barcode Decoder Implementation (FORWARD ONLY)
 * Improved edge tracker + Code-39 decoder with visual START/DATA/STOP display
 */

#include "barcode.h"
#include "line_sensor.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// Configuration from robot_config.h
#define IR_PIN BARCODE_IR_DO_PIN
#define SAMPLE_MS BARCODE_SAMPLE_MS
#define VERIFY_MS BARCODE_VERIFY_MS
#define BAR_IS_LOW BARCODE_BAR_IS_LOW
#define SCAN_RESET_MS BARCODE_RESET_MS
#define MAX_MESSAGE_LEN BARCODE_MAX_LEN

// Debug output control
#define DEBUG_SEGMENTS     1       // Show each segment (Bar/Space + duration)
#define DEBUG_WINDOW_DUMP  1       // Dump 9-segment window & classification

// Scanner state machine
typedef enum {
    SCAN_IDLE = 0,    // Waiting for start delimiter (*)
    SCAN_READ         // Reading barcode data
} scan_state_t;

// Barcode segment (bar or space with duration)
typedef struct { 
    uint16_t dur_ms;   // Duration in milliseconds
    uint8_t is_bar;    // 1 if bar (black), 0 if space (white)
} seg_t;

// Edge detection state
static volatile uint8_t raw_level = 0;
static volatile uint8_t stable_level = 0;
static volatile uint8_t prev_stable_level = 0;
static volatile uint32_t last_change_ms = 0;
static volatile uint32_t last_activity_ms = 0;
static uint32_t last_status_ms = 0;

// Segment storage (9-element sliding window)
static seg_t win[9];
static int win_len = 0;

// Warmup counter - skip initial segments to let car reach stable speed
static int warmup_segments_remaining = 0;
#define WARMUP_SKIP_COUNT 4  // Skip first 4 segments (usually 2 bars + 2 spaces)

// Decoder state
static scan_state_t scan_state = SCAN_IDLE;
static char decoded_msg[MAX_MESSAGE_LEN];
static bool new_result_available = false;
static barcode_result_t last_result;

// Callback support
static barcode_callback_t user_callback = NULL;
static bool scanning_enabled = false;

// Speed control support - track narrow durations for autonomous speed adjustment
#define NARROW_HISTORY_SIZE 10
static uint16_t narrow_durations[NARROW_HISTORY_SIZE];
static int narrow_count = 0;
static float module_width_m = 0.0f;  // Calibrated module width in meters

// Timer for periodic updates
static struct repeating_timer timer;
static bool reader_active = false;

// Forward declarations
static const char* barcode_command_to_string(barcode_command_t cmd);

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
    warmup_segments_remaining = WARMUP_SKIP_COUNT;  // Reset warmup counter
}

// Process segment
static void push_segment(uint16_t dur_ms, bool ended_is_bar) {
    if (dur_ms == 0) return;
    
    // **WARMUP: Skip first few segments to let car reach stable speed**
    if (warmup_segments_remaining > 0) {
        warmup_segments_remaining--;
        #if DEBUG_SEGMENTS
        printf("WARMUP: Skipping %c %u ms (%d remaining)\n", 
               ended_is_bar ? 'B' : 'S', (unsigned)dur_ms, warmup_segments_remaining);
        #endif
        return;
    }
    
    // **REJECT SEGMENTS THAT ARE TOO LONG** (likely not barcode data)
    // At BASE_SPEED=0.4, typical narrow should be 20-80ms, wide should be 60-200ms
    // Anything over 300ms is probably car stopped/paused or off barcode
    const uint16_t MAX_SEGMENT_MS = 300;
    if (dur_ms > MAX_SEGMENT_MS) {
        #if DEBUG_SEGMENTS
        printf("SEG REJECT: %c %u ms (too long, max %u ms)\n", 
               ended_is_bar ? 'B' : 'S', (unsigned)dur_ms, MAX_SEGMENT_MS);
        #endif
        reset_window();  // Reset and wait for valid data
        return;
    }

    // Start only on a BAR
    if (win_len == 0) {
        if (!ended_is_bar) return;
    } else {
        // Enforce alternation: B,S,B,S,...,B
        bool expect_bar = ((win_len % 2) == 0);
        if (expect_bar != ended_is_bar) {
            // Alternation broke: resync. Start new window if this is a BAR.
            reset_window();
            if (ended_is_bar) { 
                win[0] = (seg_t){dur_ms, 1}; 
                win_len = 1; 
            }
            return;
        }
    }

    // Append segment to window
    win[win_len++] = (seg_t){dur_ms, (uint8_t)ended_is_bar};
    
#if DEBUG_SEGMENTS
    printf("SEG %d: %c %u ms\n", win_len, ended_is_bar ? 'B' : 'S', (unsigned)dur_ms);
#endif
    
    if (win_len < 9) return;

    // 9 segments collected -> validate ratios before classification
    // Find narrowest 6 elements (should be the 'n' narrow ones in Code-39)
    uint16_t sorted_durs[9];
    for (int k = 0; k < 9; k++) sorted_durs[k] = win[k].dur_ms;
    // Simple bubble sort for 9 elements
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8 - i; j++) {
            if (sorted_durs[j] > sorted_durs[j+1]) {
                uint16_t tmp = sorted_durs[j];
                sorted_durs[j] = sorted_durs[j+1];
                sorted_durs[j+1] = tmp;
            }
        }
    }
    
    // Average the narrowest 6 (excluding the 3 wide ones)
    uint32_t narrow_sum = 0;
    for (int k = 0; k < 6; k++) narrow_sum += sorted_durs[k];
    uint16_t avg_narrow = (uint16_t)(narrow_sum / 6);
    
    // Average the widest 3
    uint32_t wide_sum = 0;
    for (int k = 6; k < 9; k++) wide_sum += sorted_durs[k];
    uint16_t avg_wide = (uint16_t)(wide_sum / 3);
    
    // **VALIDATE WIDE/NARROW RATIO** (Code-39 spec: wide should be 2.2-3.0x narrow)
    // Allow 1.8-4.0x range for tolerance
    float ratio = (float)avg_wide / (float)avg_narrow;
    if (ratio < 1.8f || ratio > 4.0f) {
        #if DEBUG_WINDOW_DUMP
        printf("WIN9 REJECT: narrow_avg=%u wide_avg=%u ratio=%.2f (out of range 1.8-4.0)\n",
               avg_narrow, avg_wide, ratio);
        #endif
        reset_window();
        return;
    }
    
    // Classify using top-3 mask
    char ch = 0;
    uint16_t mask = build_mask_top3(win);
    
    // Store narrow average in circular buffer for speed control
    if (narrow_count < NARROW_HISTORY_SIZE) {
        narrow_durations[narrow_count++] = avg_narrow;
    } else {
        // Shift and add new
        for (int k = 0; k < NARROW_HISTORY_SIZE - 1; k++) {
            narrow_durations[k] = narrow_durations[k + 1];
        }
        narrow_durations[NARROW_HISTORY_SIZE - 1] = avg_narrow;
    }
    
#if DEBUG_WINDOW_DUMP
    printf("WIN9: ");
    for (int k = 0; k < 9; k++) {
        printf("%c%u ", win[k].is_bar ? 'B' : 'S', win[k].dur_ms);
    }
    printf("| W/n: ");
    for (int i = 0; i < 9; i++) {
        bool is_wide = (mask & (1u << (8 - i))) != 0;
        printf("%c", is_wide ? 'W' : 'n');
    }
    printf(" | n=%u w=%u ratio=%.2f -> ", avg_narrow, avg_wide, ratio);
#endif
    
    bool ok = lookup_mask(mask, &ch);
    reset_window();
    
    if (!ok) {
#if DEBUG_WINDOW_DUMP
        printf("✗ (no match)\n");
#endif
        return;
    }

    // Handle start delimiter
    if (scan_state == SCAN_IDLE) {
        if (ch == '*') {
            decoded_msg[0] = '\0';
            scan_state = SCAN_READ;
            printf("\n");
            printf("╔════════════════════════════════════════════╗\n");
            printf("║  🔍 BARCODE SCAN STARTED                   ║\n");
            printf("╠════════════════════════════════════════════╣\n");
            printf("║  START: *                                  ║\n");
            printf("╚════════════════════════════════════════════╝\n");
        }
        return;
    }

    // Handle stop delimiter and payload characters
    if (ch == '*') {
        printf("╔════════════════════════════════════════════╗\n");
        printf("║  STOP: *                                   ║\n");
        printf("╠════════════════════════════════════════════╣\n");
        printf("║  ✅ COMPLETE BARCODE: \"%s\"%*s║\n", 
               decoded_msg, (int)(24 - strlen(decoded_msg)), "");
        printf("╚════════════════════════════════════════════╝\n");
        printf("\n");
        
        // Store result
        strncpy(last_result.decoded_string, decoded_msg, MAX_MESSAGE_LEN-1);
        last_result.decoded_string[MAX_MESSAGE_LEN-1] = '\0';
        last_result.command = barcode_parse_command(decoded_msg);
        last_result.valid = (last_result.command != CMD_NONE);
        last_result.timestamp_ms = to_ms_since_boot(get_absolute_time());
        new_result_available = true;
        
        printf("[BARCODE] Command: %s\n\n", barcode_command_to_string(last_result.command));
        
        // Call callback if registered
        if (user_callback && scanning_enabled) {
            user_callback(last_result.decoded_string, last_result.command);
        }
        
        scan_state = SCAN_IDLE;
        return;
    }

    // Append payload character
    size_t L = strlen(decoded_msg);
    if (L + 2 < sizeof(decoded_msg)) {
        decoded_msg[L] = ch;
        decoded_msg[L + 1] = '\0';
    }
    
#if DEBUG_WINDOW_DUMP
    printf("'%c'  DATA=\"%s\"\n", ch, decoded_msg);
#endif
}

// Public API Implementation

void barcode_init(void) {
    gpio_init(IR_PIN);
    gpio_set_dir(IR_PIN, GPIO_IN);
    gpio_pull_up(IR_PIN);

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    raw_level = stable_level = prev_stable_level = gpio_get(IR_PIN);
    last_change_ms = now_ms;
    last_activity_ms = now_ms;
    last_status_ms = now_ms;
    
    reset_scan_state();
    new_result_available = false;
    
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║  Barcode Reader Initialized (Code-39)     ║\n");
    printf("║  IR Sensor: GPIO %d                        ║\n", IR_PIN);
    printf("║  Mode: FORWARD ONLY                        ║\n");
    printf("║  Timeout: %d ms (inactivity reset)        ║\n", SCAN_RESET_MS);
    printf("╚════════════════════════════════════════════╝\n\n");
}

void barcode_update(void) {
    if (!scanning_enabled) return;
    
    raw_level = gpio_get(IR_PIN);
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    static uint8_t pending_level = 0xFF;
    static uint32_t pending_since = 0;

    // Quiet edge detection
    if (raw_level == stable_level) { 
        pending_level = 0xFF; 
        
        // Check timeout even when no edges are happening
        // This handles cases where we're stuck in a long segment
        if ((now_ms - last_change_ms) > SCAN_RESET_MS) {
            if (scan_state != SCAN_IDLE) {
                printf("\n╔════════════════════════════════════════════╗\n");
                printf("║  ⏱️  SCAN TIMEOUT (>%dms no edges)        ║\n", SCAN_RESET_MS);
                printf("║  Partial data: \"%s\"%*s║\n", 
                       decoded_msg, (int)(27 - strlen(decoded_msg)), "");
                printf("║  Window: %d/9 segments                     ║\n", win_len);
                printf("║  Resetting scanner...                      ║\n");
                printf("╚════════════════════════════════════════════╝\n\n");
                reset_scan_state();
                last_activity_ms = now_ms;
            } else if (win_len > 0) {
                printf("[BARCODE] Timeout - clearing partial window (%d segments)\n", win_len);
                reset_scan_state();
                last_activity_ms = now_ms;
            }
        }
        
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
    
#ifdef BARCODE_STATUS_MS
    #if BARCODE_STATUS_MS > 0
    // Periodic status update (optional)
    if (scanning_enabled && (now_ms - last_status_ms) > BARCODE_STATUS_MS) {
        printf("[BARCODE] Status: %s | Last edge: %lu ms ago | Window: %d/9 | IR: %d\n",
               scan_state == SCAN_IDLE ? "IDLE" : "READING",
               (unsigned long)(now_ms - last_change_ms),
               win_len,
               stable_level);
        last_status_ms = now_ms;
    }
    #endif
#endif
}

bool barcode_get_result(barcode_result_t *result) {
    if (!new_result_available || !result) return false;
    
    *result = last_result;
    new_result_available = false;
    return true;
}

const char* barcode_get_last_decoded(void) {
    if (decoded_msg[0] == '\0') return NULL;
    return decoded_msg;
}

void barcode_reset(void) {
    reset_scan_state();
    new_result_available = false;
}

barcode_command_t barcode_parse_command(const char *str) {
    if (!str) return CMD_NONE;
    
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
    
    if (strcmp(upper, "LEFT") == 0) return CMD_LEFT;
    if (strcmp(upper, "RIGHT") == 0) return CMD_RIGHT;
    if (strcmp(upper, "STOP") == 0) return CMD_STOP;
    if (strcmp(upper, "UTURN") == 0) return CMD_UTURN;
    if (strcmp(upper, "U-TURN") == 0) return CMD_UTURN;
    if (strcmp(upper, "U TURN") == 0) return CMD_UTURN;
    
    return CMD_NONE;
}

const char* barcode_command_to_string(barcode_command_t cmd) {
    switch (cmd) {
        case CMD_NONE: return "NONE";
        case CMD_LEFT: return "LEFT";
        case CMD_RIGHT: return "RIGHT";
        case CMD_STOP: return "STOP";
        case CMD_UTURN: return "U-TURN";
        default: return "INVALID";
    }
}

// Callback and scanning control functions
void barcode_set_callback(barcode_callback_t callback) {
    user_callback = callback;
}

void barcode_start_scanning(void) {
    scanning_enabled = true;
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    last_activity_ms = now_ms;
    last_status_ms = now_ms;
    printf("[BARCODE] 🟢 Scanning ENABLED - Waiting for barcode...\n\n");
}

void barcode_stop_scanning(void) {
    scanning_enabled = false;
    reset_scan_state();
    printf("[BARCODE] 🔴 Scanning DISABLED\n");
}

// ===== Speed Control API =====

// Get average narrow duration from recent scans (ms)
float barcode_get_avg_narrow_duration_ms(void) {
    if (narrow_count == 0) return 0.0f;
    
    uint32_t sum = 0;
    for (int i = 0; i < narrow_count; i++) {
        sum += narrow_durations[i];
    }
    return (float)sum / (float)narrow_count;
}

// Set calibrated module width (call once during calibration)
void barcode_set_module_width_m(float width_m) {
    module_width_m = width_m;
    printf("[BARCODE] Module width set to %.5f m (%.2f mm)\n", width_m, width_m * 1000.0f);
}

// Get calibrated module width
float barcode_get_module_width_m(void) {
    return module_width_m;
}

// Compute target RPM for desired narrow duration (ms)
// Returns 0 if not calibrated or invalid input
float barcode_compute_target_rpm(float desired_narrow_ms) {
    if (module_width_m <= 0.0f || desired_narrow_ms <= 0.0f) {
        return 0.0f;
    }
    
    // v_target = module_width / (desired_narrow / 1000)
    float v_target_mps = module_width_m * 1000.0f / desired_narrow_ms;
    
    // rpm = v * 60 / wheel_circumference
    float rpm_target = v_target_mps * 60.0f / WHEEL_CIRCUM_M;
    
    return rpm_target;
}

// Clear narrow duration history
void barcode_reset_narrow_history(void) {
    narrow_count = 0;
    printf("[BARCODE] Narrow duration history cleared\n");
}

// Get number of narrow samples collected
int barcode_get_narrow_sample_count(void) {
    return narrow_count;
}