// ===============================================
//  Module: Barcode Decoder (Code-39)
//  Description: Edge-based barcode scanning for junction commands
//  Based on: Edge tracker + Code-39 decoder (FORWARD ONLY)
// ===============================================
#include "barcode/barcode.h"
#include <stdio.h>
#include <string.h>

// ===== Edge detection state =====
typedef enum { EDGE_NONE=0, EDGE_RISE, EDGE_FALL } edge_t;

static volatile uint8_t  raw_level, stable_level, prev_stable_level;
static volatile uint32_t last_change_ms;
static volatile edge_t   last_edge = EDGE_NONE;

// ===== Segment structure =====
typedef struct { 
    uint16_t dur_ms; 
    uint8_t is_bar; 
} seg_t;

static seg_t win[9];
static int   win_len = 0;

// ===== Scanner state =====
typedef enum { SCAN_IDLE=0, SCAN_READ } scan_state_t;

static scan_state_t scan_state = SCAN_IDLE;
static char         decoded_msg[BARCODE_MAX_LEN];
static uint32_t     last_activity_ms = 0;
static bool         scanning_active = false;

static barcode_callback_t user_callback = NULL;

// ===== Code-39 pattern table =====
typedef struct { char ch; const char *pat; } c39_pat_t;
static const c39_pat_t C39[] = {
    // Digits
    {'0',"nnnwwnwnn"}, {'1',"wnnwnnnnw"}, {'2',"nnwwnnnnw"}, {'3',"wnwwnnnnn"},
    {'4',"nnnwwnnnw"}, {'5',"wnnwwnnnn"}, {'6',"nnwwwnnnn"}, {'7',"nnnwnnwnw"},
    {'8',"wnnwnnwnn"}, {'9',"nnwwnnwnn"},
    // Letters A–Z
    {'A',"wnnnnwnnw"}, {'B',"nnwnnwnnw"}, {'C',"wnwnnwnnn"}, {'D',"nnnnwwnnw"},
    {'E',"wnnnwwnnn"}, {'F',"nnwnwwnnn"}, {'G',"nnnnnwwnw"}, {'H',"wnnnnwwnn"},
    {'I',"nnwnnwwnn"}, {'J',"nnnnwwwnn"},
    {'K',"wnnnnnnww"}, {'L',"nnwnnnnww"}, {'M',"wnwnnnnwn"}, {'N',"nnnnwnnww"},
    {'O',"wnnnwnnwn"}, {'P',"nnwnwnnwn"}, {'Q',"nnnnnnwww"}, {'R',"wnnnnnwwn"},
    {'S',"nnwnnnwwn"}, {'T',"nnnnwnwwn"},
    {'U',"wwnnnnnnw"}, {'V',"nwwnnnnnw"}, {'W',"wwwnnnnnn"}, {'X',"nwnnwnnnw"},
    {'Y',"wwnnwnnnn"}, {'Z',"nwwnwnnnn"},
    // Punctuation
    {'-',"nwnnnnwnw"}, {'.',"wwnnnnwnn"}, {' ',"nwwnnnwnn"},
    {'*',"nwnnwnwnn"}, {'$',"nwnwnwnnn"}, {'/',"nwnwnnnwn"},
    {'+', "nwnnnwnwn"}, {'%',"nnnwnwnwn"},
    {0, NULL}
};

// ===== Helper functions =====
static inline void reset_window(void) { 
    win_len = 0; 
}

static inline uint16_t pat_to_mask(const char *p) {
    uint16_t m = 0;
    for (int i=0; i<9 && p[i]; ++i) 
        if (p[i]=='w'||p[i]=='W') 
            m |= (1u << (8 - i));
    return m;
}

static inline uint16_t build_mask_top3_from(const seg_t a[9]) {
    int idx[9]; 
    uint16_t d[9];
    
    for (int k=0; k<9; k++) { 
        idx[k] = k; 
        d[k] = a[k].dur_ms; 
    }
    
    // Partial selection sort - bring top 3 to front
    for (int pos=0; pos<3; ++pos) {
        int max_i = pos;
        for (int j=pos+1; j<9; ++j) 
            if (d[j] > d[max_i]) 
                max_i = j;
        
        if (max_i != pos) {
            uint16_t td = d[pos]; 
            d[pos] = d[max_i]; 
            d[max_i] = td;
            
            int ti = idx[pos];    
            idx[pos] = idx[max_i]; 
            idx[max_i] = ti;
        }
    }
    
    // Build 9-bit mask (MSB = first element; 1 = wide)
    uint16_t mask9 = 0;
    mask9 |= (1u << (8 - idx[0]));
    mask9 |= (1u << (8 - idx[1]));
    mask9 |= (1u << (8 - idx[2]));
    
    return mask9;
}

static bool lookup_mask(uint16_t mask9, char *out) {
    for (int i=0; C39[i].ch; ++i) {
        if (pat_to_mask(C39[i].pat) == mask9) { 
            *out = C39[i].ch; 
            return true; 
        }
    }
    return false;
}

static inline void reset_scan_state(const char *why) {
    reset_window();
    scan_state = SCAN_IDLE;
    decoded_msg[0] = '\0';
    if (why) {
        printf("[BARCODE] Reset: %s\n", why);
    }
}

// ===== Segment processing =====
static void push_segment(uint16_t dur_ms, bool ended_is_bar) {
    if (dur_ms == 0) return;

    // Start only on a BAR
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
    
    if (win_len < 9) return;

    // 9 segments collected → classify
    char ch = 0;
    uint16_t mask = build_mask_top3_from(win);
    bool ok = lookup_mask(mask, &ch);
    reset_window();
    
    if (!ok) return;

    // State machine
    if (scan_state == SCAN_IDLE) {
        if (ch == '*') {
            decoded_msg[0] = '\0';
            scan_state = SCAN_READ;
            printf("[BARCODE] START detected\n");
        }
        return;
    }

    // SCAN_READ state
    if (ch == '*') {
        printf("[BARCODE] STOP detected\n");
        printf("[BARCODE] Decoded: \"%s\"\n", decoded_msg);
        
        // Trigger callback
        if (user_callback) {
            barcode_command_t cmd = barcode_parse_command(decoded_msg);
            user_callback(decoded_msg, cmd);
        }
        
        scan_state = SCAN_IDLE;
        return;
    }

    // Append character
    size_t L = strlen(decoded_msg);
    if (L + 2 < sizeof(decoded_msg)) { 
        decoded_msg[L] = ch; 
        decoded_msg[L+1] = '\0'; 
    }
}

// ===== Timer callback (1kHz sampling) =====
static bool timer_cb(struct repeating_timer *t) {
    if (!scanning_active) return true;
    
    raw_level = gpio_get(BARCODE_IR_PIN);
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    static uint8_t  pending_level = 0xFF;
    static uint32_t pending_since = 0;

    // Debounce logic
    if (raw_level == stable_level) { 
        pending_level = 0xFF; 
        return true; 
    }
    
    if (pending_level != raw_level) { 
        pending_level = raw_level; 
        pending_since = now_ms; 
        return true; 
    }
    
    if ((now_ms - pending_since) < BARCODE_VERIFY_MS) 
        return true;

    // Accept change
    prev_stable_level = stable_level;
    stable_level = pending_level;

    uint32_t width_ms = (pending_since > last_change_ms) ? 
                        (pending_since - last_change_ms) : 0;

    // Map ended level to bar/space
    bool ended_was_low = (stable_level > prev_stable_level);
    bool ended_is_bar  = BARCODE_BAR_IS_LOW ? ended_was_low : !ended_was_low;

    if (width_ms > 0 && width_ms < 65535) {
        push_segment((uint16_t)width_ms, ended_is_bar);
        last_activity_ms = pending_since;
    }

    last_change_ms = pending_since;
    return true;
}

static struct repeating_timer barcode_timer;

// ===== Public API =====
void barcode_init(void) {
    gpio_init(BARCODE_IR_PIN);
    gpio_set_dir(BARCODE_IR_PIN, GPIO_IN);
    gpio_pull_up(BARCODE_IR_PIN);

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    raw_level = stable_level = prev_stable_level = gpio_get(BARCODE_IR_PIN);
    last_change_ms = now_ms;
    last_activity_ms = now_ms;
    
    decoded_msg[0] = '\0';
    
    printf("[BARCODE] Initialized on GPIO %d\n", BARCODE_IR_PIN);
}

void barcode_start_scanning(void) {
    if (scanning_active) return;
    
    scanning_active = true;
    reset_scan_state("start scanning");
    add_repeating_timer_ms(-BARCODE_SAMPLE_MS, timer_cb, NULL, &barcode_timer);
    
    printf("[BARCODE] Scanning started\n");
}

void barcode_stop_scanning(void) {
    if (!scanning_active) return;
    
    scanning_active = false;
    cancel_repeating_timer(&barcode_timer);
    
    printf("[BARCODE] Scanning stopped\n");
}

void barcode_set_callback(barcode_callback_t callback) {
    user_callback = callback;
}

const char* barcode_get_last_decoded(void) {
    return decoded_msg;
}

barcode_command_t barcode_parse_command(const char* str) {
    if (!str || !str[0]) return CMD_NONE;
    
    // Match against expected command strings
    if (strcmp(str, "LEFT") == 0 || strcmp(str, "L") == 0) 
        return CMD_LEFT;
    if (strcmp(str, "RIGHT") == 0 || strcmp(str, "R") == 0) 
        return CMD_RIGHT;
    if (strcmp(str, "STOP") == 0 || strcmp(str, "S") == 0) 
        return CMD_STOP;
    if (strcmp(str, "FORWARD") == 0 || strcmp(str, "F") == 0) 
        return CMD_FORWARD;
    
    return CMD_NONE;
}

void barcode_update(void) {
    // Check for inactivity timeout
    if (!scanning_active) return;
    
    uint32_t tnow = to_ms_since_boot(get_absolute_time());
    if ((tnow - last_activity_ms) > BARCODE_RESET_MS) {
        if (scan_state != SCAN_IDLE || win_len > 0) {
            reset_scan_state("inactivity timeout");
        }
        last_activity_ms = tnow;
    }
}
