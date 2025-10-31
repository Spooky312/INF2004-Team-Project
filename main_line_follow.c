// ===============================================
//  Project: Robotic Car - Line Following + Barcode
//  Description: Follows black line, reads barcodes,
//               executes turns based on decoded letters.
//               A,C,E,G,I,K,M,O,Q,S,U,W,Y = Turn Right
//               B,D,F,H,J,L,N,P,R,T,V,X,Z = Turn Left
// ===============================================

#include <stdio.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "debug_led.h"
#include "line_sensor.h"
#include "barcode.h"

// -----------------------------------------------
// Task Configuration
// -----------------------------------------------
#define LINE_FOLLOW_TASK_PRIORITY   (tskIDLE_PRIORITY + 3)
#define BARCODE_TASK_PRIORITY       (tskIDLE_PRIORITY + 2)
#define TELEMETRY_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)

#define LINE_FOLLOW_PERIOD_MS       20      // 50 Hz
#define BARCODE_PERIOD_MS           1       // 1 kHz for barcode sampling
#define TELEMETRY_PERIOD_MS         500     // 2 Hz

// -----------------------------------------------
// Control Button Configuration
// -----------------------------------------------
#define BUTTON_PIN                  21      // GP21 for start/stop button
#define BUTTON_DEBOUNCE_MS          200     // Debounce delay

// -----------------------------------------------
// Movement Configuration
// -----------------------------------------------
#define BASE_SPEED              80.0f  // Base speed for line following (increased from 60)
#define TURN_SPEED              50.0f  // Speed during 90° turns (increased from 50)
#define TURN_DURATION_MS        1000    // Time for 90° turn (tune this)
#define POST_TURN_DELAY_MS      200     // Stabilization delay after turn

// PID gains for line following
#define LINE_KP                 0.8f
#define LINE_KI                 0.0f
#define LINE_KD                 0.2f

// -----------------------------------------------
// State Machine
// -----------------------------------------------
typedef enum {
    STATE_IDLE = 0,
    STATE_LINE_FOLLOW,
    STATE_BARCODE_DETECTED,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_RESUME_LINE
} robot_state_t;

static volatile robot_state_t robot_state = STATE_IDLE;
static volatile char last_barcode = 0;
static volatile bool button_pressed = false;
static volatile uint32_t last_button_time = 0;

// Line following PID state
static float line_integral = 0.0f;
static float line_prev_error = 0.0f;

// -----------------------------------------------
// Button IRQ Handler
// -----------------------------------------------
static void button_irq_handler(uint gpio, uint32_t events)
{
    if (gpio != BUTTON_PIN) return;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Debounce
    if ((now - last_button_time) < BUTTON_DEBOUNCE_MS) {
        return;
    }
    last_button_time = now;
    
    // Toggle between IDLE and LINE_FOLLOW
    if (robot_state == STATE_IDLE) {
        robot_state = STATE_LINE_FOLLOW;
        printf("[BUTTON] START - Entering LINE_FOLLOW mode\n");
    } else {
        robot_state = STATE_IDLE;
        motor_set_speed(0, 0);
        printf("[BUTTON] STOP - Entering IDLE mode\n");
    }
    
    button_pressed = true;
}

// -----------------------------------------------
// Helper: Determine turn direction from barcode
// -----------------------------------------------
static bool should_turn_right(char ch)
{
    ch = toupper(ch);
    // A,C,E,G,I,K,M,O,Q,S,U,W,Y = Right
    return (ch == 'A' || ch == 'C' || ch == 'E' || ch == 'G' || 
            ch == 'I' || ch == 'K' || ch == 'M' || ch == 'O' || 
            ch == 'Q' || ch == 'S' || ch == 'U' || ch == 'W' || ch == 'Y');
}

static bool should_turn_left(char ch)
{
    ch = toupper(ch);
    // B,D,F,H,J,L,N,P,R,T,V,X,Z = Left
    return (ch == 'B' || ch == 'D' || ch == 'F' || ch == 'H' || 
            ch == 'J' || ch == 'L' || ch == 'N' || ch == 'P' || 
            ch == 'R' || ch == 'T' || ch == 'V' || ch == 'X' || ch == 'Z');
}

// -----------------------------------------------
// Line Following PID
// -----------------------------------------------
static float line_pid_compute(float error)
{
    line_integral += error;
    float derivative = error - line_prev_error;
    line_prev_error = error;
    
    float output = (LINE_KP * error) + 
                   (LINE_KI * line_integral) + 
                   (LINE_KD * derivative);
    
    // Anti-windup
    if (line_integral > 100.0f) line_integral = 100.0f;
    if (line_integral < -100.0f) line_integral = -100.0f;
    
    // Clamp output
    if (output > 50.0f) output = 50.0f;
    if (output < -50.0f) output = -50.0f;
    
    return output;
}

static void line_pid_reset(void)
{
    line_integral = 0.0f;
    line_prev_error = 0.0f;
}

// -----------------------------------------------
// Line Following Task
// -----------------------------------------------
static void line_follow_task(void *p)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t turn_start_time = 0;
    static int loop_count = 0;
    
    printf("[LINE] Line following task started (waiting for button press)\n");
    
    // Robot starts in IDLE - user must press button to start
    // robot_state is already set to STATE_IDLE in the global declaration
    
    for (;;)
    {
        switch (robot_state)
        {
            case STATE_IDLE:
                motor_set_speed(0, 0);
                break;
                
            case STATE_LINE_FOLLOW:
            {
                // Read line sensor
                bool on_line = line_sensor_on_line();
                uint16_t raw = line_sensor_read_raw();
                
                if (!on_line) {
                    // Lost line - stop and report
                    motor_set_speed(0, 0);
                    if (++loop_count >= 25) {  // Every 500ms
                        printf("[LINE] LINE LOST! Raw=%u\n", raw);
                        loop_count = 0;
                    }
                    debug_led_set(24, false);  // Encoder LED off = line lost
                    break;
                }
                
                debug_led_set(24, true);  // Encoder LED on = line detected
                
                // Simple line following: single sensor means binary control
                // For better performance, use multiple sensors in an array
                // For now: assume centered when on line
                float line_error = 0.0f;  // Centered
                
                float correction = line_pid_compute(line_error);
                
                // Apply correction to motors (similar to heading PID)
                float left_speed = BASE_SPEED + correction;
                float right_speed = BASE_SPEED - correction;
                
                motor_set_speed(left_speed, right_speed);
                
                // Debug output - MORE VERBOSE
                if (++loop_count >= 25) {  // Every 500ms
                    printf("[LINE] Following | Raw=%u | L=%.1f R=%.1f | MOVING!\n", 
                           raw, left_speed, right_speed);
                    loop_count = 0;
                }
                break;
            }
            
            case STATE_BARCODE_DETECTED:
            {
                // Stop and prepare for turn
                motor_set_speed(0, 0);
                printf("[LINE] BARCODE STOP - Barcode: '%c'\n", last_barcode);
                
                // Determine turn direction
                if (should_turn_right(last_barcode)) {
                    robot_state = STATE_TURN_RIGHT;
                    printf("[LINE] -> Turning RIGHT\n");
                } else if (should_turn_left(last_barcode)) {
                    robot_state = STATE_TURN_LEFT;
                    printf("[LINE] -> Turning LEFT\n");
                } else {
                    // Invalid barcode, resume line following
                    printf("[LINE] -> Invalid barcode, resuming\n");
                    robot_state = STATE_LINE_FOLLOW;
                }
                
                turn_start_time = to_ms_since_boot(get_absolute_time());
                vTaskDelay(pdMS_TO_TICKS(300));  // Brief pause
                break;
            }
            
            case STATE_TURN_LEFT:
            {
                uint32_t now = to_ms_since_boot(get_absolute_time());
                uint32_t elapsed = now - turn_start_time;
                
                if (elapsed < TURN_DURATION_MS) {
                    // Execute left turn: left slower, right faster
                    motor_set_speed(0, TURN_SPEED);
                    debug_led_set(13, true);  // Turn indicator
                } else {
                    // Turn complete
                    motor_set_speed(0, 0);
                    debug_led_set(13, false);
                    printf("[LINE] Left turn complete\n");
                    robot_state = STATE_RESUME_LINE;
                    vTaskDelay(pdMS_TO_TICKS(POST_TURN_DELAY_MS));
                }
                break;
            }
            
            case STATE_TURN_RIGHT:
            {
                uint32_t now = to_ms_since_boot(get_absolute_time());
                uint32_t elapsed = now - turn_start_time;
                
                if (elapsed < TURN_DURATION_MS) {
                    // Execute right turn: left faster, right slower
                    motor_set_speed(TURN_SPEED, 0);
                    debug_led_set(13, true);  // Turn indicator
                } else {
                    // Turn complete
                    motor_set_speed(0, 0);
                    debug_led_set(13, false);
                    printf("[LINE] Right turn complete\n");
                    robot_state = STATE_RESUME_LINE;
                    vTaskDelay(pdMS_TO_TICKS(POST_TURN_DELAY_MS));
                }
                break;
            }
            
            case STATE_RESUME_LINE:
            {
                // Reset PID and resume line following
                line_pid_reset();
                last_barcode = 0;
                robot_state = STATE_LINE_FOLLOW;
                printf("[LINE] Resuming line following\n");
                break;
            }
        }
        
        vTaskDelayUntil(&last, pdMS_TO_TICKS(LINE_FOLLOW_PERIOD_MS));
    }
}

// -----------------------------------------------
// Barcode Scanning Task
// -----------------------------------------------
static void barcode_task(void *p)
{
    TickType_t last = xTaskGetTickCount();
    
    printf("[BARCODE] Barcode scanning task started\n");
    
    for (;;)
    {
        // Process barcode scanner (1kHz sampling)
        barcode_process();
        
        // Check if barcode was decoded
        if (barcode_scan_available()) {
            char decoded = barcode_get_result();
            barcode_clear();
            
            // Always update last_barcode for telemetry display
            last_barcode = decoded;
            printf("[BARCODE] *** DETECTED: '%c' ***\n", decoded);
            
            // Only trigger state change if we're currently following line
            if (robot_state == STATE_LINE_FOLLOW) {
                robot_state = STATE_BARCODE_DETECTED;
                printf("[BARCODE] -> Triggering turn decision\n");
            } else {
                printf("[BARCODE] -> Robot not running, barcode saved for telemetry\n");
            }
        }
        
        vTaskDelayUntil(&last, pdMS_TO_TICKS(BARCODE_PERIOD_MS));
    }
}

// -----------------------------------------------
// Telemetry Task
// -----------------------------------------------
static void telemetry_task(void *p)
{
    TickType_t last = xTaskGetTickCount();
    
    printf("[TELEM] Telemetry task started\n");
    
    const char* state_names[] = {
        "IDLE", "LINE_FOLLOW", "BARCODE_DETECTED", 
        "TURN_LEFT", "TURN_RIGHT", "RESUME_LINE"
    };
    
    for (;;)
    {
        // Get sensor data
        uint16_t line_raw = line_sensor_read_raw();
        bool on_line = line_sensor_on_line();
        
        float rpm_l = encoder_get_rpm_left();
        float rpm_r = encoder_get_rpm_right();
        float dist = encoder_get_distance_m();
        
        uint32_t ticks_l = 0, ticks_r = 0;
        encoder_get_ticks(&ticks_l, &ticks_r);
        
        // Build telemetry JSON
        char msg[256];
        snprintf(msg, sizeof(msg),
                 "{\"state\":\"%s\",\"line_raw\":%u,\"on_line\":%d,"
                 "\"barcode\":\"%c\",\"rpm_l\":%.1f,\"rpm_r\":%.1f,"
                 "\"dist\":%.2f,\"ticks_l\":%lu,\"ticks_r\":%lu}",
                 state_names[robot_state], line_raw, on_line,
                 last_barcode ? last_barcode : '-',
                 rpm_l, rpm_r, dist, ticks_l, ticks_r);
        
        printf("[TELEM] %s\n", msg);
        
        // Heartbeat LED
        debug_led_blink(25, 100);
        
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

// -----------------------------------------------
// Main Entry
// -----------------------------------------------
int main(void)
{
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n");
    printf("================================================\n");
    printf("  Robotic Car - Line Following + Barcode Demo\n");
    printf("================================================\n");
    printf("Right Turn: A,C,E,G,I,K,M,O,Q,S,U,W,Y\n");
    printf("Left Turn:  B,D,F,H,J,L,N,P,R,T,V,X,Z\n");
    printf("================================================\n");
    printf("Press button (GP21) to START/STOP robot\n");
    printf("================================================\n\n");
    
    // Initialize drivers
    debug_led_init();
    motor_init();
    encoder_init();
    imu_init();  // Keep for future obstacle avoidance
    line_sensor_init();
    barcode_init();
    
    // Initialize button
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);  // Active low (press = low)
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &button_irq_handler);
    printf("[BUTTON] Control button initialized on GP%d\n", BUTTON_PIN);
    printf("[BUTTON] Robot starts in IDLE mode - press button to start\n\n");
    
    printf("\n[INFO] Creating FreeRTOS tasks...\n");
    xTaskCreate(line_follow_task, "LineFollow", 2048, NULL, LINE_FOLLOW_TASK_PRIORITY, NULL);
    xTaskCreate(barcode_task, "Barcode", 2048, NULL, BARCODE_TASK_PRIORITY, NULL);
    xTaskCreate(telemetry_task, "Telemetry", 2048, NULL, TELEMETRY_TASK_PRIORITY, NULL);
    
    printf("[INFO] Starting FreeRTOS scheduler...\n");
    printf("========================================\n\n");
    
    vTaskStartScheduler();
    
    // Should never reach here
    printf("ERROR: Scheduler failed to start!\n");
    while (1) {
        sleep_ms(1000);
    }
}
