// ===============================================
//  Demo 2: Line Following + Barcode Navigation + Telemetry
//  Description: Full perception & decision loop with FreeRTOS
// ===============================================
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Hardware drivers
#include "drivers/motor/motor.h"
#include "drivers/encoder/encoder.h"
#include "drivers/imu/imu.h"
#include "drivers/pid/pid.h"
#include "drivers/line_sensor/line_sensor.h"
#include "drivers/barcode/barcode.h"
#include "drivers/state_machine/state_machine.h"

// ===== Configuration =====
#define BASE_SPEED 0.5f         // Base speed for line following
#define TURN_SPEED 0.20f        // Speed during 90° turns
#define TURN_ANGLE_DEG 90.0f    // Target angle for turns
#define TURN_TOLERANCE_DEG 5.0f // Acceptable angle error

#define EMERGENCY_STOP_PIN 20 // GP20 for emergency stop button

#define LINE_FOLLOW_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define BARCODE_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define TELEMETRY_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define TURN_TASK_PRIORITY (tskIDLE_PRIORITY + 3)

// ===== Global state =====
static SemaphoreHandle_t state_mutex;
static TaskHandle_t turn_task_handle = NULL;
static volatile bool emergency_stop_triggered = false;
static volatile bool barcode_scanning_active = false;

// ===== Barcode state tracking =====
static char last_barcode_decoded[BARCODE_MAX_LEN] = "NONE";
static uint32_t total_barcodes_detected = 0;

// ===== GPIO Interrupt Router =====
// Routes GPIO interrupts to the appropriate handler
void gpio_interrupt_router(uint gpio, uint32_t events)
{
    if (gpio == EMERGENCY_STOP_PIN) {
        // Handle emergency stop
        motor_stop();
        emergency_stop_triggered = true;
        
        if (state_mutex != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(state_mutex, &xHigherPriorityTaskWoken);
        }
        printf("\n!!! EMERGENCY STOP TRIGGERED (GP20) !!!\n");
    }
    else if (gpio == 3 || gpio == 26) {
        // Route to encoder handler (GP3 = left, GP26 = right)
        encoder_irq_handler(gpio, events);
    }
}

// ===== Barcode callback =====
void on_barcode_detected(const char *decoded_str, barcode_command_t cmd)
{
    total_barcodes_detected++;
    strncpy(last_barcode_decoded, decoded_str, BARCODE_MAX_LEN - 1);
    last_barcode_decoded[BARCODE_MAX_LEN - 1] = '\0';

    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║    BARCODE DETECTED #%-4lu            ║\n", total_barcodes_detected);
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Raw Code:     %-24s║\n", decoded_str);

    robot_event_t event = EVENT_BARCODE_FORWARD;
    const char *action = "UNKNOWN";
    bool valid_command = false;

    switch (cmd)
    {
    case CMD_LEFT:
        action = "LEFT TURN";
        event = EVENT_BARCODE_LEFT;
        valid_command = true;
        break;
    case CMD_RIGHT:
        action = "RIGHT TURN";
        event = EVENT_BARCODE_RIGHT;
        valid_command = true;
        break;
    case CMD_STOP:
        action = "STOP";
        event = EVENT_BARCODE_STOP;
        valid_command = true;
        break;
    case CMD_FORWARD:
        action = "FORWARD";
        event = EVENT_BARCODE_FORWARD;
        valid_command = true;
        break;
    default:
        action = "INVALID/IGNORED";
        break;
    }

    printf("║ Command:      %-24s║\n", action);
    
    // Get robot status
    float distance = encoder_get_distance_m();
    float rpm_l = encoder_get_rpm_left();
    float rpm_r = encoder_get_rpm_right();
    float yaw_raw, yaw;
    imu_get_heading_deg(&yaw_raw, &yaw);
    
    printf("║ Distance:     %.2fm%-20s║\n", distance, "");
    printf("║ Speed:        %.0f / %.0f RPM%-13s║\n", rpm_l, rpm_r, "");
    printf("║ Heading:      %.1f°%-20s║\n", yaw, "");
    printf("║ Action:       %-24s║\n", valid_command ? "EXECUTING" : "IGNORING");
    printf("╚════════════════════════════════════════╝\n\n");

    // Only trigger state machine for valid commands
    if (valid_command && xSemaphoreTake(state_mutex, portMAX_DELAY))
    {
        state_machine_process_event(event);
        xSemaphoreGive(state_mutex);
    }
}

// ===== Barcode scanning task =====
void barcode_scan_task(void *params)
{
    printf("[BARCODE_TASK] Task started\n");
    
    while (1)
    {
        // Call barcode_update to handle timeouts
        // The actual barcode scanning happens in timer interrupt
        barcode_update();
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz polling
    }
}

// ===== Turn execution task =====
void turn_task(void *params)
{
    bool turn_left = *(bool *)params;
    vPortFree(params);

    printf("[TURN_TASK] Starting %s turn\n", turn_left ? "LEFT" : "RIGHT");

    // Stop line following temporarily
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(200));

    // Get initial heading
    float initial_heading_raw, initial_heading;
    imu_get_heading_deg(&initial_heading_raw, &initial_heading);
    float target_heading = initial_heading + (turn_left ? -TURN_ANGLE_DEG : TURN_ANGLE_DEG);

    // Normalize target heading to [-180, 180]
    while (target_heading > 180.0f)
        target_heading -= 360.0f;
    while (target_heading < -180.0f)
        target_heading += 360.0f;

    printf("[TURN_TASK] Initial: %.1f° -> Target: %.1f°\n", initial_heading, target_heading);

    // Execute turn with IMU feedback
    uint32_t turn_start = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = 5000; // 5 second timeout

    while (1)
    {
        float current_heading_raw, current_heading;
        imu_get_heading_deg(&current_heading_raw, &current_heading);
        float error = target_heading - current_heading;

        // Normalize error
        while (error > 180.0f)
            error -= 360.0f;
        while (error < -180.0f)
            error += 360.0f;

        printf("[TURN_TASK] Current: %.1f°, Error: %.1f°\n", current_heading, error);

        // Check if turn complete
        if (fabs(error) < TURN_TOLERANCE_DEG)
        {
            printf("[TURN_TASK] Turn complete!\n");
            motor_stop();
            break;
        }

        // Check timeout
        if (to_ms_since_boot(get_absolute_time()) - turn_start > timeout_ms)
        {
            printf("[TURN_TASK] Turn timeout!\n");
            motor_stop();
            if (xSemaphoreTake(state_mutex, portMAX_DELAY))
            {
                state_machine_process_event(EVENT_ERROR);
                xSemaphoreGive(state_mutex);
            }
            turn_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }

        // Apply turning motion
        if (turn_left)
        {
            motor_set_speed(-TURN_SPEED, TURN_SPEED); // Spin left
        }
        else
        {
            motor_set_speed(TURN_SPEED, -TURN_SPEED); // Spin right
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Turn complete - trigger state machine
    vTaskDelay(pdMS_TO_TICKS(200));

    if (xSemaphoreTake(state_mutex, portMAX_DELAY))
    {
        state_machine_process_event(EVENT_TURN_COMPLETE);
        xSemaphoreGive(state_mutex);
    }

    turn_task_handle = NULL;
    vTaskDelete(NULL);
}

// ===== Line following task =====
void line_follow_task(void *params)
{
    printf("[LINE_FOLLOW] Task started\n");

    // Single-sensor line following variables (from original)
    typedef enum
    {
        SEARCH_NONE,
        SEARCH_LEFT,
        SEARCH_RIGHT
    } search_direction_t;

    search_direction_t last_turn = SEARCH_NONE;
    float search_intensity = 0.4f; // How hard to turn while searching
    uint32_t recovery_count = 0;   // Counter for recovery stabilization
    const uint32_t RECOVERY_CYCLES = 15; // Stabilize for 15 cycles (150ms) after finding line
    bool was_off_track = false;    // Track if we were just off the line

    // Barcode mode variables
    volatile bool barcode_scanning_active = false;
    float barcode_reference_heading = 0.0f;
    uint32_t time_on_line_start = 0;
    uint32_t barcode_mode_start_time = 0;
    const uint32_t BARCODE_ENABLE_TIME_MS = 500;
    const uint32_t BARCODE_TIMEOUT_MS = 2000;

    while (1)
    {
        // Check emergency stop
        if (emergency_stop_triggered)
        {
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        robot_state_t state;

        if (xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            state = state_machine_get_state();
            xSemaphoreGive(state_mutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Only follow line in LINE_FOLLOWING state
        if (state != STATE_LINE_FOLLOWING)
        {
            was_off_track = false;
            last_turn = SEARCH_NONE;
            search_intensity = 0.4f;
            recovery_count = 0;
            barcode_scanning_active = false;
            time_on_line_start = 0;
            pid_init();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Read sensors
        line_state_t line_state = line_sensor_read();
        float current_heading_raw, current_heading;
        imu_get_heading_deg(&current_heading_raw, &current_heading);
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Check if on line
        if (line_state == LINE_BLACK)
        {
            // ===== ON LINE =====
            
            // Track time on line for barcode mode
            if (time_on_line_start == 0)
            {
                time_on_line_start = now;
            }
            
            uint32_t time_on_line = now - time_on_line_start;
            
            // Activate barcode scan mode after stable tracking
            if (!barcode_scanning_active && time_on_line > BARCODE_ENABLE_TIME_MS)
            {
                barcode_scanning_active = true;
                barcode_mode_start_time = now;
                barcode_reference_heading = current_heading;
                pid_init(); // Reset PID for barcode mode
                printf("[LINE_FOLLOW] Barcode scan mode ON (heading: %.1f°)\n", barcode_reference_heading);
            }
            
            // Check barcode timeout
            if (barcode_scanning_active)
            {
                uint32_t time_in_barcode = now - barcode_mode_start_time;
                if (time_in_barcode > BARCODE_TIMEOUT_MS)
                {
                    barcode_scanning_active = false;
                    time_on_line_start = now;
                    printf("[LINE_FOLLOW] Barcode mode OFF (timeout)\n");
                }
            }
            
            float left_cmd, right_cmd;
            
            if (barcode_scanning_active)
            {
                // **BARCODE MODE: Go perfectly straight using IMU + PID**
                float heading_error = barcode_reference_heading - current_heading;
                
                // Normalize error to [-180, 180]
                while (heading_error > 180.0f) heading_error -= 360.0f;
                while (heading_error < -180.0f) heading_error += 360.0f;
                
                // Apply deadband
                if (fabs(heading_error) < 1.5f)
                {
                    heading_error = 0.0f;
                }
                
                // Use PID to compute correction
                float heading_correction = pid_compute_heading(heading_error);
                
                // Limit correction
                const float MAX_HEADING_CORRECTION = 0.20f;
                if (heading_correction > MAX_HEADING_CORRECTION) heading_correction = MAX_HEADING_CORRECTION;
                if (heading_correction < -MAX_HEADING_CORRECTION) heading_correction = -MAX_HEADING_CORRECTION;
                
                // Apply correction to maintain straight heading
                left_cmd = BASE_SPEED - heading_correction;
                right_cmd = BASE_SPEED + heading_correction;
            }
            else if (was_off_track && recovery_count < RECOVERY_CYCLES)
            {
                // **RECOVERY MODE: Countersteer briefly to prevent overshoot (original behavior)**
                recovery_count++;
                
                // Apply stronger counter-turn opposite to search direction
                if (last_turn == SEARCH_RIGHT)
                {
                    // Was turning right, so countersteer left strongly
                    left_cmd = BASE_SPEED - 0.35f;  // Slow left wheel significantly
                    right_cmd = BASE_SPEED;         // Normal right wheel
                }
                else
                {
                    // Default: go straight slowly
                    left_cmd = BASE_SPEED * 0.7f;
                    right_cmd = BASE_SPEED * 0.7f;
                }
            }
            else
            {
                // **NORMAL MODE: Go straight, reset all counters (original behavior)**
                was_off_track = false;
                recovery_count = 0;
                search_intensity = 0.4f;
                last_turn = SEARCH_NONE;

                left_cmd = BASE_SPEED;
                right_cmd = BASE_SPEED;
            }

            motor_set_speed(left_cmd, right_cmd);

            // Update state machine context
            float distance = encoder_get_distance_m();
            if (xSemaphoreTake(state_mutex, portMAX_DELAY))
            {
                state_machine_update_context(left_cmd, right_cmd, distance, 0.0f, true);
                xSemaphoreGive(state_mutex);
            }
        }
        else
        {
            // ===== OFF LINE: Search for it (original behavior) =====
            was_off_track = true;
            recovery_count = 0; // Reset recovery counter when off track
            time_on_line_start = 0;
            
            if (barcode_scanning_active)
            {
                barcode_scanning_active = false;
                printf("[LINE_FOLLOW] Barcode mode OFF (line lost)\n");
            }

            // SEARCH PATTERN: Always turn RIGHT (robot veers left, so search right)
            search_intensity = 0.4f; // Reduced intensity for gentler turn
            last_turn = SEARCH_RIGHT; // Track that we're searching right

            // Always search right (veer right) - left wheel faster, right wheel slower
            float left_speed = BASE_SPEED;
            float right_speed = BASE_SPEED - search_intensity;  // 0.5 - 0.4 = 0.1

            motor_set_speed(left_speed, right_speed);

            float distance = encoder_get_distance_m();
            if (xSemaphoreTake(state_mutex, portMAX_DELAY))
            {
                state_machine_update_context(left_speed, right_speed, distance, 0.0f, false);
                xSemaphoreGive(state_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz update - faster for narrow 1.5cm line
    }
}

// ===== State monitor task =====
void state_monitor_task(void *params)
{
    printf("[STATE_MONITOR] Task started\n");

    robot_state_t prev_state = STATE_IDLE;

    while (1)
    {
        robot_state_t current_state;

        if (xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            current_state = state_machine_get_state();
            xSemaphoreGive(state_mutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Handle state changes
        if (current_state != prev_state)
        {
            printf("[STATE_MONITOR] State changed: %s -> %s\n",
                   state_machine_state_name(prev_state),
                   state_machine_state_name(current_state));

            switch (current_state)
            {
            case STATE_TURNING_LEFT:
                if (turn_task_handle == NULL)
                {
                    bool *turn_dir = pvPortMalloc(sizeof(bool));
                    *turn_dir = true; // left
                    xTaskCreate(turn_task, "Turn", 2048, turn_dir,
                                TURN_TASK_PRIORITY, &turn_task_handle);
                }
                break;

            case STATE_TURNING_RIGHT:
                if (turn_task_handle == NULL)
                {
                    bool *turn_dir = pvPortMalloc(sizeof(bool));
                    *turn_dir = false; // right
                    xTaskCreate(turn_task, "Turn", 2048, turn_dir,
                                TURN_TASK_PRIORITY, &turn_task_handle);
                }
                break;

            case STATE_STOPPED:
                motor_stop();
                printf("\n*** ROBOT STOPPED ***\n");
                break;

            case STATE_ERROR:
                motor_stop();
                printf("\n!!! ERROR STATE !!!\n");
                break;

            default:
                break;
            }

            prev_state = current_state;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===== Telemetry task =====
void telemetry_task(void *params)
{
    printf("[TELEMETRY] Task started\n");

    uint32_t report_count = 0;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz

        const robot_context_t *ctx;
        robot_state_t state;

        if (xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            ctx = state_machine_get_context();
            state = ctx->current_state;

            printf("\n----- Telemetry Report #%lu -----\n", ++report_count);
            printf("State:         %s\n", state_machine_state_name(state));
            printf("Line Error:    %.3f\n", ctx->line_error);
            printf("Line Status:   %s\n", ctx->line_on_track ? "ON TRACK" : "OFF TRACK");
            printf("Speed L/R:     %.2f / %.2f\n", ctx->current_speed_left, ctx->current_speed_right);
            printf("RPM L/R:       %.0f / %.0f\n", encoder_get_rpm_left(), encoder_get_rpm_right());
            printf("Distance:      %.2f m\n", ctx->distance_traveled_m);
            printf("Last Barcode:  %s\n",
                   ctx->last_barcode_cmd == CMD_LEFT ? "LEFT" : 
                   ctx->last_barcode_cmd == CMD_RIGHT ? "RIGHT" :
                   ctx->last_barcode_cmd == CMD_STOP ? "STOP" :
                   ctx->last_barcode_cmd == CMD_FORWARD ? "FORWARD" : "NONE");

            uint16_t line_raw = line_sensor_read_raw();
            float yaw_raw, yaw;
            imu_get_heading_deg(&yaw_raw, &yaw);
            const char* last_barcode = barcode_get_last_decoded();
            printf("Raw Line ADC:  %u\n", line_raw);
            printf("IMU Yaw:       %.1f°\n", yaw);
            printf("Last Barcode:  \"%s\" (Total: %lu)\n", 
                   last_barcode ? last_barcode : "none", total_barcodes_detected);
            printf("Barcode Mode:  %s\n", barcode_scanning_active ? "ACTIVE" : "OFF");
            printf("-----------------------------\n\n");

            xSemaphoreGive(state_mutex);
        }
    }
}

// ===== Main =====
int main()
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n\n");
    printf("========================================\n");
    printf("  Demo 2: Line Following + Barcode\n");
    printf("========================================\n\n");

    printf("[INIT] Initializing hardware...\n");
    motor_init();
    encoder_init();
    imu_init();
    pid_init();
    line_sensor_init();
    barcode_init();
    state_machine_init();

    printf("[INIT] Setting up GPIO interrupts...\n");
    
    gpio_init(EMERGENCY_STOP_PIN);
    gpio_set_dir(EMERGENCY_STOP_PIN, GPIO_IN);
    gpio_pull_up(EMERGENCY_STOP_PIN);
    
    gpio_set_irq_enabled_with_callback(EMERGENCY_STOP_PIN, GPIO_IRQ_EDGE_FALL, 
                                       true, &gpio_interrupt_router);
    gpio_set_irq_enabled(3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(26, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    printf("[INIT] Interrupts enabled: GP3 (left), GP26 (right), GP20 (e-stop)\n");

    barcode_set_callback(on_barcode_detected);
    state_mutex = xSemaphoreCreateMutex();

    printf("[INIT] Creating FreeRTOS tasks...\n");

    xTaskCreate(line_follow_task, "LineFollow", 2048, NULL,
                LINE_FOLLOW_TASK_PRIORITY, NULL);
    xTaskCreate(state_monitor_task, "StateMonitor", 2048, NULL,
                BARCODE_TASK_PRIORITY, NULL);
    xTaskCreate(telemetry_task, "Telemetry", 2048, NULL,
                TELEMETRY_TASK_PRIORITY, NULL);

    printf("[INIT] Starting barcode scanner...\n");
    barcode_start_scanning();

    printf("[INIT] Starting robot...\n");
    state_machine_process_event(EVENT_START);

    printf("\n*** System Ready - Starting FreeRTOS Scheduler ***\n\n");

    vTaskStartScheduler();

    // Should never reach here
    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}
