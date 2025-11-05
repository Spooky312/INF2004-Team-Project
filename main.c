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

// Centralized configuration
#include "robot_config.h"

// Hardware drivers
#include "src/motor/motor.h"
#include "src/encoder/encoder.h"
#include "src/imu/imu.h"
#include "src/pid/pid.h"
#include "src/line_sensor/line_sensor.h"
#include "src/barcode/barcode.h"
#include "src/state_machine/state_machine.h"
#include "src/networking/mqtt/mqtt.h"
#include "src/networking/networkManager.h"

// All configuration now in robot_config.h:
// - BASE_SPEED, TURN_SPEED
// - TURN_ANGLE_DEG, TURN_TOLERANCE_DEG
// - EMERGENCY_STOP_PIN
// - Task priorities and stack sizes
// - Timing parameters

// ===== Global state =====
static SemaphoreHandle_t state_mutex;
static TaskHandle_t turn_task_handle = NULL;
static volatile bool emergency_stop_triggered = false;
static volatile bool barcode_scanning_active = false;

#define TELEMETRY_PERIOD_MS 1000
static float target_heading = 0.0f;
static float obstacle_width_cm = 0.0f;
static int obstacle_count = 0;

// ===== Barcode state tracking =====
static char last_barcode_decoded[BARCODE_MAX_LEN] = "NONE";
static uint32_t total_barcodes_detected = 0;

// ===== GPIO Interrupt Router =====
// Routes GPIO interrupts to the appropriate handler
void gpio_interrupt_router(uint gpio, uint32_t events)
{
    if (gpio == EMERGENCY_STOP_PIN)
    {
        // Handle emergency stop
        motor_stop();
        emergency_stop_triggered = true;

        if (state_mutex != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(state_mutex, &xHigherPriorityTaskWoken);
        }
        printf("\n!!! EMERGENCY STOP TRIGGERED (GP20) !!!\n");
    }
    else if (gpio == 3 || gpio == 6)
    {
        // Route to encoder handler (GP3 = left, GP6 = right)
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

    // Parse barcode letters to determine turn direction
    // RIGHT turn: A, C, E, G, I, K, M, O, Q, S, U, W, Y (odd alphabet positions 1,3,5...)
    // LEFT turn:  B, D, F, H, J, L, N, P, R, T, V, X, Z (even alphabet positions 2,4,6...)
    bool found_turn_letter = false;
    for (int i = 0; decoded_str[i] != '\0' && !found_turn_letter; i++)
    {
        char c = decoded_str[i];

        // Convert to uppercase if lowercase
        if (c >= 'a' && c <= 'z')
            c = c - 'a' + 'A';

        // Check if it's a valid letter A-Z
        if (c >= 'A' && c <= 'Z')
        {
            // Calculate alphabet position (A=1, B=2, C=3, ...)
            int position = (c - 'A') + 1;

            if (position % 2 == 1) // Odd position = RIGHT
            {
                action = "RIGHT TURN (Letter)";
                event = EVENT_BARCODE_RIGHT;
                valid_command = true;
                found_turn_letter = true;
            }
            else // Even position = LEFT
            {
                action = "LEFT TURN (Letter)";
                event = EVENT_BARCODE_LEFT;
                valid_command = true;
                found_turn_letter = true;
            }
        }
    }

    // If no turn letter found, fall back to command-based logic
    if (!found_turn_letter)
    {
        switch (cmd)
        {
        case CMD_LEFT:
            action = "LEFT TURN (CMD)";
            event = EVENT_BARCODE_LEFT;
            valid_command = true;
            break;
        case CMD_RIGHT:
            action = "RIGHT TURN (CMD)";
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
        case CMD_UTURN:
            action = "U-TURN";
            event = EVENT_BARCODE_FORWARD; // Can handle U-turn logic later
            valid_command = true;
            break;
        default:
            action = "INVALID/IGNORED";
            break;
        }
    }

    printf("║ Command:      %-24s║\n", action);

    // Get robot status
    float distance = encoder_get_distance_m();
    float rpm_l = encoder_get_rpm_left();
    float rpm_r = encoder_get_rpm_right();
    float rpm_avg = (rpm_l + rpm_r) / 2.0f;
    float yaw_raw, yaw;
    imu_get_heading_deg(&yaw_raw, &yaw);

    // Calculate current actual motor fraction (approximation based on RPM)
    // Assuming max RPM is around 150 at full speed (adjust if needed)
    float current_motor_fraction = (rpm_avg / 150.0f);
    if (current_motor_fraction > 1.0f)
        current_motor_fraction = 1.0f;

    printf("║ Distance:     %.2fm%-20s║\n", distance, "");
    printf("║ Speed:        %.0f / %.0f RPM (avg: %.0f)%-4s║\n", rpm_l, rpm_r, rpm_avg, "");
    printf("║ Base Speed:   %.3f (%.0f%%)%-14s║\n", BASE_SPEED, BASE_SPEED * 100.0f, "");
    printf("║ Current Est:  %.3f (%.0f%%)%-14s║\n", current_motor_fraction, current_motor_fraction * 100.0f, "");
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

        vTaskDelay(pdMS_TO_TICKS(BARCODE_UPDATE_RATE_MS)); // 100Hz polling
    }
}

// ===== Line following task =====
void line_follow_task(void *params)
{
    printf("[LINE_FOLLOW] Task started - EDGE FOLLOWING MODE (SMOOTH)\n");
    printf("[LINE_FOLLOW] Base Speed: %.2f | BLACK→RIGHT, WHITE→LEFT (sine-wave)\n", BASE_SPEED);

    uint32_t debug_counter = 0;

    while (1)
    {
        // Check emergency stop
        if (emergency_stop_triggered)
        {
            motor_stop();
            line_sensor_reset_state(); // Reset line following state
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

        // Only run in LINE_FOLLOWING state
        if (state != STATE_LINE_FOLLOWING)
        {
            line_sensor_reset_state(); // Reset line following state
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Get motor commands from line sensor driver
        motor_commands_t commands = line_sensor_compute_motor_commands();
        motor_set_speed(commands.left_speed, commands.right_speed);

        // Debug output every 200ms - COMMENTED OUT (only show barcode info)
        // debug_counter++;
        // if (debug_counter >= 20)  // 20 * 10ms = 200ms
        // {
        //     line_state_t line_state = line_sensor_read();
        //     const char* sensor_str = (line_state == LINE_BLACK) ? "BLACK" : "WHITE";
        //     printf("[%s] Motors: L:%.3f R:%.3f\n",
        //            sensor_str, commands.left_speed, commands.right_speed);
        //     debug_counter = 0;
        // }

        // Update state machine context
        float distance = encoder_get_distance_m();
        bool on_line = (line_sensor_read() == LINE_BLACK);
        if (xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            state_machine_update_context(commands.left_speed, commands.right_speed, distance, 0.0f, on_line);
            xSemaphoreGive(state_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz update
    }
}

// ===== Autonomous barcode speed control task =====
// Configuration constants
#define BARCODE_TARGET_NARROW_MS 25.0f // Target narrow duration in ms (tune this)
#define SPEED_CONTROL_RATE_MS 100      // Control loop rate (100ms = 10Hz)
#define SPEED_CONTROL_ENABLE 0         // DISABLED - Manual calibration mode
#define MIN_NARROW_SAMPLES 3           // Minimum samples before speed adjustment

void barcode_speed_control_task(void *params)
{
    printf("[SPEED_CTRL] Autonomous speed control task started\n");
    printf("[SPEED_CTRL] Target narrow duration: %.1f ms\n", BARCODE_TARGET_NARROW_MS);

#if !SPEED_CONTROL_ENABLE
    printf("[SPEED_CTRL] DISABLED - task will sleep\n");
    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
    return;
#endif

    // Wait for module width calibration
    while (barcode_get_module_width_m() <= 0.0f)
    {
        printf("[SPEED_CTRL] Waiting for calibration... (module_width not set)\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    printf("[SPEED_CTRL] Calibrated! Module width: %.5f m\n", barcode_get_module_width_m());
    printf("[SPEED_CTRL] Speed control active\n");

    // PID controller state (simple integral control)
    float speed_base = BASE_SPEED; // Start from configured base speed
    float integral = 0.0f;
    const float Kp = 0.002f;            // Proportional gain (tune this)
    const float Ki = 0.0005f;           // Integral gain (tune this)
    const float MAX_ADJUSTMENT = 0.15f; // Max ±15% speed adjustment

    while (1)
    {
        // Get average narrow duration from recent scans
        float avg_narrow_ms = barcode_get_avg_narrow_duration_ms();

        if (avg_narrow_ms > 0.0f)
        {
            // Compute error (positive = segments too long = going too slow)
            float error = BARCODE_TARGET_NARROW_MS - avg_narrow_ms;

            // Only adjust if we have enough samples
            if (barcode_get_narrow_sample_count() >= MIN_NARROW_SAMPLES)
            {
                // PID control
                integral += error;

                // Anti-windup
                if (integral > MAX_ADJUSTMENT / Ki)
                    integral = MAX_ADJUSTMENT / Ki;
                if (integral < -MAX_ADJUSTMENT / Ki)
                    integral = -MAX_ADJUSTMENT / Ki;

                float adjustment = (Kp * error) + (Ki * integral);

                // Clamp adjustment
                if (adjustment > MAX_ADJUSTMENT)
                    adjustment = MAX_ADJUSTMENT;
                if (adjustment < -MAX_ADJUSTMENT)
                    adjustment = -MAX_ADJUSTMENT;

                // Apply speed adjustment (symmetric for both wheels during line following)
                float target_speed = speed_base + adjustment;

                // Clamp to safe range
                if (target_speed < 0.2f)
                    target_speed = 0.2f;
                if (target_speed > 0.7f)
                    target_speed = 0.7f;

                // Update base speed smoothly (low-pass filter)
                speed_base = speed_base * 0.95f + target_speed * 0.05f;

                // Debug output
                printf("[SPEED_CTRL] narrow=%.1fms target=%.1fms err=%.1f adj=%+.3f speed=%.3f\n",
                       avg_narrow_ms, BARCODE_TARGET_NARROW_MS, error, adjustment, speed_base);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SPEED_CONTROL_RATE_MS));
    }
}

/*
// ===== One-time barcode calibration routine (DISABLED - Manual calibration mode) =====
// Call this once to measure and set module_width_m
void barcode_calibrate_module_width(float known_motor_fraction)
{
    printf("\n");
    printf("╔════════════════════════════════════════════╗\n");
    printf("║  BARCODE CALIBRATION MODE                  ║\n");
    printf("╠════════════════════════════════════════════╣\n");
    printf("║  Running at motor fraction: %.2f            ║\n", known_motor_fraction);
    printf("║  Collecting barcode samples...             ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");

    // Set motors to known speed
    motor_set_speed(known_motor_fraction, known_motor_fraction);

    // Wait for speed to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    // Clear narrow history
    barcode_reset_narrow_history();

    // Collect samples for 5 seconds
    printf("[CALIBRATE] Collecting samples for 5 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Get measured values
    float rpm_left = encoder_get_rpm_left();
    float rpm_right = encoder_get_rpm_right();
    float rpm_avg = (rpm_left + rpm_right) * 0.5f;
    float v_meas_mps = rpm_avg * WHEEL_CIRCUM_M / 60.0f;

    float avg_narrow_ms = barcode_get_avg_narrow_duration_ms();

    if (avg_narrow_ms > 0.0f && rpm_avg > 0.0f) {
        // Compute module width
        float module_width_m = v_meas_mps * (avg_narrow_ms / 1000.0f);

        // Store it
        barcode_set_module_width_m(module_width_m);

        printf("\n");
        printf("╔════════════════════════════════════════════╗\n");
        printf("║  ✅ CALIBRATION COMPLETE                   ║\n");
        printf("╠════════════════════════════════════════════╣\n");
        printf("║  Motor fraction:    %.3f                   ║\n", known_motor_fraction);
        printf("║  Measured RPM:      %.1f                   ║\n", rpm_avg);
        printf("║  Linear speed:      %.3f m/s              ║\n", v_meas_mps);
        printf("║  Avg narrow:        %.1f ms                ║\n", avg_narrow_ms);
        printf("║  Module width:      %.5f m (%.2f mm)      ║\n",
               module_width_m, module_width_m * 1000.0f);
        printf("╠════════════════════════════════════════════╣\n");
        printf("║  For target narrow = %.1f ms:              ║\n", BARCODE_TARGET_NARROW_MS);
        float rpm_target = barcode_compute_target_rpm(BARCODE_TARGET_NARROW_MS);
        printf("║  Target RPM:        %.1f                   ║\n", rpm_target);
        float fraction_est = rpm_target / rpm_avg * known_motor_fraction;
        printf("║  Estimated fraction: %.3f                  ║\n", fraction_est);
        printf("╚════════════════════════════════════════════╝\n\n");
    } else {
        printf("[CALIBRATE] ❌ FAILED - no barcode data or encoder data\n");
        printf("[CALIBRATE] avg_narrow=%.1f ms, rpm=%.1f\n", avg_narrow_ms, rpm_avg);
    }
}
*/

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
        if (to_ms_since_boot(get_absolute_time()) - turn_start > TURN_TIMEOUT_MS)
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

        // Apply turning motion - FIXED: Corrected motor directions
        if (turn_left)
        {
            motor_set_speed(TURN_SPEED, -TURN_SPEED); // Spin left: left forward, right backward
        }
        else
        {
            motor_set_speed(-TURN_SPEED, TURN_SPEED); // Spin right: left backward, right forward
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

    // Start line following task after turn completion
    xTaskCreate(line_follow_task, "LineFollow", LINE_FOLLOW_STACK_SIZE, NULL,
                LINE_FOLLOW_TASK_PRIORITY, NULL);

    vTaskDelete(NULL);
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

static void telemetry_task(void *p)
{
    TickType_t last = xTaskGetTickCount();
    printf("[TELEM] Task loop starting\n");

    static int mqtt_fail_count = 0;

    for (;;)
    {
        // --- fast local readings (no mutex) ---
        float rpm_l   = encoder_get_rpm_left();
        float rpm_r   = encoder_get_rpm_right();
        float dist_m  = encoder_get_distance_m();

        uint32_t ticks_l = 0, ticks_r = 0;
        encoder_get_ticks(&ticks_l, &ticks_r);

        float heading_raw = 0.f, heading_filt = 0.f;
        imu_get_heading_deg(&heading_raw, &heading_filt);

        // --- defaults for SM / barcode fields (filled below if we can) ---
        const char *state_str = "IDLE";
        bool        line_on_track = false;
        const char *barcode_text = "NONE";
        const char *cmd_str = "NONE";

        // ---- Get last decoded barcode text (no lock needed for your getter) ----
        const char *decoded = barcode_get_last_decoded();
        if (decoded && decoded[0] != '\0') {
            barcode_text = decoded;
        }

        // ---- Prefer command from state machine; else derive from barcode text ----
        barcode_command_t cmd_final = CMD_NONE;

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2))) {
            const robot_context_t *ctx = state_machine_get_context();
            if (ctx) {
                // state & line status
                state_str     = state_machine_state_name(ctx->current_state);
                line_on_track = ctx->line_on_track;

                // prefer the SM command if set
                if (ctx->last_barcode_cmd != CMD_NONE) {
                    cmd_final = ctx->last_barcode_cmd;
                }
            }
            xSemaphoreGive(state_mutex);
        }

        // if SM had no command, parse from the last decoded text
        if (cmd_final == CMD_NONE) {
            cmd_final = barcode_parse_command(barcode_text);
        }
        cmd_str = barcode_command_to_string(cmd_final);

        // --- publish ---
        if (mqtt_app_is_connected())
        {
            char msg[512];
            int n = snprintf(msg, sizeof msg,
                "{"
                  "\"rpm_l\":%.2f,\"rpm_r\":%.2f,"
                  "\"dist_m\":%.3f,"
                  "\"heading_raw\":%.2f,\"heading_filt\":%.2f,"
                  "\"state\":\"%s\","
                  "\"barcode_text\":\"%s\","
                  "\"barcode_cmd\":\"%s\","
                  "\"line_on_track\":%s"
                "}",
                rpm_l, rpm_r,
                dist_m,
                heading_raw, heading_filt,
                state_str,
                barcode_text,
                cmd_str,
                line_on_track ? "true" : "false");

            if (n > 0 && n < (int)sizeof msg) {
                mqtt_app_publish("pico/telemetry", msg, 0, 0);
                printf("[TELEM] Published: %s\n", msg);
                mqtt_fail_count = 0;
            } else {
                printf("[TELEM] JSON truncated (len=%d)\n", n);
            }
        }
        else
        {
            mqtt_fail_count++;
            if ((mqtt_fail_count % 10) == 1) {
                printf("[TELEM] MQTT not connected (fail #%d)\n", mqtt_fail_count);
            }
        }

        // keep the task truly periodic
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
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
    gpio_set_irq_enabled(6, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    printf("[INIT] Interrupts enabled: GP3 (left enc), GP6 (right enc), GP20 (e-stop)\n");

    barcode_set_callback(on_barcode_detected);
    state_mutex = xSemaphoreCreateMutex();

    printf("[INIT] Creating FreeRTOS tasks...\n");

    xTaskCreate(network_manager_task, "NetMgr", 2048, NULL, NET_TASK_PRIORITY, NULL);

    xTaskCreate(line_follow_task, "LineFollow", LINE_FOLLOW_STACK_SIZE, NULL,
                LINE_FOLLOW_TASK_PRIORITY, NULL);
    xTaskCreate(barcode_scan_task, "BarcodeScanner", 2048, NULL,
                LINE_FOLLOW_TASK_PRIORITY + 1, NULL); // Higher priority than line follow
    xTaskCreate(barcode_speed_control_task, "SpeedCtrl", 2048, NULL,
                LINE_FOLLOW_TASK_PRIORITY, NULL); // Same priority as line follow
    xTaskCreate(state_monitor_task, "StateMonitor", STATE_MONITOR_STACK_SIZE, NULL,
                STATE_MONITOR_PRIORITY, NULL);
    xTaskCreate(telemetry_task, "Telemetry", TELEMETRY_STACK_SIZE, NULL,
                TELEMETRY_TASK_PRIORITY, NULL);

    printf("[INIT] Starting barcode scanner...\n");
    barcode_start_scanning();

    // Optional: Run calibration at startup (comment out after first calibration)
    // Uncomment the line below, flash, run for ~10 seconds, then re-comment and reflash
    // barcode_calibrate_module_width(BASE_SPEED);  // Calibrate at base speed

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