// ===============================================
//  Project: Robotic Car - Demo 1
//  Description: Integrates PID motor control, IMU filtering,
//               encoder live speed interrupts, MQTT telemetry,
//               debug LEDs, and optional direction-change button
//               under FreeRTOS.
// ===============================================

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "wifi_mqtt.h"
#include "debug_led.h"

#if HAVE_CHG_DIRECTION
#include "chg_direction.h"
#endif

// -----------------------------------------------
// Task Configuration
// -----------------------------------------------
#define WIFI_TASK_PRIORITY       (tskIDLE_PRIORITY + 3)
#define PID_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)
#define TELEMETRY_TASK_PRIORITY  (tskIDLE_PRIORITY + 1)

#define PID_TASK_PERIOD_MS       20      // 50 Hz
#define TELEMETRY_PERIOD_MS      1000    // 1 Hz

// -----------------------------------------------
// Global Variables
// -----------------------------------------------
static float target_speed   = 80.0f;  // Reduced from 150 - adjust as needed (0-255)
static float target_heading = 0.0f;

// -----------------------------------------------
// PID Control Task
// -----------------------------------------------
static void pid_task(void *p)
{
    TickType_t last = xTaskGetTickCount();
    static int state = 0;  // 0 = stopped, 1 = forward, 2 = backward
    static int loop_count = 0;
    static bool transitioning = false;
    static int transition_counter = 0;
    
    printf("[PID] Task loop starting\n");
    printf("[PID] Initial state: STOPPED\n");

    for (;;)
    {
#if HAVE_CHG_DIRECTION
        if (chg_direction_was_pressed() && !transitioning)
        {
            // Cycle through states: STOPPED -> FORWARD -> BACKWARD -> STOPPED
            int old_state = state;
            state = (state + 1) % 3;
            
            transitioning = true;
            transition_counter = 0;
            
            const char* state_names[] = {"STOPPED", "FORWARD", "BACKWARD"};
            printf("State change: %s -> %s\n", state_names[old_state], state_names[state]);
        }
        
        // Handle smooth state transition
        if (transitioning)
        {
            if (transition_counter < 25)  // 0.5 seconds - ramp down to stop
            {
                // Always slow down first regardless of state change
                float ramp_speed = target_speed * (1.0f - (transition_counter / 25.0f));
                
                // Use old state direction for slowing down
                int old_state = (state == 0) ? 2 : (state - 1);
                int direction = (old_state == 1) ? 1 : (old_state == 2) ? -1 : 0;
                
                motor_set_speed(direction * ramp_speed, direction * ramp_speed);
                transition_counter++;
            }
            else if (transition_counter < 40)  // 0.3 seconds - hold at stop
            {
                motor_set_speed(0, 0);
                transition_counter++;
            }
            else if (transition_counter < 65)  // 0.5 seconds - ramp up (if not staying stopped)
            {
                if (state == 0)  // If new state is STOPPED, stay at 0
                {
                    motor_set_speed(0, 0);
                }
                else  // Ramp up for FORWARD or BACKWARD
                {
                    float ramp_speed = target_speed * ((transition_counter - 40) / 25.0f);
                    int direction = (state == 1) ? 1 : -1;
                    motor_set_speed(direction * ramp_speed, direction * ramp_speed);
                }
                transition_counter++;
            }
            else
            {
                // Transition complete
                transitioning = false;
                const char* state_names[] = {"STOPPED", "FORWARD", "BACKWARD"};
                printf("State transition complete: Now %s\n", state_names[state]);
            }
            
            vTaskDelayUntil(&last, pdMS_TO_TICKS(PID_TASK_PERIOD_MS));
            continue;  // Skip normal PID during transition
        }
#endif

        // If in STOPPED state, just keep motors off
        if (state == 0)
        {
            motor_set_speed(0, 0);
            
            // Still update LEDs and read sensors
            float rpm_l = encoder_get_rpm_left();
            float rpm_r = encoder_get_rpm_right();
            if (rpm_l > 0.1f || rpm_r > 0.1f)
                debug_led_set(24, true);
            else
                debug_led_set(24, false);
                
            float heading_raw, heading_filt;
            if (imu_get_heading_deg(&heading_raw, &heading_filt))
                debug_led_set(19, true);
            else
                debug_led_set(19, false);
            
            if (++loop_count >= 50) {
                printf("[PID] STOPPED (rpm: L=%.1f R=%.1f)\n", rpm_l, rpm_r);
                loop_count = 0;
            }
            
            vTaskDelayUntil(&last, pdMS_TO_TICKS(PID_TASK_PERIOD_MS));
            continue;
        }

        // ---- Encoder data ----
        float rpm_l = encoder_get_rpm_left();
        float rpm_r = encoder_get_rpm_right();
        if (rpm_l > 0.1f || rpm_r > 0.1f)
            debug_led_set(24, true);  // Encoder LED (fixed pin)
        else
            debug_led_set(24, false);

        // ---- IMU data ----
        float heading_raw, heading_filt;
        if (imu_get_heading_deg(&heading_raw, &heading_filt))
            debug_led_set(19, true);  // IMU LED (fixed pin)
        else
            debug_led_set(19, false);

        // ---- PID control ----
        // Determine direction from state: 1=forward, 2=backward
        int direction = (state == 1) ? 1 : -1;
        
        float avg_rpm = (rpm_l + rpm_r) / 2.0f;
        float heading_error = target_heading - heading_filt;
        if (heading_error > 180.0f) heading_error -= 360.0f;
        if (heading_error < -180.0f) heading_error += 360.0f;

        float speed_corr   = pid_compute_speed(target_speed, avg_rpm);
        float heading_corr = pid_compute_heading(heading_error);

        float left_output  = direction * (target_speed + speed_corr - heading_corr);
        float right_output = direction * (target_speed + speed_corr + heading_corr);

        // Debug output every 50 loops (1 second)
        if (++loop_count >= 50) {
            const char* state_names[] = {"STOPPED", "FORWARD", "BACKWARD"};
            printf("[PID] %s: L=%.1f R=%.1f (tgt=%.1f rpm=%.1f)\n",
                   state_names[state], left_output, right_output, target_speed, avg_rpm);
            loop_count = 0;
        }

        motor_set_speed(left_output, right_output);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PID_TASK_PERIOD_MS));
    }
}

// -----------------------------------------------
// Telemetry Task
// -----------------------------------------------
static void telemetry_task(void *p)
{
    TickType_t last = xTaskGetTickCount();
    
    printf("[TELEM] Task loop starting\n");

    for (;;)
    {
        printf("[TELEM] Loop iteration\n");
        
        float rpm_l = encoder_get_rpm_left();
        float rpm_r = encoder_get_rpm_right();
        float dist  = encoder_get_distance_m();
        
        // Get tick counts for diagnostics
        uint32_t ticks_l = 0, ticks_r = 0;
        encoder_get_ticks(&ticks_l, &ticks_r);
        
        float heading_raw, heading_filt;
        imu_get_heading_deg(&heading_raw, &heading_filt);

        char msg[256];
        snprintf(msg, sizeof(msg),
                 "{\"rpm_l\":%.2f,\"rpm_r\":%.2f,\"dist\":%.3f,"
                 "\"heading_raw\":%.2f,\"heading_filt\":%.2f,"
                 "\"ticks_l\":%lu,\"ticks_r\":%lu}",
                 rpm_l, rpm_r, dist, heading_raw, heading_filt, ticks_l, ticks_r);

        printf("Telemetry: %s\n", msg);
        
        // ENCODER DIAGNOSTIC - if ticks are 0, encoders aren't working!
        if (ticks_l == 0 && ticks_r == 0) {
            printf("[WARN] No encoder ticks detected! Check connections:\n");
            printf("       Left encoder: GP6 (Grove Port 5)\n");
            printf("       Right encoder: GP27 (Grove Port 6)\n");
        }
        
        // Only publish if WiFi/MQTT available
        // wifi_mqtt_publish("pico/telemetry", msg);  // Disabled for testing

        // Blink status LED (heartbeat)
        debug_led_blink(25, 20);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

// -----------------------------------------------
// Wi-Fi and MQTT Task
// -----------------------------------------------
static void wifi_task(void *p)
{
    wifi_connect_init();

    while (!wifi_is_connected())
    {
        printf("Connecting to Wi-Fi...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    printf("Wi-Fi connected.\n");
    debug_led_set(13, true); // Wi-Fi LED solid ON

    mqtt_init();
    while (!mqtt_is_connected())
    {
        printf("Connecting to MQTT broker...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    printf("MQTT connected.\n");

    for (;;)
    {
        mqtt_loop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// -----------------------------------------------
// Main Entry
// -----------------------------------------------
int main(void)
{
    stdio_init_all();
    sleep_ms(500);
    printf("=== Demo 1: PID + IMU + MQTT + LEDs ===\n");

    debug_led_init();
    motor_init();
    encoder_init();
    imu_init();
    pid_init();

#if HAVE_CHG_DIRECTION
    chg_direction_init();
    printf("Change-direction driver active.\n");
#else
    printf("Change-direction driver not found â€” running forward only.\n");
#endif

    float heading_raw, heading_filt;
    imu_get_heading_deg(&heading_raw, &heading_filt);
    target_heading = heading_filt;

    printf("\n[INFO] Creating FreeRTOS tasks...\n");
    printf("      WiFi task DISABLED for testing\n");
    // xTaskCreate(wifi_task, "WiFi", 2048, NULL, WIFI_TASK_PRIORITY, NULL);
    printf("      Creating PID task...\n");
    xTaskCreate(pid_task, "PID", 2048, NULL, PID_TASK_PRIORITY, NULL);
    printf("      Creating Telemetry task...\n");
    xTaskCreate(telemetry_task, "Telemetry", 2048, NULL, TELEMETRY_TASK_PRIORITY, NULL);

    printf("\n[INFO] Starting FreeRTOS scheduler...\n");
    printf("========================================\n\n");
    
    vTaskStartScheduler();
    
    // Should never reach here
    printf("ERROR: Scheduler failed to start!\n");
    while (1) {
        sleep_ms(1000);
    }
}