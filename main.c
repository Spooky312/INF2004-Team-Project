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
#include "debug_led.h"

// Updated include: use new thread-safe network manager
#include "networkManager.h"
#include "mqtt.h"


#if HAVE_CHG_DIRECTION
#include "chg_direction.h"
#endif

// -----------------------------------------------
// Task Configuration
// -----------------------------------------------
#define NET_TASK_PRIORITY        (tskIDLE_PRIORITY + 3)
#define PID_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)
#define TELEMETRY_TASK_PRIORITY  (tskIDLE_PRIORITY + 1)

#define PID_TASK_PERIOD_MS       20      // 50 Hz
#define TELEMETRY_PERIOD_MS      1000    // 1 Hz

// -----------------------------------------------
// Global Variables
// -----------------------------------------------
static float target_speed   = 50.0f;  // Reduced from 80 for more stable control (0-255)
static float target_heading = 0.0f;

// -----------------------------------------------
// Helper: Convert heading to compass direction
// -----------------------------------------------
static const char* heading_to_compass(float heading)
{
    // Normalize heading to 0-360
    while (heading < 0) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    
    if (heading >= 337.5f || heading < 22.5f) return "N";
    if (heading >= 22.5f && heading < 67.5f) return "NE";
    if (heading >= 67.5f && heading < 112.5f) return "E";
    if (heading >= 112.5f && heading < 157.5f) return "SE";
    if (heading >= 157.5f && heading < 202.5f) return "S";
    if (heading >= 202.5f && heading < 247.5f) return "SW";
    if (heading >= 247.5f && heading < 292.5f) return "W";
    if (heading >= 292.5f && heading < 337.5f) return "NW";
    return "?";
}


static void gpio_irq_router(uint gpio, uint32_t events) {
    encoder_irq_handler(gpio, events);

#if HAVE_CHG_DIRECTION
    chg_direction_irq_handler(gpio, events);
#endif
}

void gpio_router_init(void) {
    // Register shared callback ONCE
    gpio_set_irq_enabled_with_callback(ENCODER_LEFT_PIN,
        GPIO_IRQ_EDGE_FALL, true, &gpio_irq_router);

    // Enable all relevant pins (already done in their inits)
    gpio_set_irq_enabled(ENCODER_RIGHT_PIN, GPIO_IRQ_EDGE_FALL, true);

#if HAVE_CHG_DIRECTION
    gpio_set_irq_enabled(CHG_DIRECTION_PIN, GPIO_IRQ_EDGE_FALL, true);
#endif
}


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

        float heading_raw, heading_filt;
        if (imu_get_heading_deg(&heading_raw, &heading_filt))
            debug_led_set(19, true);  // IMU LED (fixed pin)
        else
            debug_led_set(19, false);

        // ---- PID control with individual wheel feedback ----
        // Determine direction from state: 1=forward, 2=backward
        int direction = (state == 1) ? 1 : -1;
        
        // Calculate heading error and correction
        float heading_error = target_heading - heading_filt;
        if (heading_error > 180.0f) heading_error -= 360.0f;
        if (heading_error < -180.0f) heading_error += 360.0f;
        float heading_corr = pid_compute_heading(heading_error);

        // Desired speeds with heading correction applied
        // Positive heading_corr = need to turn left (increase left, decrease right)
        float desired_speed_left  = target_speed + heading_corr;
        float desired_speed_right = target_speed - heading_corr;

        // Individual wheel PID control based on actual encoder feedback
        float speed_corr_left  = pid_compute_speed_left(desired_speed_left, rpm_l);
        float speed_corr_right = pid_compute_speed_right(desired_speed_right, rpm_r);

        // Final motor outputs
        float left_output  = direction * (desired_speed_left + speed_corr_left);
        float right_output = direction * (desired_speed_right + speed_corr_right);

        // Debug output every 50 loops (1 second)
        if (++loop_count >= 50) {
            const char* state_names[] = {"STOPPED", "FORWARD", "BACKWARD"};
            float avg_rpm = (rpm_l + rpm_r) / 2.0f;
            printf("[PID] %s: L=%.1f(%.1frpm) R=%.1f(%.1frpm) Tgt=%.1f Avg=%.1f\n",
                   state_names[state], left_output, rpm_l, right_output, rpm_r, 
                   target_speed, avg_rpm);
            printf("[PID] Head: Target=%.2f°(%s) Current=%.2f° Err=%.2f° Corr=%.2f\n",
                   target_heading, heading_to_compass(target_heading), 
                   heading_filt, heading_error, heading_corr);
            printf("[PID] Desired: L=%.1f R=%.1f | Corrections: L=%.2f R=%.2f\n",
                   desired_speed_left, desired_speed_right, speed_corr_left, speed_corr_right);
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
    
    static int mqtt_fail_count = 0;

    for (;;)
    {
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
                 "\"target_heading\":%.2f,\"heading_raw\":%.2f,\"heading_filt\":%.2f,"
                 "\"ticks_l\":%lu,\"ticks_r\":%lu}",
                 rpm_l, rpm_r, dist, target_heading, heading_raw, heading_filt, ticks_l, ticks_r);

        printf("[TELEM] %s\n", msg);
        
        // ENCODER DIAGNOSTIC - if ticks are 0, encoders aren't working!
        if (ticks_l == 0 && ticks_r == 0) {
            printf("[TELEM] [WARN] No encoder ticks detected!\n");
        }
        
        // Publish via MQTT if connected
        if (mqtt_app_is_connected()) {
            mqtt_app_publish("pico/telemetry", msg, 0, 0);
            printf("[TELEM] ✅ Published to MQTT topic: pico/telemetry\n");
            mqtt_fail_count = 0;
        } else {
            mqtt_fail_count++;
            if (mqtt_fail_count <= 3 || mqtt_fail_count % 10 == 0) {
                printf("[TELEM] ❌ MQTT not connected - message not published (fail #%d)\n", mqtt_fail_count);
            }
        }


        // Blink status LED (heartbeat)
        debug_led_blink(25, 20);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

// -----------------------------------------------
// Main Entry
// -----------------------------------------------
int main(void)
{
    // Basic initialization
    stdio_init_all();
    
    // Wait for USB serial to be ready
    sleep_ms(3000);
    
    // Infinite loop with output to verify serial works
    printf("\n\n\n=== SERIAL TEST ===\n");
    printf("If you see this, serial is working!\n");
    
    int counter = 0;
    while (counter < 5) {
        printf("Test message %d\n", counter);
        sleep_ms(1000);
        counter++;
    }
    
    printf("\n[INIT] Starting hardware initialization...\n");
    
    printf("[INIT] Debug LED...\n");
    debug_led_init();
    
    printf("[INIT] Motor driver...\n");
    motor_init();
    
    printf("[INIT] Encoder driver...\n");
    encoder_init();
    
    printf("[INIT] IMU sensor...\n");
    imu_init();
    
    printf("[INIT] PID controller...\n");
    pid_init();

    printf("[INIT] GPIO interrupt router...\n");
    gpio_router_init(); 

#if HAVE_CHG_DIRECTION
    printf("[INIT] Change-direction button...\n");
    chg_direction_init();
    printf("[INFO] Change-direction driver active.\n");
#else
    printf("[INFO] Change-direction driver not available - forward only mode.\n");
#endif

    printf("\n[IMU] Stabilizing magnetometer...\n");
    sleep_ms(100);  // Give sensor time to settle
    
    float heading_raw, heading_filt;
    printf("[IMU] Reading initial heading samples...\n");
    for (int i = 0; i < 10; i++) {
        if (imu_get_heading_deg(&heading_raw, &heading_filt)) {
            printf("  Sample %d: %.2f° (raw) %.2f° (filtered)\n", i+1, heading_raw, heading_filt);
        } else {
            printf("  Sample %d: IMU read failed!\n", i+1);
        }
        sleep_ms(50);
    }
    
    target_heading = heading_filt;
    printf("[HEADING] Target heading set to: %.2f° (%s)\n", 
           target_heading, heading_to_compass(target_heading));

    printf("\n[INFO] Creating FreeRTOS tasks...\n");
    printf("      Creating Network Manager task...\n");
    xTaskCreate(network_manager_task, "NetMgr", 2048, NULL, NET_TASK_PRIORITY, NULL);

    printf("      Creating PID task...\n");
    xTaskCreate(pid_task, "PID", 2048, NULL, PID_TASK_PRIORITY, NULL);

    printf("      Creating Telemetry task...\n");
    xTaskCreate(telemetry_task, "Telemetry", 2048, NULL, TELEMETRY_TASK_PRIORITY, NULL);

    printf("\n[INFO] Starting FreeRTOS scheduler...\n");
    printf("========================================\n\n");
    
    vTaskStartScheduler();
    
    printf("ERROR: Scheduler failed to start!\n");
    while (1) {
        sleep_ms(1000);
    }
}
