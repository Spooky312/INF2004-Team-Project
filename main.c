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
static float target_speed   = 150.0f;
static float target_heading = 0.0f;

// -----------------------------------------------
// PID Control Task
// -----------------------------------------------
static void pid_task(void *p)
{
    TickType_t last = xTaskGetTickCount();
    static int direction = 1;  // 1 = forward, -1 = reverse

    for (;;)
    {
#if HAVE_CHG_DIRECTION
        if (chg_direction_was_pressed())
        {
            direction = -direction;
            printf("Direction toggled: %s\n",
                   direction > 0 ? "FORWARD" : "REVERSE");
        }
#endif

        // ---- Encoder data ----
        float rpm_l = encoder_get_rpm_left();
        float rpm_r = encoder_get_rpm_right();
        if (rpm_l > 0.1f || rpm_r > 0.1f)
            debug_led_success(12);  // Encoder LED
        else
            debug_led_error(12);

        // ---- IMU data ----
        float heading_raw, heading_filt;
        if (imu_get_heading_deg(&heading_raw, &heading_filt))
            debug_led_success(11);  // IMU LED
        else
            debug_led_error(11);

        // ---- PID control ----
        float avg_rpm = (rpm_l + rpm_r) / 2.0f;
        float heading_error = target_heading - heading_filt;
        if (heading_error > 180.0f) heading_error -= 360.0f;
        if (heading_error < -180.0f) heading_error += 360.0f;

        float speed_corr   = pid_compute_speed(target_speed, avg_rpm);
        float heading_corr = pid_compute_heading(heading_error);

        float left_output  = direction * (target_speed + speed_corr - heading_corr);
        float right_output = direction * (target_speed + speed_corr + heading_corr);

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

    for (;;)
    {
        float rpm_l = encoder_get_rpm_left();
        float rpm_r = encoder_get_rpm_right();
        float dist  = encoder_get_distance_m();
        float heading_raw, heading_filt;
        imu_get_heading_deg(&heading_raw, &heading_filt);

        char msg[256];
        snprintf(msg, sizeof(msg),
                 "{\"rpm_l\":%.2f,\"rpm_r\":%.2f,\"dist\":%.3f,"
                 "\"heading_raw\":%.2f,\"heading_filt\":%.2f}",
                 rpm_l, rpm_r, dist, heading_raw, heading_filt);

        printf("Telemetry: %s\n", msg);
        mqtt_publish("pico/telemetry", msg);

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
    printf("Change-direction driver not found — running forward only.\n");
#endif

    float heading_raw, heading_filt;
    imu_get_heading_deg(&heading_raw, &heading_filt);
    target_heading = heading_filt;

    xTaskCreate(wifi_task, "WiFi", 2048, NULL, WIFI_TASK_PRIORITY, NULL);
    xTaskCreate(pid_task, "PID", 2048, NULL, PID_TASK_PRIORITY, NULL);
    xTaskCreate(telemetry_task, "Telemetry", 2048, NULL, TELEMETRY_TASK_PRIORITY, NULL);

    vTaskStartScheduler();
    while (1) {}
}
