#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h" // For wifi_init

// FreeRTOS Includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // For Mutexes

// Include all Demo 1 driver headers (using corrected paths)
#include "motor.h"
#include "encoder.h"
#include "wifi_mqtt.h"
#include "imu.h"
#include "pid.h"

// --- Task Priorities ---
#define PID_TASK_PRIORITY       (configMAX_PRIORITIES - 1) // Highest priority
#define TELEMETRY_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define MQTT_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)

// --- Task Timings ---
#define PID_TASK_RUN_PERIOD_MS       20   // Run PID loop at 50Hz
#define TELEMETRY_TASK_RUN_PERIOD_MS 1000 // Publish telemetry at 1Hz

// --- Global Shared Resources ---
// Mutex to protect access to sensor data (IMU, Encoders) and PID controller
SemaphoreHandle_t g_data_mutex;

// --- Target State ---
static float g_target_heading = 0.0f;
#define TARGET_DEMO_RPM 30.0f // Target speed for the demo

// ====================================================================
//                       == TASKS ==
// ====================================================================

/**
 * @brief High-priority task for running the PID control loop.
 */
void pid_task(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PID_TASK_RUN_PERIOD_MS);

    // Set the initial target speed
    pid_set_target_rpm(TARGET_DEMO_RPM, TARGET_DEMO_RPM);

    // Get the starting heading from the IMU after a short delay for sensor settling
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow IMU to stabilize
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY) == pdTRUE) {
        g_target_heading = imu_get_filtered_heading();
        pid_set_target_heading(g_target_heading);
        xSemaphoreGive(g_data_mutex);
    }
    printf("PID Task: Target RPM=%.1f, Initial Target Heading=%.1f\n", TARGET_DEMO_RPM, g_target_heading);


    for (;;) {
        // Wait for the exact period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Take the mutex to gain exclusive access to sensor/PID data
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) { // Wait max 5ms for mutex

            // Run the main controller logic
            pid_controller_run();

            // Release the mutex
            xSemaphoreGive(g_data_mutex);
        } else {
             printf("PID Task: Failed to get mutex!\n");
             // Handle mutex acquisition failure if necessary (e.g., skip control iteration)
        }
    }
}

/**
 * @brief Low-priority task for gathering and publishing telemetry.
 */
void telemetry_task(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_TASK_RUN_PERIOD_MS);

    for (;;) {
        // Wait for the exact period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Wait for MQTT to be ready
        if (!mqtt_is_connected()) {
            continue;
        }

        // Local variables to hold data
        float current_rpm_l = 0.0, current_rpm_r = 0.0, distance = 0.0, heading_filt = 0.0;

        // Take the mutex to safely read all sensor data at once
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) { // Wait max 50ms for mutex
            current_rpm_l = encoder_get_left_rpm();
            current_rpm_r = encoder_get_right_rpm();
            distance = encoder_get_distance_m();
            heading_filt = imu_get_filtered_heading();
            xSemaphoreGive(g_data_mutex);

            // Format data into a JSON string
            char telemetry_msg[256];
            snprintf(telemetry_msg, sizeof(telemetry_msg),
                     "{\"rpm_l\": %.2f, \"rpm_r\": %.2f, \"dist\": %.2f, \"heading_filt\": %.2f}",
                     current_rpm_l, current_rpm_r, distance, heading_filt);

            // Publish the data
            mqtt_publish_telemetry(telemetry_msg);
            printf("Telemetry: %s\n", telemetry_msg);

        } else {
            printf("Telemetry Task: Failed to get mutex!\n");
            // Skip publishing if mutex wasn't acquired
        }
    }
}

/**
 * @brief Low-priority task for managing the MQTT connection state.
 */
void mqtt_task(void *params) {
    printf("MQTT Task: Waiting for Wi-Fi...\n");
    // Ensure Wi-Fi is up before attempting MQTT connection. wifi_init is called before scheduler.
    // Small delay might be good practice.
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("MQTT Task: Connecting to MQTT broker...\n");
    mqtt_connect();

    for (;;) {
        // This function checks Wi-Fi and MQTT status and reconnects if needed.
        // It's designed to be called periodically.
        mqtt_check_reconnect();

        // Check connection status every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ====================================================================
//                       == MAIN ==
// ====================================================================

int main() {
    stdio_init_all();
    sleep_ms(3000); // Wait for serial monitor to connect
    printf("=== Demo 1: Basic Motion & Telemetry (FreeRTOS) ===\n");

    // 1. Initialize Wi-Fi FIRST (required by cyw43_arch_lwip_sys_freertos)
    printf("Initializing WiFi...\n");
    if (wifi_init() != 0) { // Call wifi_init from wifi_mqtt.c
        printf("!! Wi-Fi Init Failed. Halting. !!\n");
        while (true);
    }

    // 2. Initialize other hardware
    printf("Initializing Hardware...\n");
    motor_init();       // From motor.c
    encoder_init();     // From encoder.c
    if (!imu_driver_init()) { // From imu.c
        printf("!! IMU Init Failed. Halting. !!\n");
        while (true);
    }
    pid_init();         // From pid.c

    // 3. Create global mutex
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL) {
        printf("!! Failed to create mutex. Halting. !!\n");
        while (true);
    }

    // 4. Create Tasks
    printf("Creating FreeRTOS tasks...\n");
    BaseType_t pid_status = xTaskCreate(pid_task, "PID_Task", configMINIMAL_STACK_SIZE * 2, NULL, PID_TASK_PRIORITY, NULL);
    BaseType_t tel_status = xTaskCreate(telemetry_task, "Telemetry_Task", configMINIMAL_STACK_SIZE * 2, NULL, TELEMETRY_TASK_PRIORITY, NULL);
    BaseType_t mq_status  = xTaskCreate(mqtt_task, "MQTT_Task", configMINIMAL_STACK_SIZE * 2, NULL, MQTT_TASK_PRIORITY, NULL);

    if (pid_status != pdPASS || tel_status != pdPASS || mq_status != pdPASS) {
        printf("!! Failed to create one or more tasks. Halting. !!\n");
         while(true);
    }


    // 5. Start the scheduler
    printf("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();

    // The scheduler should never return.
    while (true) {
        // Should not get here
    }
}