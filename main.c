/**
 * @file main.c
 * @brief Main - FreeRTOS Integration
 * 
 * Architecture:
 * - Barcode Task: Runs barcode_decoder_update() at 1kHz, sends results to IMU
 * - Line Task: Runs line_detection_read() at 100Hz, sends data to IMU
 * - IMU Task: Receives sensor data, simulates robot behavior
 * - Telemetry Task: Prints status periodically
 */

#include <stdio.h>
#include "pico/stdlib.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Module headers
#include "barcode_decoder.h"
#include "line_detection.h"
#include "simulated_imu.h"

// =============================================================================
// CONFIGURATION
// =============================================================================

// Task priorities
#define PRIORITY_BARCODE    (tskIDLE_PRIORITY + 4)
#define PRIORITY_LINE       (tskIDLE_PRIORITY + 3)
#define PRIORITY_IMU        (tskIDLE_PRIORITY + 3)
#define PRIORITY_TELEMETRY  (tskIDLE_PRIORITY + 2)

// Stack sizes
#define STACK_BARCODE       512
#define STACK_LINE          256
#define STACK_IMU           384
#define STACK_TELEMETRY     256

// Task rates (ms)
#define BARCODE_RATE_MS     1       // 1kHz for edge detection
#define LINE_RATE_MS        10      // 100Hz line reading
#define IMU_RATE_MS         10      // 100Hz IMU update
#define TELEMETRY_RATE_MS   1000    // 1Hz status print

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Queues for inter-task communication
QueueHandle_t barcode_queue;
QueueHandle_t line_queue;

// =============================================================================
// TASK IMPLEMENTATIONS
// =============================================================================

/**
 * @brief Barcode Decoder Task
 * Continuously updates barcode decoder and sends results to IMU
 */
void barcode_task(void *params) {
    (void)params;
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BARCODE_RATE_MS);
    
    printf("[TASK] Barcode task started\n");
    
    while (1) {
        // Update barcode decoder (processes edges)
        barcode_decoder_update();
        
        // Check if new barcode decoded
        barcode_result_t result;
        if (barcode_decoder_get_result(&result)) {
            printf("[BARCODE] Decoded: \"%s\" -> %s\n",
                   result.decoded_string,
                   barcode_command_to_string(result.command));
            
            // Send to queue for IMU task
            xQueueSend(barcode_queue, &result, 0);
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Line Detection Task
 * Reads line sensor and sends data to IMU
 */
void line_task(void *params) {
    (void)params;
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LINE_RATE_MS);
    
    printf("[TASK] Line detection task started\n");
    
    while (1) {
        // Read line sensor
        line_data_t data;
        line_detection_read(&data);
        
        // Send to queue for IMU task (overwrite old data)
        xQueueOverwrite(line_queue, &data);
        
        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Simulated IMU Task
 * Receives sensor data and simulates robot behavior
 */
void imu_task(void *params) {
    (void)params;
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_RATE_MS);
    
    barcode_result_t barcode_data;
    line_data_t line_data;
    
    printf("[TASK] IMU task started\n");
    
    while (1) {
        // Check for barcode data
        if (xQueueReceive(barcode_queue, &barcode_data, 0) == pdTRUE) {
            simulated_imu_process_barcode(&barcode_data);
        }
        
        // Check for line data
        if (xQueuePeek(line_queue, &line_data, 0) == pdTRUE) {
            simulated_imu_process_line(&line_data);
        }
        
        // Update IMU simulation
        simulated_imu_update();
        
        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Telemetry Task
 * Prints system status periodically
 */
void telemetry_task(void *params) {
    (void)params;
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TELEMETRY_RATE_MS);
    
    printf("[TASK] Telemetry task started\n");
    
    uint32_t iteration = 0;
    
    while (1) {
        // Print header every 10 iterations
        if ((iteration % 10) == 0) {
            printf("\n=== Demo 2 Status (iteration %u) ===\n", iteration);
        }
        
        // Print IMU status
        simulated_imu_print_status();
        
        // Print line sensor status
        line_data_t line_data;
        if (xQueuePeek(line_queue, &line_data, 0) == pdTRUE) {
            printf("[LINE] ADC=%u, Position=%d, Detected=%s\n",
                   line_data.raw_adc,
                   line_data.position,
                   line_data.line_detected ? "YES" : "NO");
        }
        
        printf("\n");
        iteration++;
        
        vTaskDelayUntil(&last_wake, period);
    }
}

// =============================================================================
// MAIN
// =============================================================================

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial
    
    printf("\n");
    printf("========================================\n");
    printf("        Line Following + Barcode    \n");
    printf("      Navigation + Simulated IMU          \n");
    printf("========================================\n\n");
    
    // Initialize hardware modules
    printf("Initializing modules...\n");
    barcode_decoder_init();
    line_detection_init();
    simulated_imu_init();
    printf("Modules initialized\n\n");
    
    // Create queues
    barcode_queue = xQueueCreate(3, sizeof(barcode_result_t));
    line_queue = xQueueCreate(1, sizeof(line_data_t));  // Only need latest
    
    if (!barcode_queue || !line_queue) {
        printf("ERROR: Failed to create queues!\n");
        while (1) tight_loop_contents();
    }
    
    printf("Queues created\n");
    
    // Create tasks
    BaseType_t status;
    
    status = xTaskCreate(barcode_task, 
                        "Barcode",
                        STACK_BARCODE,
                        NULL,
                        PRIORITY_BARCODE,
                        NULL);
    if (status != pdPASS) {
        printf("ERROR: Failed to create barcode task!\n");
        while (1) tight_loop_contents();
    }
    
    status = xTaskCreate(line_task,
                        "Line",
                        STACK_LINE,
                        NULL,
                        PRIORITY_LINE,
                        NULL);
    if (status != pdPASS) {
        printf("ERROR: Failed to create line task!\n");
        while (1) tight_loop_contents();
    }
    
    status = xTaskCreate(imu_task,
                        "IMU",
                        STACK_IMU,
                        NULL,
                        PRIORITY_IMU,
                        NULL);
    if (status != pdPASS) {
        printf("ERROR: Failed to create IMU task!\n");
        while (1) tight_loop_contents();
    }
    
    status = xTaskCreate(telemetry_task,
                        "Telemetry",
                        STACK_TELEMETRY,
                        NULL,
                        PRIORITY_TELEMETRY,
                        NULL);
    if (status != pdPASS) {
        printf("ERROR: Failed to create telemetry task!\n");
        while (1) tight_loop_contents();
    }
    
    printf("Tasks created\n\n");
    printf("Starting FreeRTOS scheduler...\n");
    printf("========================================\n\n");
    
    // Start scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    printf("ERROR: Scheduler failed to start!\n");
    while (1) {
        tight_loop_contents();
    }
    
    return 0;
}