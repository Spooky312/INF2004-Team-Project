/**
 * @file main.c
 * @brief Demo 3: Obstacle Detection + Avoidance + Telemetry with FreeRTOS
 * 
 * Purpose: Line following robot with ultrasonic obstacle detection,
 *          servo-based scanning, IMU-assisted navigation, and MQTT telemetry
 * 
 * Tasks:
 * 1. Line Following Task - PID control for line tracking
 * 2. Obstacle Detection Task - Scans and measures obstacles
 * 3. Telemetry Task - Publishes data via MQTT
 * 4. IMU Task - Reads sensor data for navigation
 * 
 * State Machine:
 * FOLLOW_LINE -> OBSTACLE_DETECTED -> SCANNING -> AVOIDING -> REJOINING -> FOLLOW_LINE
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "config.h"
#include "motor_control.h"
#include "obstacle_detection.h"
#include "mqtt_telemetry.h"
#include "imu.h"

// Task priorities
#define PRIORITY_LINE_FOLLOW    (tskIDLE_PRIORITY + 3)
#define PRIORITY_OBSTACLE_CHECK (tskIDLE_PRIORITY + 4)
#define PRIORITY_TELEMETRY      (tskIDLE_PRIORITY + 1)
#define PRIORITY_IMU            (tskIDLE_PRIORITY + 2)

// Task periods (ms)
#define LINE_FOLLOW_PERIOD_MS   50
#define OBSTACLE_CHECK_PERIOD_MS 100
#define TELEMETRY_PERIOD_MS     500
#define IMU_PERIOD_MS           100

/**
 * @brief Robot state machine
 */
typedef enum {
    STATE_INIT,
    STATE_FOLLOW_LINE,
    STATE_OBSTACLE_DETECTED,
    STATE_SCANNING,
    STATE_AVOIDING,
    STATE_REJOINING,
    STATE_ERROR
} robot_state_t;

/**
 * @brief Shared state structure
 */
typedef struct {
    robot_state_t state;
    obstacle_scan_t last_scan;
    line_data_t line_data;
    motor_telemetry_t motor_telem;
    imu_accel_raw_t accel_data;
    imu_mag_raw_t mag_data;
    bool imu_available;
    uint32_t state_change_time;
} robot_data_t;

// Global shared data
static robot_data_t g_robot_data = {0};
static SemaphoreHandle_t g_data_mutex = NULL;

/**
 * @brief IMU configuration
 */
static imu_config_t imu_config = {
    .i2c_port = i2c0,
    .sda_pin = 4,
    .scl_pin = 5,
    .i2c_freq = 100000,
    .scale = IMU_SCALE_2G,
    .mag_gain = IMU_MAG_GAIN_1_3_GAUSS
};

/**
 * @brief Change robot state (thread-safe)
 */
static void set_robot_state(robot_state_t new_state) {
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
        if (g_robot_data.state != new_state) {
            g_robot_data.state = new_state;
            g_robot_data.state_change_time = to_ms_since_boot(get_absolute_time());
            
            // Log state change via MQTT
            robot_event_t event;
            switch (new_state) {
                case STATE_FOLLOW_LINE:
                    event = EVENT_LINE_FOLLOWING;
                    break;
                case STATE_OBSTACLE_DETECTED:
                    event = EVENT_OBSTACLE_DETECTED;
                    break;
                case STATE_SCANNING:
                    event = EVENT_OBSTACLE_SCANNING;
                    break;
                case STATE_AVOIDING:
                    event = EVENT_OBSTACLE_AVOIDING;
                    break;
                case STATE_REJOINING:
                    event = EVENT_REJOINING_LINE;
                    break;
                default:
                    event = EVENT_ERROR;
            }
            mqtt_publish_event(event);
            
            printf("[STATE] -> %d\n", new_state);
        }
        xSemaphoreGive(g_data_mutex);
    }
}

/**
 * @brief Get current robot state (thread-safe)
 */
static robot_state_t get_robot_state(void) {
    robot_state_t state;
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
        state = g_robot_data.state;
        xSemaphoreGive(g_data_mutex);
    }
    return state;
}

/**
 * @brief Task 1: Line Following
 * Reads line sensors and controls motors for line tracking
 */
static void line_follow_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        robot_state_t current_state = get_robot_state();
        
        // Read line sensors
        line_data_t line_data = motor_read_line_sensors();
        
        // Update shared data
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10))) {
            g_robot_data.line_data = line_data;
            xSemaphoreGive(g_data_mutex);
        }
        
        // Only control motors when in line following states
        if (current_state == STATE_FOLLOW_LINE || current_state == STATE_REJOINING) {
            if (line_data.on_line) {
                motor_line_follow(&line_data);
            } else {
                // Lost the line
                motor_stop();
                if (current_state == STATE_FOLLOW_LINE) {
                    mqtt_publish_event(EVENT_LINE_LOST);
                    printf("[LINE] Lost the line!\n");
                }
            }
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LINE_FOLLOW_PERIOD_MS));
    }
}

/**
 * @brief Task 2: Obstacle Detection and Avoidance
 * Checks for obstacles and manages avoidance maneuvers
 */
static void obstacle_detection_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        robot_state_t current_state = get_robot_state();
        
        switch (current_state) {
            case STATE_FOLLOW_LINE: {
                // Periodically check for obstacles ahead
                if (obstacle_check_ahead()) {
                    printf("[OBSTACLE] Detected ahead!\n");
                    motor_stop();
                    set_robot_state(STATE_OBSTACLE_DETECTED);
                }
                break;
            }
            
            case STATE_OBSTACLE_DETECTED: {
                // Stop and prepare to scan
                motor_stop();
                vTaskDelay(pdMS_TO_TICKS(300));
                set_robot_state(STATE_SCANNING);
                break;
            }
            
            case STATE_SCANNING: {
                // Perform full obstacle scan
                printf("[OBSTACLE] Scanning...\n");
                obstacle_scan_t scan = obstacle_perform_scan();
                
                // Measure obstacle width if detected
                if (scan.obstacle_present) {
                    scan.obstacle_width_cm = obstacle_measure_width();
                    printf("[OBSTACLE] Width: %.2f cm\n", scan.obstacle_width_cm);
                }
                
                // Update shared data
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50))) {
                    g_robot_data.last_scan = scan;
                    xSemaphoreGive(g_data_mutex);
                }
                
                // Publish obstacle data
                const char *path_str = (scan.recommended_path == SCAN_LEFT) ? "LEFT" :
                                      (scan.recommended_path == SCAN_RIGHT) ? "RIGHT" : "CENTER";
                mqtt_publish_obstacle_data(scan.obstacle_present, scan.obstacle_width_cm,
                                          scan.left_clearance_cm, scan.center_distance_cm,
                                          scan.right_clearance_cm, path_str);
                
                if (!scan.obstacle_present) {
                    // False alarm, resume
                    set_robot_state(STATE_FOLLOW_LINE);
                } else {
                    set_robot_state(STATE_AVOIDING);
                }
                break;
            }
            
            case STATE_AVOIDING: {
                // Execute avoidance maneuver
                obstacle_scan_t scan;
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10))) {
                    scan = g_robot_data.last_scan;
                    xSemaphoreGive(g_data_mutex);
                }
                
                printf("[OBSTACLE] Avoiding via %s\n",
                       scan.recommended_path == SCAN_LEFT ? "LEFT" : "RIGHT");
                
                // Simple avoidance: turn, move forward, turn back
                if (scan.recommended_path == SCAN_LEFT) {
                    // Turn left
                    motor_pivot_left(TURN_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(800));
                    
                    // Move forward past obstacle
                    motor_forward(LINE_FOLLOWING_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    
                    // Turn right to rejoin
                    motor_pivot_right(TURN_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(800));
                    
                } else {
                    // Turn right
                    motor_pivot_right(TURN_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(800));
                    
                    // Move forward past obstacle
                    motor_forward(LINE_FOLLOWING_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    
                    // Turn left to rejoin
                    motor_pivot_left(TURN_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(800));
                }
                
                motor_stop();
                set_robot_state(STATE_REJOINING);
                break;
            }
            
            case STATE_REJOINING: {
                // Move forward slowly until line is found
                motor_forward(LINE_FOLLOWING_SPEED * 0.5f);
                
                line_data_t line_data;
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10))) {
                    line_data = g_robot_data.line_data;
                    xSemaphoreGive(g_data_mutex);
                }
                
                if (line_data.on_line) {
                    printf("[OBSTACLE] Rejoined line!\n");
                    set_robot_state(STATE_FOLLOW_LINE);
                }
                
                // Timeout after 3 seconds
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10))) {
                    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) 
                                      - g_robot_data.state_change_time;
                    if (elapsed > 3000) {
                        printf("[OBSTACLE] Rejoin timeout, resuming anyway\n");
                        set_robot_state(STATE_FOLLOW_LINE);
                    }
                    xSemaphoreGive(g_data_mutex);
                }
                break;
            }
            
            default:
                break;
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(OBSTACLE_CHECK_PERIOD_MS));
    }
}

/**
 * @brief Task 3: Telemetry Publishing
 * Publishes all sensor and status data via MQTT
 * 
 * NOTE: With pico_cyw43_arch_lwip_sys_freertos, lwIP runs in its own
 * FreeRTOS task. No manual polling is needed - we just publish data.
 */
static void telemetry_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Reconnect if needed (checks Wi-Fi status internally)
        mqtt_reconnect_if_needed();
        
        if (mqtt_is_connected()) {
            // Publish motor telemetry
            motor_telemetry_t telem = motor_get_telemetry();
            mqtt_publish_motor_data(telem.rpm_left, telem.rpm_right,
                                   telem.encoder_left, telem.encoder_right);
            
            // Publish line sensor data
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10))) {
                line_data_t line = g_robot_data.line_data;
                xSemaphoreGive(g_data_mutex);
                mqtt_publish_line_data(line.on_line, line.error,
                                      line.left_sensor, line.right_sensor);
            }
            
            // Publish IMU data if available
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10))) {
                if (g_robot_data.imu_available) {
                    mqtt_publish_imu_data(g_robot_data.accel_data.x,
                                         g_robot_data.accel_data.y,
                                         g_robot_data.accel_data.z,
                                         g_robot_data.mag_data.x,
                                         g_robot_data.mag_data.y,
                                         g_robot_data.mag_data.z);
                }
                xSemaphoreGive(g_data_mutex);
            }
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

/**
 * @brief Task 4: IMU Reading
 * Reads accelerometer and magnetometer data
 */
static void imu_task(void *params) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        imu_accel_raw_t accel;
        imu_mag_raw_t mag;
        bool success = false;
        
        // Read accelerometer
        if (imu_data_available(&imu_config)) {
            if (imu_read_accel_raw(&imu_config, &accel)) {
                success = true;
            }
        }
        
        // Read magnetometer
        if (imu_mag_data_available(&imu_config)) {
            imu_mag_read_raw(&imu_config, &mag);
        }
        
        // Update shared data
        if (success && xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10))) {
            g_robot_data.accel_data = accel;
            g_robot_data.mag_data = mag;
            g_robot_data.imu_available = true;
            xSemaphoreGive(g_data_mutex);
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/**
 * @brief Main entry point
 */
int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Demo 3: Obstacle Detection + Avoidance + Telemetry ===\n");
    printf("Initializing systems...\n");
    
    // Initialize subsystems
    if (!motor_control_init()) {
        printf("ERROR: Motor init failed\n");
        return -1;
    }
    printf("✓ Motor control initialized\n");
    
    if (!obstacle_detection_init()) {
        printf("ERROR: Obstacle detection init failed\n");
        return -1;
    }
    printf("✓ Obstacle detection initialized\n");
    
    if (!imu_init(&imu_config)) {
        printf("WARNING: IMU init failed, continuing without IMU\n");
    } else {
        printf("✓ IMU initialized\n");
    }
    
    // if (!mqtt_telemetry_init()) {
    //     printf("WARNING: MQTT init failed, continuing without telemetry\n");
    // } else {
    //     printf("✓ MQTT telemetry initialized\n");
    // }
    
    // Create mutex
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL) {
        printf("ERROR: Failed to create mutex\n");
        return -1;
    }
    
    // Set initial state
    g_robot_data.state = STATE_FOLLOW_LINE;
    mqtt_publish_event(EVENT_STARTUP);
    
    printf("\nCreating FreeRTOS tasks...\n");
    
    // Create tasks
    xTaskCreate(line_follow_task, "LineFollow", 
                256, NULL, PRIORITY_LINE_FOLLOW, NULL);
    
    xTaskCreate(obstacle_detection_task, "ObstacleDetect",
                512, NULL, PRIORITY_OBSTACLE_CHECK, NULL);
    
    xTaskCreate(telemetry_task, "Telemetry",
                512, NULL, PRIORITY_TELEMETRY, NULL);
    
    xTaskCreate(imu_task, "IMU",
                256, NULL, PRIORITY_IMU, NULL);
    
    printf("✓ Tasks created\n");
    printf("\n*** Starting scheduler - Robot Active! ***\n\n");
    
    // Start scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    printf("ERROR: Scheduler failed to start\n");
    while (1) {
        tight_loop_contents();
    }
    
    return 0;
}
