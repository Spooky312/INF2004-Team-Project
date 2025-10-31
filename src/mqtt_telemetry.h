/**
 * @file mqtt_telemetry.h
 * @brief MQTT telemetry for robot data publishing
 * @date October 2025
 */

#ifndef MQTT_TELEMETRY_H
#define MQTT_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

// Wi-Fi credentials
#define WIFI_SSID CFG_WIFI_SSID
#define WIFI_PASS CFG_WIFI_PASS

// MQTT broker settings
#define MQTT_SERVER_IP CFG_MQTT_SERVER_IP
#define MQTT_PORT CFG_MQTT_PORT

// MQTT topics
#define MQTT_TOPIC_STATUS "robot/status"
#define MQTT_TOPIC_MOTOR "robot/motor"
#define MQTT_TOPIC_OBSTACLE "robot/obstacle"
#define MQTT_TOPIC_LINE "robot/line"
#define MQTT_TOPIC_IMU "robot/imu"
#define MQTT_TOPIC_EVENT "robot/event"

/**
 * @brief MQTT connection status
 */
typedef enum {
    MQTT_STATUS_DISCONNECTED,
    MQTT_STATUS_CONNECTING,
    MQTT_STATUS_CONNECTED,
    MQTT_STATUS_ERROR
} mqtt_status_t;

/**
 * @brief Robot event types for logging
 */
typedef enum {
    EVENT_STARTUP,
    EVENT_LINE_FOLLOWING,
    EVENT_OBSTACLE_DETECTED,
    EVENT_OBSTACLE_SCANNING,
    EVENT_OBSTACLE_AVOIDING,
    EVENT_REJOINING_LINE,
    EVENT_LINE_LOST,
    EVENT_ERROR
} robot_event_t;

/**
 * @brief Initialize MQTT telemetry system
 * Connects to Wi-Fi and MQTT broker
 * @return true if successful
 */
bool mqtt_telemetry_init(void);

/**
 * @brief Check if MQTT is connected
 */
bool mqtt_is_connected(void);

/**
 * @brief Get current MQTT status
 */
mqtt_status_t mqtt_get_status(void);

/**
 * @brief Poll network stack (must be called regularly)
 * 
 * NOTE: When using pico_cyw43_arch_lwip_sys_freertos, this function
 * is a no-op because lwIP runs in its own FreeRTOS task.
 * Kept for API compatibility only.
 */
void mqtt_telemetry_poll(void);

/**
 * @brief Publish motor telemetry
 * Format: "rpm_l:%.2f,rpm_r:%.2f,enc_l:%u,enc_r:%u"
 */
bool mqtt_publish_motor_data(float rpm_left, float rpm_right, 
                             uint32_t enc_left, uint32_t enc_right);

/**
 * @brief Publish obstacle scan data
 * Format: "detected:%d,width:%.2f,left:%.2f,center:%.2f,right:%.2f,path:%s"
 */
bool mqtt_publish_obstacle_data(bool detected, float width_cm, 
                               float left_cm, float center_cm, float right_cm,
                               const char *chosen_path);

/**
 * @brief Publish line sensor data
 * Format: "on_line:%d,error:%d,left:%u,right:%u"
 */
bool mqtt_publish_line_data(bool on_line, int16_t error, 
                           uint16_t left_sensor, uint16_t right_sensor);

/**
 * @brief Publish IMU data (accelerometer and magnetometer)
 * Format: "ax:%d,ay:%d,az:%d,mx:%d,my:%d,mz:%d"
 */
bool mqtt_publish_imu_data(int16_t ax, int16_t ay, int16_t az,
                          int16_t mx, int16_t my, int16_t mz);

/**
 * @brief Publish robot event/state change
 * Format: "event:%s,timestamp:%u"
 */
bool mqtt_publish_event(robot_event_t event);

/**
 * @brief Publish general status message
 */
bool mqtt_publish_status(const char *status_msg);

/**
 * @brief Reconnect to MQTT if disconnected
 * Returns immediately if already connected
 */
void mqtt_reconnect_if_needed(void);

#endif // MQTT_TELEMETRY_H
