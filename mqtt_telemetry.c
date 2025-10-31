/**
 * @file mqtt_telemetry.c
 * @brief MQTT telemetry implementation
 */

#include "mqtt_telemetry.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include <string.h>
#include <stdio.h>
#include "config.h"

// Internal state
static mqtt_client_t *mqtt_client = NULL;
static ip_addr_t server_ip;
static mqtt_status_t current_status = MQTT_STATUS_DISCONNECTED;
static absolute_time_t last_reconnect_attempt = {0};

#define RECONNECT_DELAY_MS 5000

/**
 * @brief Event names for logging
 */
static const char* event_names[] = {
    "STARTUP",
    "LINE_FOLLOWING", 
    "OBSTACLE_DETECTED",
    "OBSTACLE_SCANNING",
    "OBSTACLE_AVOIDING",
    "REJOINING_LINE",
    "LINE_LOST",
    "ERROR"
};

/**
 * @brief MQTT incoming publish callback
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("[MQTT] Message on topic: %s\n", topic);
}

/**
 * @brief MQTT incoming data callback
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("[MQTT] Data: %.*s\n", len, data);
}

/**
 * @brief MQTT connection callback
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Connected successfully!\n");
        current_status = MQTT_STATUS_CONNECTED;
        
        // Register callbacks
        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, 
                               mqtt_incoming_data_cb, NULL);
        
        // Subscribe to command topics if needed
        // mqtt_subscribe(client, "robot/command", 0, NULL, NULL);
        
    } else {
        printf("[MQTT] Connection failed, status: %d\n", status);
        current_status = MQTT_STATUS_ERROR;
    }
}

/**
 * @brief Connect to MQTT broker
 */
static bool connect_to_mqtt(void) {
    if (mqtt_client != NULL) {
        mqtt_disconnect(mqtt_client);
        mqtt_client = NULL;
    }

    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("[MQTT] Failed to create client\n");
        return false;
    }

    IP4_ADDR(&server_ip, 172, 20, 10, 4);
    
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_robot";
    ci.keep_alive = 60;
    
    current_status = MQTT_STATUS_CONNECTING;
    err_t err = mqtt_client_connect(mqtt_client, &server_ip, MQTT_PORT,
                                    mqtt_connection_cb, NULL, &ci);
    
    if (err != ERR_OK) {
        printf("[MQTT] Connection initiation failed: %d\n", err);
        current_status = MQTT_STATUS_ERROR;
        return false;
    }
    
    return true;
}

bool mqtt_telemetry_init(void) {
    // Initialize Wi-Fi
    if (cyw43_arch_init()) {
        printf("[MQTT] Wi-Fi init failed\n");
        return false;
    }
    
    cyw43_arch_enable_sta_mode();
    printf("[MQTT] Connecting to Wi-Fi: %s\n", WIFI_SSID);
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, 
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[MQTT] Wi-Fi connection failed\n");
        return false;
    }
    
    printf("[MQTT] Wi-Fi connected\n");
    
    // Print IP address
    extern struct netif *netif_default;
    printf("[MQTT] IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    
    sleep_ms(1000);
    
    // Connect to MQTT
    return connect_to_mqtt();
}

bool mqtt_is_connected(void) {
    return (current_status == MQTT_STATUS_CONNECTED);
}

mqtt_status_t mqtt_get_status(void) {
    return current_status;
}

/**
 * @brief Poll network stack (DEPRECATED with FreeRTOS integration)
 * 
 * NOTE: When using pico_cyw43_arch_lwip_sys_freertos, lwIP runs in its
 * own FreeRTOS task and does not need manual polling. This function is
 * kept for API compatibility but does nothing.
 */
void mqtt_telemetry_poll(void) {
    // No-op: lwIP runs in its own FreeRTOS task
    // cyw43_arch_poll() is not needed with sys_freertos variant
}

void mqtt_reconnect_if_needed(void) {
    if (current_status == MQTT_STATUS_CONNECTED) {
        return;
    }
    
    // Rate limit reconnection attempts
    if (absolute_time_diff_us(last_reconnect_attempt, get_absolute_time()) 
        < RECONNECT_DELAY_MS * 1000) {
        return;
    }
    
    last_reconnect_attempt = get_absolute_time();
    
    // Check Wi-Fi link status
    // NOTE: With sys_freertos, lwIP runs in its own task
    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link_status != CYW43_LINK_UP) {
        printf("[MQTT] Wi-Fi disconnected, reconnecting...\n");
        current_status = MQTT_STATUS_DISCONNECTED;
        cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, 
                                           CYW43_AUTH_WPA2_AES_PSK, 10000);
        return;
    }
    
    // Wi-Fi is up, try MQTT connection
    printf("[MQTT] Attempting reconnection...\n");
    connect_to_mqtt();
}

bool mqtt_publish_motor_data(float rpm_left, float rpm_right, 
                            uint32_t enc_left, uint32_t enc_right) {
    if (!mqtt_is_connected()) return false;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "rpm_l:%.2f,rpm_r:%.2f,enc_l:%u,enc_r:%u",
             rpm_left, rpm_right, enc_left, enc_right);
    
    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_MOTOR, 
                            buffer, strlen(buffer), 0, 0, NULL, NULL);
    
    return (err == ERR_OK);
}

bool mqtt_publish_obstacle_data(bool detected, float width_cm,
                               float left_cm, float center_cm, float right_cm,
                               const char *chosen_path) {
    if (!mqtt_is_connected()) return false;
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "detected:%d,width:%.2f,left:%.2f,center:%.2f,right:%.2f,path:%s",
             detected, width_cm, left_cm, center_cm, right_cm, chosen_path);
    
    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_OBSTACLE,
                            buffer, strlen(buffer), 0, 0, NULL, NULL);
    
    return (err == ERR_OK);
}

bool mqtt_publish_line_data(bool on_line, int16_t error,
                           uint16_t left_sensor, uint16_t right_sensor) {
    if (!mqtt_is_connected()) return false;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "on_line:%d,error:%d,left:%u,right:%u",
             on_line, error, left_sensor, right_sensor);
    
    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_LINE,
                            buffer, strlen(buffer), 0, 0, NULL, NULL);
    
    return (err == ERR_OK);
}

bool mqtt_publish_imu_data(int16_t ax, int16_t ay, int16_t az,
                          int16_t mx, int16_t my, int16_t mz) {
    if (!mqtt_is_connected()) return false;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "ax:%d,ay:%d,az:%d,mx:%d,my:%d,mz:%d",
             ax, ay, az, mx, my, mz);
    
    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_IMU,
                            buffer, strlen(buffer), 0, 0, NULL, NULL);
    
    return (err == ERR_OK);
}

bool mqtt_publish_event(robot_event_t event) {
    if (!mqtt_is_connected()) return false;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "event:%s,timestamp:%u",
             event_names[event], to_ms_since_boot(get_absolute_time()));
    
    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_EVENT,
                            buffer, strlen(buffer), 0, 0, NULL, NULL);
    
    return (err == ERR_OK);
}

bool mqtt_publish_status(const char *status_msg) {
    if (!mqtt_is_connected()) return false;
    
    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_STATUS,
                            status_msg, strlen(status_msg), 0, 0, NULL, NULL);
    
    return (err == ERR_OK);
}
