// ===============================================
//  Module: MQTT Client
//  Description: MQTT client using lwIP MQTT library (NO_SYS mode)
// ===============================================

#include "mqtt.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>

// MQTT broker configuration
#ifndef MQTT_SERVER_IP
#define MQTT_SERVER_IP "192.168.43.1"  // Default broker IP
#endif

#define MQTT_SERVER_PORT 1883
#define MQTT_CLIENT_ID "pico_robocar"

// State
static mqtt_client_t *mqtt_client = NULL;
static volatile bool mqtt_connected = false;

// MQTT callbacks
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    (void)arg;
    
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] ✅ Connected to broker\n");
        mqtt_connected = true;
    } else {
        printf("[MQTT] ❌ Connection failed (status %d)\n", status);
        mqtt_connected = false;
    }
}

static void mqtt_publish_cb(void *arg, err_t err)
{
    (void)arg;
    
    if (err == ERR_OK) {
        // Successfully published - no need to spam logs
    } else {
        printf("[MQTT] ⚠️ Publish failed (error %d)\n", err);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    (void)arg;
    printf("[MQTT] Incoming publish: topic='%s', len=%lu\n", topic, tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    (void)arg;
    (void)flags;
    
    // Print received data (for subscribed topics)
    printf("[MQTT] Received data: %.*s\n", len, data);
}

void mqtt_app_init(void)
{
    printf("[MQTT] Initializing MQTT client...\n");
    
    // Create MQTT client
    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("[MQTT] ❌ Failed to create MQTT client\n");
        return;
    }
    
    // Set up connection info
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;
    ci.keep_alive = 60;  // 60 seconds
    
    // Parse broker IP address
    ip_addr_t broker_ip;
    if (!ip4addr_aton(MQTT_SERVER_IP, &broker_ip)) {
        printf("[MQTT] ❌ Invalid broker IP address: %s\n", MQTT_SERVER_IP);
        return;
    }
    
    printf("[MQTT] Connecting to broker %s:%d...\n", MQTT_SERVER_IP, MQTT_SERVER_PORT);
    
    // Connect to broker
    err_t err = mqtt_client_connect(
        mqtt_client,
        &broker_ip,
        MQTT_SERVER_PORT,
        mqtt_connection_cb,
        NULL,
        &ci
    );
    
    if (err != ERR_OK) {
        printf("[MQTT] ❌ Failed to initiate connection (error %d)\n", err);
        mqtt_connected = false;
    } else {
        printf("[MQTT] Connection initiated...\n");
    }
    
    // Set callbacks for incoming messages
    mqtt_set_inpub_callback(mqtt_client, 
                           mqtt_incoming_publish_cb,
                           mqtt_incoming_data_cb,
                           NULL);
}

bool mqtt_app_is_connected(void)
{
    if (mqtt_client == NULL) {
        return false;
    }
    
    // Check if client is connected
    bool connected = mqtt_client_is_connected(mqtt_client);
    mqtt_connected = connected;
    
    return connected;
}

bool mqtt_app_publish(const char *topic, const char *message, uint8_t qos, uint8_t retain)
{
    if (!mqtt_app_is_connected()) {
        return false;
    }
    
    // Publish message (no mutex needed in NO_SYS mode)
    err_t err = mqtt_publish(
        mqtt_client,
        topic,
        message,
        strlen(message),
        qos,
        retain,
        mqtt_publish_cb,
        NULL
    );
    
    if (err != ERR_OK) {
        printf("[MQTT] ⚠️ Publish failed (error %d)\n", err);
        return false;
    }
    
    return true;
}

void mqtt_app_disconnect(void)
{
    if (mqtt_client != NULL) {
        printf("[MQTT] Disconnecting...\n");
        mqtt_disconnect(mqtt_client);
        mqtt_connected = false;
    }
}
