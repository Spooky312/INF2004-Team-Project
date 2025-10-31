// ===============================================
//  Module: Wi-Fi and MQTT Client
//  Description: Handles connection to hotspot and telemetry publishing.
// ===============================================
#include "wifi_mqtt.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/cyw43_arch.h"



static bool wifi_ok = false;
static bool mqtt_ok = false;
static mqtt_client_t *client = NULL;

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const char *status_str;
    switch (status) {
        case MQTT_CONNECT_ACCEPTED:
            status_str = "ACCEPTED";
            mqtt_ok = true;
            break;
        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
            status_str = "REFUSED (protocol version)";
            mqtt_ok = false;
            break;
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
            status_str = "REFUSED (identifier)";
            mqtt_ok = false;
            break;
        case MQTT_CONNECT_REFUSED_SERVER:
            status_str = "REFUSED (server unavailable)";
            mqtt_ok = false;
            break;
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
            status_str = "REFUSED (bad username/password)";
            mqtt_ok = false;
            break;
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            status_str = "REFUSED (not authorized)";
            mqtt_ok = false;
            break;
        case MQTT_CONNECT_DISCONNECTED:
            status_str = "DISCONNECTED";
            mqtt_ok = false;
            break;
        case MQTT_CONNECT_TIMEOUT:
            status_str = "TIMEOUT";
            mqtt_ok = false;
            break;
        default:
            status_str = "UNKNOWN";
            mqtt_ok = false;
            break;
    }
    
    printf("MQTT connection status: %s\n", status_str);
}

void wifi_connect_init(void)
{
    printf("wifi_connect_init: Starting...\n");
    
    if (cyw43_arch_init()) {
        printf("❌ cyw43 init failed\n");
        while (1);
    }



    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi: %s...\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 15000))
    {
        printf("Wi-Fi connection failed.\n");
        return;
    }

    wifi_ok = true;
    printf("Connected to Wi-Fi.\n");
}

bool wifi_is_connected(void)
{
    return wifi_ok && (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP);
}

void mqtt_init(void) {
    printf("mqtt_init: Creating MQTT client...\n");
    
    // Create a new MQTT client instance
    client = mqtt_client_new();
    if (client == NULL) {
        printf("❌ Failed to create MQTT client.\n");
        return;
    }
    printf("✅ MQTT client created.\n");

    // Parse broker IP
    ip_addr_t broker_ip;
    if (!ip4addr_aton(MQTT_SERVER_IP, &broker_ip)) {
        printf("❌ Invalid MQTT server IP: %s\n", MQTT_SERVER_IP);
        return;
    }
    printf("Parsed broker IP: %s\n", MQTT_SERVER_IP);

    // Prepare client info (optional but good practice)
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_w_robot";  // Unique client ID
    ci.keep_alive = 60;  // seconds
    ci.will_topic = NULL;  // No last will
    ci.will_msg = NULL;
    ci.will_qos = 0;
    ci.will_retain = 0;
    
    // Connect to the broker
    printf("Connecting to MQTT broker at %s:%d...\n", MQTT_SERVER_IP, MQTT_PORT);
    err_t err = mqtt_client_connect(
        client, 
        &broker_ip, 
        MQTT_PORT, 
        mqtt_connection_cb, 
        NULL,  // arg for callback
        &ci     // client info
    );
    
    if (err != ERR_OK) {
        printf("❌ MQTT connection request failed: %d\n", err);
        return;
    }
    
    printf("MQTT connection request sent, waiting for callback...\n");
}

bool mqtt_is_connected(void)
{
    return mqtt_ok && client != NULL && mqtt_client_is_connected(client);
}

void wifi_mqtt_publish(const char *topic, const char *msg) {
    if (client == NULL) {
        printf("❌ MQTT client not initialized.\n");
        return;
    }
    
    if (!mqtt_client_is_connected(client)) {
        printf("❌ MQTT client not connected.\n");
        return;
    }
    
    err_t err = mqtt_publish(
        client, 
        topic, 
        msg, 
        strlen(msg), 
        0,  // QoS 0
        0,  // Not retained
        NULL,  // No publish callback
        NULL   // No callback arg
    );
    
    if (err != ERR_OK) {
        printf("❌ MQTT publish failed: %d\n", err);
    }
}

void mqtt_loop(void)
{
    // Poll the CYW43 driver to keep network stack running
    // This is safe to call from FreeRTOS task
    cyw43_arch_poll();
    
    // Small yield to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(1));
}