// ===============================================
//  Module: Wi-Fi and MQTT Client
//  Description: Handles connection to hotspot and telemetry publishing.
// ===============================================
#include "wifi_mqtt.h"
#include <stdio.h>
#include <string.h>

static bool wifi_ok = false;
static bool mqtt_ok = false;
static mqtt_client_t *client;


static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        mqtt_ok = true;
        printf("MQTT connected successfully.\n");
    }
    else
    {
        mqtt_ok = false;
        printf("MQTT disconnected.\n");
    }
}

void wifi_connect_init(void)
{
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed!\n");
        return;
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
    return wifi_ok;
}

void mqtt_init(void) {
    // Create a new MQTT client instance
    client = mqtt_client_new();
    if (client == NULL) {
        printf("Failed to create MQTT client.\n");
        return;
    }

    // Connect to the broker (example)
    ip_addr_t broker_ip;
    ip4addr_aton(MQTT_SERVER_IP, &broker_ip);

    mqtt_client_connect(client, &broker_ip, MQTT_PORT, mqtt_connection_cb, NULL, NULL);
}

bool mqtt_is_connected(void)
{
    return mqtt_ok;
}

void wifi_mqtt_publish(const char *topic, const char *msg) {
    err_t err = mqtt_publish(client, topic, msg, strlen(msg), 0, 0, NULL, NULL);
    if (err != ERR_OK) {
        printf("MQTT publish failed: %d\n", err);
    }
}


void mqtt_loop(void)
{
    cyw43_arch_poll();
}
