// mqtt.c
#include "mqtt.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdio.h>

static mqtt_client_t *client;
static ip_addr_t server_ip;
static bool mqtt_connected = false;
static char g_topic[64];

static void mqtt_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    printf(mqtt_connected ? "MQTT connected\n" : "MQTT disconnected\n");
}

bool mqtt_setup(const char *ssid, const char *pass,
                const char *broker_ip, const char *topic) {
    if (cyw43_arch_init()) return false;
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 30000)) return false;
    IP4_ADDR(&server_ip, 172,20,10,4);
    client = mqtt_client_new();
    strcpy(g_topic, topic);
    struct mqtt_connect_client_info_t ci = { .client_id = "pico_demo3" };
    return mqtt_client_connect(client, &server_ip, 1883, mqtt_cb, NULL, &ci) == ERR_OK;
}

void mqtt_publish_message(const char *topic, const char *msg) {
    if (mqtt_connected && client)
        mqtt_publish(client, topic, msg, strlen(msg), 0, 0, NULL, NULL);
}

void mqtt_task(void *params) {
    while (1) {
        cyw43_arch_poll();
        sleep_ms(50);
    }
}
