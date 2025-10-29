#include "wifi_mqtt.h"
#include <string.h>
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/netif.h"

// Global state
static mqtt_client_t *client = NULL;
static ip_addr_t server_ip;
static bool mqtt_connected_status = false;

// --- MQTT Callbacks ---

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("Got MQTT message on topic: %s\n", topic);
    // This is where you would handle incoming commands
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("Message data: %.*s\n", len, data);
    // TODO: Parse command (e.g., "LEFT", "RIGHT") and set state
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    printf("\n=== MQTT Connection Callback ===\n");
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("Connected to MQTT broker successfully!\n");
        mqtt_connected_status = true;

        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
        
        // Subscribe to the "commands" topic
        err_t err = mqtt_subscribe(client, MQTT_TOPIC_COMMANDS, 0, NULL, NULL);
        if (err == ERR_OK) {
            printf("Subscribed to topic: %s\n", MQTT_TOPIC_COMMANDS);
        } else {
            printf("Subscription failed! Error code: %d\n", err);
        }

    } else {
        printf("MQTT connection failed or lost. Status code: %d\n", status);
        mqtt_connected_status = false;
    }
    printf("================================\n\n");
}

// --- Public Functions ---

int wifi_init(void) {
    // This function is special and *must* be called before vTaskStartScheduler
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Wi-Fi connect failed\n");
        return -1;
    }
    
    printf("Wi-Fi connected\n");
    extern struct netif *netif_default;
    printf("Pico IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    return 0;
}

void mqtt_connect(void) {
    if (client == NULL) {
        client = mqtt_client_new();
    }
    
    if (client == NULL) {
        printf("Failed to create MQTT client\n");
        return;
    }

    IP4_ADDR(&server_ip, 192, 168, 23, 8); // Using macro from header
    printf("Connecting to MQTT broker at %s:%d\n", ip4addr_ntoa(&server_ip), MQTT_PORT);

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id   = "pico_w_robot";
    ci.keep_alive  = 60;

    err_t err = mqtt_client_connect(client, &server_ip, MQTT_PORT,
                                    mqtt_connection_cb, NULL, &ci);

    if (err != ERR_OK) {
        printf("mqtt_client_connect returned error: %d\n", err);
    }
}

void mqtt_publish_telemetry(const char *msg) {
    if (!mqtt_connected_status || client == NULL) {
        return;
    }

    err_t pub_err = mqtt_publish(client, MQTT_TOPIC_TELEMETRY, msg, strlen(msg),
                                 0, 0, NULL, NULL);

    if (pub_err != ERR_OK) {
        printf("Publish failed! Error code: %d\n", pub_err);
    }
}

void mqtt_check_reconnect(void) {
    // NOTE: We do NOT call cyw43_arch_poll() here.
    // The pico_cyw43_arch_lwip_sys_freertos library handles polling
    // in a separate, hidden FreeRTOS task.

    // Check Wi-Fi link status
    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link_status != CYW43_LINK_UP) {
        printf("Wi-Fi dropped, reconnecting...\n");
        mqtt_connected_status = false;
        // This function is RTOS-aware and safe to call
        cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000);
        return; // Wait for next cycle
    }

    // Reconnect MQTT if needed
    if (!mqtt_connected_status && client != NULL) {
        printf("Attempting MQTT reconnect...\n");
        mqtt_connect();
    }
}

bool mqtt_is_connected(void) {
    return mqtt_connected_status;
}