#include "mqtt.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip4_addr.h"
#include "lwip/dns.h"

static mqtt_client_t *client = NULL;
static bool connected = false;

// --------------------------------------------------
// Connection callback
// --------------------------------------------------
static void mqtt_connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] ✅ Connected to broker.\n");
        connected = true;
    } else {
        printf("[MQTT] ❌ Disconnected (status=%d)\n", status);
        connected = false;
    }
}

// --------------------------------------------------
// Initialize + connect
// --------------------------------------------------
bool mqtt_app_init_and_connect(const char *broker_ip, int port, const char *client_id) {
    ip_addr_t ip;
    err_t err;

    if (!client) {
        client = mqtt_client_new();
        if (!client) {
            printf("[MQTT] ❌ mqtt_client_new() failed.\n");
            return false;
        }
    }

    if (!ip4addr_aton(broker_ip, &ip)) {
        printf("[MQTT] Resolving %s...\n", broker_ip);
        err = dns_gethostbyname(broker_ip, &ip, NULL, NULL);
        if (err != ERR_OK) {
            printf("[MQTT] ❌ DNS resolution failed (%d)\n", err);
            return false;
        }
    }

    printf("[MQTT] Connecting to broker %s:%d ...\n", broker_ip, port);

    struct mqtt_connect_client_info_t ci = {
        .client_id   = client_id,
        .keep_alive  = 30,
        .will_topic  = NULL,
        .will_msg    = NULL,
        .will_qos    = 0,
        .will_retain = 0
    };

    err = mqtt_client_connect(client, &ip, port, mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("[MQTT] ❌ Connect call failed (%d)\n", err);
        return false;
    }

    return true;
}

// --------------------------------------------------
// Connection state
// --------------------------------------------------
bool mqtt_app_is_connected(void) {
    return connected;
}

// --------------------------------------------------
// Publish wrapper
// --------------------------------------------------
void mqtt_app_publish(const char *topic, const char *payload, int qos, int retain) {
    if (!client || !connected) return;


    err_t err = mqtt_publish(client, topic, payload, strlen(payload), qos, retain, NULL, NULL);
    if (err != ERR_OK) {
        printf("[MQTT] ⚠️ Publish failed (%d)\n", err);
    }
}

// --------------------------------------------------
// Optional poll (no-op for modern lwIP)
// --------------------------------------------------
void mqtt_app_poll(void) {
    // Nothing needed — background threadsafe mode handles this
}
