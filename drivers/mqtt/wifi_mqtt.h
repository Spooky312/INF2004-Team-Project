#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Wi-Fi credentials from "Project 2.c"
#define WIFI_SSID "Yh"
#define WIFI_PASS "tffycmzith"

// MQTT Broker from "Project 2.c"
#define MQTT_SERVER_IP "192.168.23.8"
#define MQTT_PORT 1883
#define MQTT_TOPIC_TELEMETRY "pico/telemetry" // For publishing data [cite: 364]
#define MQTT_TOPIC_COMMANDS  "pico/commands"  // For receiving commands


int wifi_init(void);
void mqtt_connect(void);
void mqtt_publish_telemetry(const char *msg);
void mqtt_check_reconnect(void);
bool mqtt_is_connected(void);


#endif // WIFI_MQTT_H