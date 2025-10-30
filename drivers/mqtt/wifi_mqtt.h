// ===============================================
//  Module: Wi-Fi and MQTT Client
//  Description: Connects to hotspot, sends telemetry via MQTT.
// ===============================================
#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"

void wifi_connect_init(void);
bool wifi_is_connected(void);

void mqtt_init(void);
bool mqtt_is_connected(void);
void wifi_mqtt_publish(const char *topic, const char *msg);
void mqtt_loop(void);

#endif
