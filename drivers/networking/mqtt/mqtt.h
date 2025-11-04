// ===============================================
//  Module: MQTT Client
//  Description: Thread-safe MQTT client with publish capabilities
// ===============================================

#ifndef MQTT_H
#define MQTT_H

#include <stdbool.h>
#include <stdint.h>

// Initialize MQTT client and connect to broker
void mqtt_app_init(void);

// Check if MQTT is connected
bool mqtt_app_is_connected(void);

// Publish message to topic (thread-safe)
// qos: 0, 1, or 2
// retain: 0 or 1
bool mqtt_app_publish(const char *topic, const char *message, uint8_t qos, uint8_t retain);

// Disconnect MQTT
void mqtt_app_disconnect(void);

#endif // MQTT_H
