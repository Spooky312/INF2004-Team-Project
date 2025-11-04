// ===============================================
//  Module: Network Manager
//  Description: Thread-safe network task that manages
//               WiFi connection and MQTT client lifecycle
// ===============================================

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "pico/stdlib.h"

// Network manager FreeRTOS task
void network_manager_task(void *params);

// Status queries
bool network_manager_is_wifi_connected(void);
bool network_manager_is_mqtt_connected(void);

#endif // NETWORK_MANAGER_H
