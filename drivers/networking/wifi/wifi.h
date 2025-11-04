// ===============================================
//  Module: WiFi Client
//  Description: Thread-safe WiFi connection management
// ===============================================

#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

// Initialize WiFi and connect to access point
void wifi_app_init(void);

// Check if WiFi is connected
bool wifi_app_is_connected(void);

// Disconnect WiFi
void wifi_app_disconnect(void);

#endif // WIFI_H
