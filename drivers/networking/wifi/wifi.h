#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

// Initializes Wi-Fi hardware and connects to the configured network.
// Returns true on success, false on failure.
bool wifi_init_and_connect(const char *ssid, const char *password);

// Checks if Wi-Fi is currently connected.
bool wifi_is_connected(void);

// Gracefully shuts down the Wi-Fi stack.
void wifi_deinit(void);

#endif
