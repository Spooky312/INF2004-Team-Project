// ===============================================
//  Module: WiFi Client
//  Description: Manages WiFi connection using CYW43 driver
//               with background lwIP threading
// ===============================================

#include "wifi.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

// WiFi credentials - these should match your CMakeLists.txt definitions
#ifndef WIFI_SSID
#define WIFI_SSID "Yh"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "tffycmzith"
#endif

static bool wifi_connected = false;

void wifi_app_init(void)
{
    printf("[WIFI] Initializing CYW43 driver with background threading...\n");
    
    // Initialize CYW43 with lwIP background support
    if (cyw43_arch_init()) {
        printf("[WIFI] ❌ Failed to initialize CYW43 driver\n");
        return;
    }
    
    printf("[WIFI] CYW43 driver initialized\n");
    
    // Enable station mode
    cyw43_arch_enable_sta_mode();
    printf("[WIFI] Station mode enabled\n");
    
    // Connect to WiFi
    printf("[WIFI] Connecting to '%s'...\n", WIFI_SSID);
    
    int result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, 
        WIFI_PASSWORD, 
        CYW43_AUTH_WPA2_AES_PSK,
        30000  // 30 second timeout
    );
    
    if (result == 0) {
        wifi_connected = true;
        printf("[WIFI] ✅ Connected successfully\n");
        
        // Print IP address
        extern cyw43_t cyw43_state;
        uint32_t ip = cyw43_state.netif[0].ip_addr.addr;
        printf("[WIFI] IP Address: %d.%d.%d.%d\n",
               ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF);
    } else {
        wifi_connected = false;
        printf("[WIFI] ❌ Connection failed (error %d)\n", result);
    }
}

bool wifi_app_is_connected(void)
{
    // Check both our flag and the actual link status
    if (wifi_connected) {
        extern cyw43_t cyw43_state;
        int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
        
        if (link_status == CYW43_LINK_UP) {
            return true;
        } else {
            wifi_connected = false;
            printf("[WIFI] Link down detected\n");
        }
    }
    return false;
}

void wifi_app_disconnect(void)
{
    printf("[WIFI] Disconnecting...\n");
    wifi_connected = false;
    cyw43_arch_deinit();
}
