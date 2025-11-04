// ===============================================
//  Module: WiFi Client
//  Description: Manages WiFi connection using CYW43 driver
//               with background lwIP threading
// ===============================================

#include "wifi.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>

// WiFi credentials - these should match your CMakeLists.txt definitions
#ifndef WIFI_SSID
#define WIFI_SSID "Yh"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "tffycmzith"
#endif

static bool wifi_connected = false;

// Helper function to decode WiFi error codes
static const char* wifi_error_string(int error)
{
    switch(error) {
        case 0: return "Success";
        case -1: return "Generic error";
        case -2: return "Timeout";
        case -3: return "Authentication failed";
        case -8: return "Link failed (wrong password or SSID not found)";
        default: return "Unknown error";
    }
}

// Helper function to decode auth type
static const char* auth_type_string(uint32_t auth)
{
    switch(auth) {
        case CYW43_AUTH_OPEN: return "OPEN";
        case CYW43_AUTH_WPA_TKIP_PSK: return "WPA-TKIP";
        case CYW43_AUTH_WPA2_AES_PSK: return "WPA2-AES";
        case CYW43_AUTH_WPA2_MIXED_PSK: return "WPA2-MIXED";
        default: return "UNKNOWN";
    }
}

void wifi_app_init(void)
{
    printf("[WIFI] ========== WiFi Initialization ==========\n");
    printf("[WIFI] Target SSID: '%s'\n", WIFI_SSID);
    printf("[WIFI] Password length: %d chars\n", strlen(WIFI_PASSWORD));
    printf("[WIFI] Auth mode: %s\n", auth_type_string(CYW43_AUTH_WPA2_AES_PSK));
    
    printf("[WIFI] Initializing CYW43 driver with background threading...\n");
    
    // Initialize CYW43 with lwIP background support
    if (cyw43_arch_init()) {
        printf("[WIFI] ❌ Failed to initialize CYW43 driver\n");
        return;
    }
    
    printf("[WIFI] ✅ CYW43 driver initialized\n");
    
    // Enable station mode
    cyw43_arch_enable_sta_mode();
    printf("[WIFI] ✅ Station mode enabled\n");
    
    // Print MAC address
    extern cyw43_t cyw43_state;
    printf("[WIFI] MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
           cyw43_state.mac[0], cyw43_state.mac[1], cyw43_state.mac[2],
           cyw43_state.mac[3], cyw43_state.mac[4], cyw43_state.mac[5]);
    
    // Scan for networks (quick scan)
    printf("[WIFI] Scanning for networks...\n");
    // Note: Full scan implementation would require scan callbacks
    // For now, we'll proceed with connection attempt
    
    // Connect to WiFi
    printf("[WIFI] Attempting connection to '%s'...\n", WIFI_SSID);
    printf("[WIFI] Connection timeout: 30 seconds\n");
    
    int result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, 
        WIFI_PASSWORD, 
        CYW43_AUTH_WPA2_AES_PSK,
        30000  // 30 second timeout
    );
    
    if (result == 0) {
        wifi_connected = true;
        printf("[WIFI] ✅✅✅ Connected successfully! ✅✅✅\n");
        
        // Give connection time to stabilize
        sleep_ms(100);
        
        // Print IP address
        uint32_t ip = cyw43_state.netif[0].ip_addr.addr;
        printf("[WIFI] IP Address: %d.%d.%d.%d\n",
               ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF);
        
        // Print netmask and gateway
        uint32_t netmask = cyw43_state.netif[0].netmask.addr;
        uint32_t gw = cyw43_state.netif[0].gw.addr;
        printf("[WIFI] Netmask: %d.%d.%d.%d\n",
               netmask & 0xFF, (netmask >> 8) & 0xFF, (netmask >> 16) & 0xFF, (netmask >> 24) & 0xFF);
        printf("[WIFI] Gateway: %d.%d.%d.%d\n",
               gw & 0xFF, (gw >> 8) & 0xFF, (gw >> 16) & 0xFF, (gw >> 24) & 0xFF);
        
        // Check link status using tcpip helper (correct for background threading)
        int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        printf("[WIFI] Link status: %d (0=DOWN, 1=JOIN, 2=NOIP, 3=UP)\n", link_status);
        
    } else {
        wifi_connected = false;
        printf("[WIFI] ❌❌❌ Connection FAILED ❌❌❌\n");
        printf("[WIFI] Error code: %d\n", result);
        printf("[WIFI] Error description: %s\n", wifi_error_string(result));
        printf("[WIFI] \n");
        printf("[WIFI] Troubleshooting tips:\n");
        printf("[WIFI]   • Error -8: Usually wrong password or SSID not found\n");
        printf("[WIFI]   • Check SSID is spelled correctly (case-sensitive)\n");
        printf("[WIFI]   • Verify password is correct\n");
        printf("[WIFI]   • Ensure network is 2.4GHz (Pico W doesn't support 5GHz)\n");
        printf("[WIFI]   • Try moving Pico W closer to router\n");
        printf("[WIFI]   • Check router is broadcasting SSID (not hidden)\n");
        
        // Check if we can get link status even on failure
        int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        printf("[WIFI] Link status after failure: %d (0=DOWN, 1=JOIN, 2=NOIP, 3=UP)\n", link_status);
    }
    
    printf("[WIFI] ==========================================\n");
}

bool wifi_app_is_connected(void)
{
    extern cyw43_t cyw43_state;
    
    // For pico_cyw43_arch_lwip_threadsafe_background, use the arch helper
    // This properly checks link status with the background lwIP thread
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    
    // CYW43_LINK_UP = 3, but we should also accept any positive link status
    // Link status values: 0=DOWN, 1=JOIN, 2=NOIP, 3=UP
    bool is_up = (status == CYW43_LINK_UP);
    
    if (wifi_connected && !is_up) {
        wifi_connected = false;
        printf("[WIFI] ⚠️ Link status changed: %d (was connected, now down)\n", status);
    }
    
    return is_up;
}

void wifi_app_disconnect(void)
{
    printf("[WIFI] Disconnecting...\n");
    wifi_connected = false;
    cyw43_arch_deinit();
}
