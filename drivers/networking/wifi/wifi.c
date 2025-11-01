#include "wifi.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

// ---------------------------------------------------------
// Wi-Fi Connection (Threadsafe Background)
// ---------------------------------------------------------

bool wifi_init_and_connect(const char *ssid, const char *password) {
    printf("\n[WiFi] Initializing CYW43...\n");

    if (cyw43_arch_init()) {
        printf("[WiFi] ❌ cyw43_arch_init() failed! Check power or SDK.\n");
        return false;
    }

    cyw43_arch_enable_sta_mode();

    printf("[WiFi] Connecting to SSID: %s ...\n", ssid);
    int rc = cyw43_arch_wifi_connect_timeout_ms(
        ssid, password, CYW43_AUTH_WPA2_AES_PSK, 30000);

    if (rc) {
        printf("[WiFi] ❌ Connection failed (code=%d). Check credentials or signal.\n", rc);
        cyw43_arch_deinit();
        return false;
    }

    extern struct netif *netif_default;
    if (netif_default) {
        printf("[WiFi] ✅ Connected! IP: %s\n",
               ip4addr_ntoa(netif_ip4_addr(netif_default)));
    } else {
        printf("[WiFi] ⚠️ Connected but no IP assigned.\n");
    }

    return true;
}

bool wifi_is_connected(void) {
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

void wifi_deinit(void) {
    printf("[WiFi] Deinitializing Wi-Fi stack...\n");
    cyw43_arch_deinit();
}
