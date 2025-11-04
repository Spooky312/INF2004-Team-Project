// ===============================================
//  Module: Network Manager
//  Description: Manages WiFi and MQTT lifecycle in FreeRTOS task
//               Uses polling for CYW43 (NO_SYS mode)
// ===============================================

#include "networkManager.h"
#include "wifi/wifi.h"
#include "mqtt/mqtt.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

// Task loop for network management
void network_manager_task(void *params)
{
    (void)params;
    
    printf("[NET] Network Manager task starting...\n");
    
    // Initialize WiFi
    printf("[NET] Initializing WiFi...\n");
    wifi_app_init();
    
    // Wait for WiFi connection
    printf("[NET] Waiting for WiFi connection...\n");
    while (!wifi_app_is_connected()) {
        // Poll CYW43 driver for WiFi events
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(500));
        printf("[NET] WiFi connecting...\n");
    }
    printf("[NET] ✅ WiFi connected!\n");
    
    // Initialize MQTT
    printf("[NET] Initializing MQTT client...\n");
    mqtt_app_init();
    
    // Wait for MQTT connection
    printf("[NET] Waiting for MQTT connection...\n");
    int mqtt_retry = 0;
    while (!mqtt_app_is_connected()) {
        // Poll CYW43 for network events
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(1000));
        mqtt_retry++;
        printf("[NET] MQTT connecting... (attempt %d)\n", mqtt_retry);
        
        if (mqtt_retry > 30) {
            printf("[NET] ⚠️ MQTT connection timeout - will retry\n");
            mqtt_retry = 0;
        }
    }
    printf("[NET] ✅ MQTT connected!\n");
    
    // Main loop - monitor connections and poll CYW43
    for (;;)
    {
        // IMPORTANT: Poll CYW43 driver to process WiFi/network events
        cyw43_arch_poll();
        
        // Check WiFi status
        if (!wifi_app_is_connected()) {
            printf("[NET] ⚠️ WiFi disconnected - reconnecting...\n");
            while (!wifi_app_is_connected()) {
                cyw43_arch_poll();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            printf("[NET] ✅ WiFi reconnected\n");
        }
        
        // Check MQTT status
        if (!mqtt_app_is_connected()) {
            printf("[NET] ⚠️ MQTT disconnected - reconnecting...\n");
            mqtt_app_init();  // Reinitialize MQTT
            while (!mqtt_app_is_connected()) {
                cyw43_arch_poll();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            printf("[NET] ✅ MQTT reconnected\n");
        }
        
        // Poll frequently for network events (100ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Status query functions
bool network_manager_is_wifi_connected(void)
{
    return wifi_app_is_connected();
}

bool network_manager_is_mqtt_connected(void)
{
    return mqtt_app_is_connected();
}
