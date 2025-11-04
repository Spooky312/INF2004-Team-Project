#include "networkManager.h"
#include "wifi.h"
#include "mqtt.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

#define MQTT_BROKER_IP  MQTT_SERVER_IP
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "PicoClient"

void network_manager_task(void *pvParameters) {
    printf("\n=== Network Manager Task ===\n");

    if (!wifi_init_and_connect(WIFI_SSID, WIFI_PASSWORD)) {
        printf("[NET] Wi-Fi failed to connect.\n");
        vTaskDelete(NULL);
    }

    if (!mqtt_app_init_and_connect(MQTT_BROKER_IP, MQTT_PORT, MQTT_CLIENT_ID)) {
        printf("[NET] MQTT failed to connect.\n");
        vTaskDelete(NULL);
    }

    for (;;) {
        mqtt_app_poll();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
