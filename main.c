#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wifi_mqtt.h"

// =====================================================================
// Configuration
// =====================================================================
#define WIFI_TASK_STACK_SIZE   (4096)
#define WIFI_TASK_PRIORITY     (tskIDLE_PRIORITY + 3)

#define PING_TASK_PERIOD_MS    3000
#define PING_TASK_STACK_SIZE   (configMINIMAL_STACK_SIZE * 2)
#define PING_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)

// =====================================================================
// MQTT Ping Task
// =====================================================================
static void mqtt_ping_task(__unused void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    int ping_counter = 0;

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PING_TASK_PERIOD_MS));

        if (!mqtt_is_connected()) {
            printf("[PING] MQTT not connected — skipping.\n");
            continue;
        }

        char msg[128];
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"ping\",\"count\":%d,\"uptime_ms\":%lu}",
                 ping_counter++, to_ms_since_boot(get_absolute_time()));

        wifi_mqtt_publish("pico/telemetry", msg);
        printf("[PING] Published: %s\n", msg);

        mqtt_loop();  // keep lwIP + MQTT alive
    }
}

// =====================================================================
// Wi-Fi Init Task
// =====================================================================
static void wifi_task(__unused void *params) {
    printf("Starting Wi-Fi task...\n");

    // 🔹 Turn LED on while initializing Wi-Fi
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("Init Wi-Fi driver...\n");

    int ret = cyw43_arch_init();

    // 🔹 Turn LED off when done
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    printf("Init returned %d\n", ret);

    if (ret) {
        printf("❌ cyw43 init failed (%d)\n", ret);
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi: %s...\n", WIFI_SSID);

    int result = cyw43_arch_wifi_connect_async(
        WIFI_SSID,
        WIFI_PASS,
        CYW43_AUTH_WPA2_AES_PSK
    );
    if (result != 0) {
        printf("❌ Connection start failed: %d\n", result);
        vTaskDelete(NULL);
    }

    // Wait for Wi-Fi link
    while (true) {
        int status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
        if (status == CYW43_LINK_UP) {
            printf("✅ Connected to Wi-Fi!\n");
            break;
        } else if (status == CYW43_LINK_FAIL) {
            printf("❌ Connection failed\n");
            vTaskDelete(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // After Wi-Fi connects, start MQTT
    mqtt_init();
    printf("✅ MQTT init done\n");

    // Start background ping publisher
    xTaskCreate(mqtt_ping_task, "MQTT_Ping",
                PING_TASK_STACK_SIZE, NULL,
                PING_TASK_PRIORITY, NULL);

    vTaskDelete(NULL);  // Wi-Fi setup done
}

// =====================================================================
// Main Entry
// =====================================================================
int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("========================================\n");
    printf("  Pico W FreeRTOS Wi-Fi + MQTT Demo\n");
    printf("========================================\n");

    // Create the Wi-Fi init task (handles Wi-Fi + MQTT)
    xTaskCreate(wifi_task, "WiFi_Task",
                WIFI_TASK_STACK_SIZE, NULL,
                WIFI_TASK_PRIORITY, NULL);

    // Start the FreeRTOS scheduler
    printf("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();

    while (true); // Should never reach here
}

// =====================================================================
// FreeRTOS Hooks
// =====================================================================
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("❌ Stack overflow in %s\n", pcTaskName);
    while (1);
}

void vApplicationMallocFailedHook(void) {
    printf("❌ FreeRTOS malloc failed!\n");
    while (1);
}

void vApplicationIdleHook(void) {
    __asm volatile("nop");
}
