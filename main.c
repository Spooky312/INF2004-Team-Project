#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "mqtt.h"

// ---- Pin & constants ----
#define TRIG_PIN   17
#define ECHO_PIN   16
#define SERVO_PIN  15
#define SAFE_DIST  15.0f
#define MQTT_TOPIC "car/telemetry"

static float left_scan_avg = 0.0f, right_scan_avg = 0.0f;
static bool  obstacle_active = false;

// ---------------------------------------------------------
// Helper: scan servo from start_angle→end_angle (step°)
static float servo_scan(uint trig, uint echo,
                        uint servo, float start_angle,
                        float end_angle, float step_deg) {
    float sum = 0.0f; int count = 0;
    float dir = (end_angle > start_angle) ? step_deg : -step_deg;
    for (float a = start_angle; (dir > 0) ? a <= end_angle : a >= end_angle; a += dir) {
        servo_set_angle(servo, a);
        sleep_ms(80);
        float d = ultrasonic_get_distance_cm(trig, echo);
        if (d > 0) { sum += d; count++; }
    }
    return (count > 0) ? (sum / count) : 0;
}

// ---------------------------------------------------------
// Task: Ultrasonic + Servo scanning + obstacle decision
static void task_ultrasonic(void *p) {
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    servo_init(SERVO_PIN);
    servo_set_angle(SERVO_PIN, 90);

    while (1) {
        float d = ultrasonic_get_distance_cm(TRIG_PIN, ECHO_PIN);

        if (d > 0 && d < SAFE_DIST && !obstacle_active) {
            obstacle_active = true;
            motor_stop();
            printf("⚠️ Obstacle %.2f cm ahead — scanning...\n", d);

            // Sweep left and right
            left_scan_avg  = servo_scan(TRIG_PIN, ECHO_PIN, SERVO_PIN, 90, 45, 15);
            right_scan_avg = servo_scan(TRIG_PIN, ECHO_PIN, SERVO_PIN, 90, 135, 15);
            servo_set_angle(SERVO_PIN, 90);

            // Decide direction
            const char *dir = (right_scan_avg > left_scan_avg) ? "RIGHT" : "LEFT";
            printf("Left = %.2f cm, Right = %.2f cm → %s path\n",
                   left_scan_avg, right_scan_avg, dir);

            char msg[128];
            snprintf(msg, sizeof msg,
                     "{\"event\":\"obstacle\",\"front\":%.2f,"
                     "\"left\":%.2f,\"right\":%.2f,\"decision\":\"%s\"}",
                     d, left_scan_avg, right_scan_avg, dir);
            mqtt_publish_message(MQTT_TOPIC, msg);

            // Turn accordingly
            if (right_scan_avg > left_scan_avg)
                motor_set_speed(0.5f, -0.5f);   // pivot right
            else
                motor_set_speed(-0.5f, 0.5f);   // pivot left
            vTaskDelay(pdMS_TO_TICKS(600));

            // Move forward a bit to bypass obstacle
            motor_set_speed(0.5f, 0.5f);
            vTaskDelay(pdMS_TO_TICKS(1200));
            motor_stop();

            obstacle_active = false;
            printf("✅ Rejoined line / path\n");
            mqtt_publish_message(MQTT_TOPIC, "{\"event\":\"resume\"}");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ---------------------------------------------------------
// Task: Normal forward motion
static void task_motor(void *p) {
    motor_init();
    float base = 0.5f;
    while (1) {
        if (!obstacle_active)
            motor_set_speed(base, base);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------
// Task: Maintain MQTT connection & network polling
static void task_mqtt(void *p) {
    mqtt_setup("Yh","tffycmzith","192.168.23.8",MQTT_TOPIC);
    mqtt_task(NULL);
}

// ---------------------------------------------------------
int main() {
    stdio_init_all();
    xTaskCreate(task_mqtt, "mqtt", 4096, NULL, 1, NULL);
    xTaskCreate(task_ultrasonic, "ultra", 4096, NULL, 2, NULL);
    xTaskCreate(task_motor, "motor", 2048, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1);
}
