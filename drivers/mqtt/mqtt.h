// mqtt.h
#ifndef MQTT_H
#define MQTT_H
#include <stdbool.h>

bool mqtt_setup(const char *ssid, const char *pass,
                const char *broker_ip, const char *topic);
void mqtt_publish_message(const char *topic, const char *msg);
void mqtt_task(void *params);

#endif
