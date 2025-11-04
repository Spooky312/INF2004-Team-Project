#ifndef MQTT_CLIENT_APP_H
#define MQTT_CLIENT_APP_H

#include <stdbool.h>

bool mqtt_app_init_and_connect(const char *broker_ip, int port, const char *client_id);
bool mqtt_app_is_connected(void);
void mqtt_app_publish(const char *topic, const char *payload, int qos, int retain);
void mqtt_app_poll(void);

#endif
