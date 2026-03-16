


// mqtt_client_wrap.h
#pragma once
#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief  Initialise and connect the ESP MQTT client to the broker.
 *         Called by server_connect after network layer is ready.
 */
esp_err_t mqtt_client_start(void);

/** Stop and destroy MQTT client. */
void mqtt_client_stop(void);

/**
 * @brief  Publish a message on a topic.
 * @param  topic    MQTT topic string.
 * @param  payload  Null-terminated JSON string.
 * @param  qos      0 or 1.
 * @param  retain   0 or 1.
 * @return ESP_OK on success.
 */
esp_err_t mqtt_publish(const char *topic, const char *payload, int qos, int retain);

/** Returns true when MQTT client is connected to broker. */
bool mqtt_is_connected(void);