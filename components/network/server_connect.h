// server_connect.h
#pragma once
#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief  Connect to the server using the appropriate transport.
 *         Reads the latest routeHistory entry to decide WiFi vs SIM.
 *         Then starts MQTT client.
 */
esp_err_t server_connect(void);

/**
 * @brief  Re-evaluate transport mode after a new shipper CMD01.
 *         If isOnBoard changed, switch transport and reconnect MQTT.
 */
esp_err_t server_connect_update_transport(bool is_on_board);

/** Disconnect MQTT and underlying network interface. */
void server_disconnect(void);