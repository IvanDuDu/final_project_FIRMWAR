#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief  Initialise NimBLE stack, register GATT service with two
 *         characteristics (RX write / TX notify), and start advertising.
 *         Advertising restarts automatically after every disconnection.
 */
esp_err_t ble_manager_init(void);

/**
 * @brief  Send a JSON string to the connected central device via notify.
 * @param  json   NULL-terminated JSON string.
 * @return ESP_OK if notify was queued, ESP_FAIL if no connection exists.
 */
esp_err_t ble_manager_notify(const char *json);

/**
 * @brief  Returns true when a central is currently connected.
 */
bool ble_manager_is_connected(void);