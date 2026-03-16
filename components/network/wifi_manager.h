// wifi_manager.h
#pragma once
#include "esp_err.h"
#include <stdbool.h>

/** Read ssid/pass from NVS and connect to WiFi. Blocks until connected or timeout. */
esp_err_t wifi_manager_connect(void);

/** Disconnect and clean up WiFi. */
void wifi_manager_disconnect(void);

/** Returns true if currently connected to WiFi AP. */
bool wifi_manager_is_connected(void);