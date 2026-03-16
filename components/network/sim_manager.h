// sim_manager.h
#pragma once
#include "esp_err.h"
#include <stdbool.h>

/** Initialise UART2, run AT handshake, attach to 5G network. */
esp_err_t sim_manager_connect(void);

/** Disconnect data bearer. */
void sim_manager_disconnect(void);

/** Returns true if data connection is active. */
bool sim_manager_is_connected(void);