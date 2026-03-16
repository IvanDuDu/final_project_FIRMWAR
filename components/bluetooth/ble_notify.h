#pragma once

/**
 * @brief  Called immediately after a central connects.
 *         Serialises the current device state and sends it via BLE notify.
 */
void ble_notify_on_connect(void);