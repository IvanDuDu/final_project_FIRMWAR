#pragma once

/**
 * @brief  Parse and dispatch an incoming BLE JSON packet.
 *         Called from the GATT write callback (BLE task context).
 *         Format: { "command": "00", "role": "...", "des": { ... } }
 */
void ble_handler_process(const char *json_str);