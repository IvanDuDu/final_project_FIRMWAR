#pragma once

#include "esp_err.h"
#include <stdbool.h>

// ─────────────────────────────────────────────
//  UDP LAN Remote Control
//
//  Allows the shipper mobile app (on the same vessel WiFi LAN) to control
//  the fan (CMD 03) and UV LED (CMD 04) without requiring a BLE connection.
//
//  Discovery flow:
//    1. Mobile app broadcasts a UDP DISCOVER packet on UDP_DISC_PORT (5556).
//       Payload: { "type":"discover", "mac":"AA:BB:CC:DD:EE:FF" }
//       The "mac" field contains the target device's known MAC address.
//
//    2. Every ESP32 on the LAN that receives the DISCOVER packet compares
//       the MAC with its own. If it matches, it replies with:
//       { "type":"offer", "mac":"<own>", "ip":"<own IP>" }
//       sent to the sender's IP:UDP_DISC_PORT.
//
//    3. Mobile app extracts the device IP from the "offer" reply, then sends
//       control commands as unicast UDP to UDP_CTRL_PORT (5555):
//       { "command":"03", "des":{ "value":1 } }   — fan on
//       { "command":"04", "des":{ "value":0 } }   — UV off
//
//    4. The device reuses the existing BLE command dispatcher
//       (ble_handler_process) to execute the command, ensuring parity.
//
//  Security note:
//    - Only CMD 03 and CMD 04 are accepted over UDP. All other commands
//      are silently rejected (BLE channel required for setup/handover).
//    - AP isolation: if the ship router enables AP isolation, UDP broadcast
//      will not traverse. The system degrades gracefully — BLE remains
//      available as fallback.
// ─────────────────────────────────────────────

/**
 * @brief  Start the UDP LAN control service.
 *         Opens two sockets: CTRL (5555) and DISC (5556).
 *         Must be called after WiFi is connected (IP available).
 *         Internally spawns a FreeRTOS task.
 *
 * @return ESP_OK on success.
 */
esp_err_t udp_control_start(void);

/**
 * @brief  Stop and clean up the UDP control service (sockets + task).
 *         Call before switching away from WiFi to SIM.
 */
void udp_control_stop(void);

/**
 * @brief  Returns true if the UDP service is currently running.
 */
bool udp_control_is_running(void);