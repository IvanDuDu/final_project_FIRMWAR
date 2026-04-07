#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────
//  ESP-NOW Pathfinding Manager
//
//  Implements a flood-based path-discovery across containers on the same
//  vessel. Each container runs this module. When a Shipper wants to locate
//  a container, they send BLE CMD 05 to the nearest container. That
//  container floods a PATHFIND_REQ via ESP-NOW broadcast; every intermediate
//  node re-broadcasts (TTL-decremented). When the target container receives
//  the flood it sends a unicast PATHFIND_ACK back along the reverse path,
//  causing every node on the path to light its PATH LED (GPIO 9). The
//  destination container lights its DEST LED (GPIO 8). All LEDs auto-off
//  after PATH_LED_DURATION_MS (5 minutes).
//
//  Protocol packet layout (serialised as compact JSON over ESP-NOW raw data):
//    PATHFIND_REQ  { "t":0, "id":"<msgId>", "dst":"<containerId>",
//                    "src":"<MAC>", "ttl":8, "hop":0,
//                    "path":["MAC1","MAC2",...] }
//    PATHFIND_ACK  { "t":1, "id":"<msgId>", "dst":"<containerId>",
//                    "path":["MAC1","MAC2",...], "found":true }
//
//  Deduplication: each node keeps a circular buffer of recently-seen msg_ids
//  and silently drops duplicates (TTL flood suppression).
// ─────────────────────────────────────────────

/**
 * @brief  Initialise ESP-NOW subsystem and register receive callback.
 *         Must be called AFTER esp_wifi_start() (WiFi must be up for
 *         ESP-NOW to use the radio, even when WiFi itself is not needed).
 *         If WiFi is not yet connected, call with wifi_init_only=true to
 *         start WiFi in STA mode without associating to an AP.
 */
esp_err_t espnow_manager_init(void);

/**
 * @brief  Initiate a pathfind flood originating from THIS container.
 *         Called by BLE CMD 05 handler.
 *
 * @param  target_container_id  NULL-terminated container ID string to find.
 */
void espnow_pathfind_start(const char *target_container_id);

/**
 * @brief  FreeRTOS task: drives the LED auto-off timer for PATH and DEST LEDs.
 *         Spawn once after espnow_manager_init().
 *         Stack: 2048 bytes, priority 2.
 */
void task_led_timer(void *arg);