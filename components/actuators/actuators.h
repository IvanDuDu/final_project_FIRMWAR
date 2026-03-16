#pragma once

#include "esp_err.h"
#include <stdbool.h>

// ─────────────────────────────────────────────
//  Initialisation
// ─────────────────────────────────────────────

/**
 * @brief  Configure LEDC timer & channel for fan PWM (GPIO 6),
 *         and GPIO output for UV LED (GPIO 7).
 */
esp_err_t actuators_init(void);

// ─────────────────────────────────────────────
//  Fan  (5 fans in series, single PWM line)
// ─────────────────────────────────────────────

/**
 * @brief  Set fan PWM duty cycle (0–1023, 10-bit).
 *         Internally checks g_fan_enable; returns immediately if disabled.
 */
void fan_set_duty(uint32_t duty);

/**
 * @brief  Automatic fan control logic, called after every sensor read.
 *
 *  Rules (in priority order):
 *  1. If g_fan_enable == false → stop fan and return.
 *  2. delta = sur_temp − in_temp
 *     a. delta <= 2  AND  humidity <= humidThreshold → stop fan.
 *     b. delta >  2  → PWM = (delta / 5) * FAN_PWM_MAX, capped at MAX.
 *  3. If delta <= 2 AND humidity > humidThreshold → run at 80%.
 *  4. If fan is running (fanEnable true) but this reading's humidity
 *     is still higher than the previous reading → set fanEnable = false
 *     (fan is not helping — operator must re-enable via BLE CMD 03).
 */
void fan_auto_control(float sur_temp, float in_temp,
                      float humidity,  float humid_threshold);

// ─────────────────────────────────────────────
//  UV LED
// ─────────────────────────────────────────────

/**
 * @brief  Turn UV LED on or off.  Mirrors g_uv_enable.
 */
void uv_set(bool on);