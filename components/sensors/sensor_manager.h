#pragma once

#include "esp_err.h"
#include "globals.h"

/**
 * @brief  Initialise all sensor hardware (calls each sensor's init).
 *         Also configures GPIO interrupts for door sensor and MPU6050.
 */
esp_err_t sensor_manager_init(void);

/**
 * @brief  Read all sensors sequentially on I2C Bus 1 then Bus 2,
 *         populate a telemetry_record_t and return it.
 *
 *         Sequence: DS3231 → SHT30 → GY-906 → GPS → MPU6050
 *
 * @param  out  Pointer to record that will be filled.
 * @return ESP_OK on full success; ESP_FAIL if any critical sensor failed.
 */
esp_err_t sensor_manager_read_all(telemetry_record_t *out);

/**
 * @brief  FreeRTOS task: reads all sensors every SENSOR_READ_INTERVAL_MS,
 *         writes to g_latest_telemetry, and pushes to g_send_telemetry[].
 *         Spawned by main after sensor_manager_init().
 */
void task_sensor_read(void *arg);