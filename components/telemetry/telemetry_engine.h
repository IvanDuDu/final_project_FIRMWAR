#pragma once
#include "esp_err.h"
#include "globals.h"

/**
 * @brief  Calculate cargo loss based on current humidity reading.
 *         Formula: Q = Q0 * (humidity - humidThreshold) / 100 * (weight / 20)
 *         Returns 0 if humidity <= humidThreshold.
 *
 * @param  Q0          Current remaining quality (pass g_config.weight on first call,
 *                     or carry forward between readings).
 * @param  humidity    Current humidity reading (%).
 * @param  threshold   humidThreshold from container config.
 * @param  weight      Container weight (kg).
 */
float telemetry_calc_loss(float Q0, float humidity,
                           float threshold, float weight);

/**
 * @brief  Serialise a telemetry_record_t to the JSON format required by spec.
 * @param  rec    Record to serialise.
 * @param  buf    Output buffer.
 * @param  len    Buffer length.
 */
esp_err_t telemetry_serialize(const telemetry_record_t *rec, char *buf, size_t len);

/**
 * @brief  FreeRTOS task: monitors g_telem_count.
 *         When count is a multiple of TELEMETRY_BATCH_SIZE (4), publishes
 *         all buffered records to device/{containerId}/telemetry, then
 *         clears the buffer and resets count.
 *         If publish fails, keeps buffer and retries at the next multiple.
 */
void task_telemetry_send(void *arg);