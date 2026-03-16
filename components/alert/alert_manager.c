#include "alert_manager.h"
#include "globals.h"
#include "mqtt_client_wrap.h"

#include "esp_log.h"
#include "cJSON.h"
#include "sensors.h"     // for ds3231_get_timestamp

#include <string.h>

static const char *TAG = "ALERT";

// ─────────────────────────────────────────────
//  Helper: build and publish a warning packet
//  payload: { "timestamp": "..." }
// ─────────────────────────────────────────────
static void publish_warning(const char *container_id,
                             const char *warning_type)
{
    char timestamp[32];
    ds3231_get_timestamp(timestamp, sizeof(timestamp));

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "timestamp", timestamp);
    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return;

    char topic[96];
    snprintf(topic, sizeof(topic), "device/%s/warning/%s",
             container_id, warning_type);
    
    esp_err_t ret = mqtt_publish(topic, payload, 1, 0);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "WARNING published: %s @ %s", warning_type, timestamp);
    } else {
        ESP_LOGE(TAG, "WARNING publish failed: %s", warning_type);
    }
    free(payload);
}

// ─────────────────────────────────────────────
//  Alert handler task
// ─────────────────────────────────────────────
void task_alert_handler(void *arg)
{
    ESP_LOGI(TAG, "Alert handler task started");

    char container_id[MAX_CONTAINER_ID_LEN] = {0};

    for (;;) {
        // Refresh container_id each iteration (set during Stage 1)
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        strlcpy(container_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
        bool door_en = g_door_enable;
        xSemaphoreGive(g_mutex_globals);

        // ── Door open alert ───────────────────
        if (xSemaphoreTake(g_sem_door_alert, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (door_en) {
                if (container_id[0] != '\0') {
                    publish_warning(container_id, "doorOpen");
                } else {
                    ESP_LOGW(TAG, "Door alert fired but containerID not set yet");
                }
            } else {
                ESP_LOGD(TAG, "Door alert suppressed (doorEnable=false)");
            }
        }

        // ── Collision alert ───────────────────
        if (xSemaphoreTake(g_sem_collision, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (container_id[0] != '\0') {
                publish_warning(container_id, "collision");
            } else {
                ESP_LOGW(TAG, "Collision alert fired but containerID not set yet");
            }
        }

        // Yield to other tasks briefly before next poll
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}