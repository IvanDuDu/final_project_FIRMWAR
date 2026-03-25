#include "alert_manager.h"
#include "globals.h"
#include "mqtt_client_wrap.h"
#include "sd_manager.h"

#include "esp_log.h"
#include "cJSON.h"
#include "sensors.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "ALERT";

// ─────────────────────────────────────────────
//  Helper: build JSON line cho 1 alert
//  Format lưu vào SD: { "topic": "...", "payload": "{...}" }
//  → giữ nguyên topic để khi flush biết gửi đi đâu
// ─────────────────────────────────────────────
static char *build_alert_line(const char *container_id, const char *warning_type)
{
    char timestamp[32];
    ds3231_get_timestamp(timestamp, sizeof(timestamp));

    // Tạo payload JSON
    cJSON *payload_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(payload_obj, "timestamp", timestamp);
    char *payload_str = cJSON_PrintUnformatted(payload_obj);
    cJSON_Delete(payload_obj);
    if (!payload_str) return NULL;

    // Tạo topic string
    char topic[96];
    snprintf(topic, sizeof(topic), "device/%s/warning/%s",
             container_id, warning_type);

    // Đóng gói thành 1 dòng: { "topic": "...", "payload": "..." }
    cJSON *line_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(line_obj, "topic",   topic);
    cJSON_AddStringToObject(line_obj, "payload", payload_str);
    free(payload_str);

    char *line_str = cJSON_PrintUnformatted(line_obj);
    cJSON_Delete(line_obj);
    return line_str;  // caller phải free()
}

// ─────────────────────────────────────────────
//  Callback cho sd_alert_flush()
//  Gửi từng alert qua MQTT; trả về ESP_OK nếu thành công
// ─────────────────────────────────────────────
static esp_err_t flush_alert_cb(const char *json_line, void *ctx)
{
    (void)ctx;

    cJSON *root = cJSON_Parse(json_line);
    if (!root) return ESP_FAIL;

    cJSON *topic_item   = cJSON_GetObjectItem(root, "topic");
    cJSON *payload_item = cJSON_GetObjectItem(root, "payload");

    if (!topic_item || !payload_item ||
        !cJSON_IsString(topic_item) || !cJSON_IsString(payload_item)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    esp_err_t ret = mqtt_publish(topic_item->valuestring,
                                 payload_item->valuestring, 1, 0);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "Alert flushed: %s", topic_item->valuestring);
    }

    cJSON_Delete(root);
    return ret;
}

// ─────────────────────────────────────────────
//  Alert handler task
// ─────────────────────────────────────────────
void task_alert_handler(void *arg)
{
    ESP_LOGI(TAG, "Alert handler task started");

    char container_id[MAX_CONTAINER_ID_LEN] = {0};

    // Bộ đếm để flush SD định kỳ khi có kết nối
    uint32_t flush_counter = 0;

    for (;;) {
        // Refresh container_id và trạng thái door
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        strlcpy(container_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
        bool door_en = g_door_enable;
        xSemaphoreGive(g_mutex_globals);

        // ── Door open alert ───────────────────────────────────────
        if (xSemaphoreTake(g_sem_door_alert, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (door_en && container_id[0] != '\0') {
                char *line = build_alert_line(container_id, "doorOpen");
                if (line) {
                    // 1. Lưu vào SD trước (đảm bảo không mất nếu publish thất bại)
                    sd_alert_append(line);

                    // 2. Thử publish ngay nếu đang có kết nối
                    if (mqtt_is_connected()) {
                        // Gọi flush để gửi tất cả pending (bao gồm vừa ghi)
                        sd_alert_flush(flush_alert_cb, NULL);
                    } else {
                        ESP_LOGW(TAG, "Door alert queued to SD (no MQTT)");
                    }
                    free(line);
                }
            } else if (!door_en) {
                ESP_LOGD(TAG, "Door alert suppressed (doorEnable=false)");
            } else {
                ESP_LOGW(TAG, "Door alert fired but containerID not set yet");
            }
        }

        // ── Collision alert ───────────────────────────────────────
        if (xSemaphoreTake(g_sem_collision, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (container_id[0] != '\0') {
                char *line = build_alert_line(container_id, "collision");
                if (line) {
                    sd_alert_append(line);

                    if (mqtt_is_connected()) {
                        sd_alert_flush(flush_alert_cb, NULL);
                    } else {
                        ESP_LOGW(TAG, "Collision alert queued to SD (no MQTT)");
                    }
                    free(line);
                }
            } else {
                ESP_LOGW(TAG, "Collision alert fired but containerID not set yet");
            }
        }

        // ── Định kỳ flush SD alerts khi có kết nối ───────────────
        // Kể cả khi không có alert mới (để xử lý backlog sau khi reconnect)
        flush_counter++;
        if (flush_counter >= 60) {   // ~3 giây (60 × 50ms)
            flush_counter = 0;
            if (mqtt_is_connected() && container_id[0] != '\0') {
                sd_alert_flush(flush_alert_cb, NULL);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}