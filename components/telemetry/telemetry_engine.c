#include "telemetry_engine.h"
#include "globals.h"
#include "mqtt_client_wrap.h"
#include "nvs_manager.h"
#include "actuators.h"
#include "sd_manager.h"

#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "TELEM";

// Dùng heap thay vì stack để đọc file SD (có thể lớn)
#define SD_READ_BUF_SIZE    (64 * 1024)   // 64 KB tối đa cho 1 lần đọc

// ─────────────────────────────────────────────
//  Loss calculation
// ─────────────────────────────────────────────
float telemetry_calc_loss(float Q0, float humidity,
                           float threshold, float weight)
{
    if (humidity <= threshold) return 0.0f;
    float loss = Q0 * (humidity - threshold) / 100.0f * (weight / 20.0f);
    return loss < 0.0f ? 0.0f : loss;
}

// ─────────────────────────────────────────────
//  JSON serialisation
// ─────────────────────────────────────────────
esp_err_t telemetry_serialize(const telemetry_record_t *rec, char *buf, size_t len)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "timestamp", rec->timestamp);

    cJSON *telem = cJSON_CreateObject();
    cJSON_AddNumberToObject(telem, "inTemp",    (double)rec->in_temp);
    cJSON_AddNumberToObject(telem, "surTemp",   (double)rec->sur_temp);
    cJSON_AddNumberToObject(telem, "humidity",  (double)rec->humidity);
    cJSON_AddNumberToObject(telem, "lean",      (double)rec->lean);
    cJSON_AddNumberToObject(telem, "altitude",  rec->latitude);
    cJSON_AddNumberToObject(telem, "lattitude", rec->longitude);
    cJSON_AddNumberToObject(telem, "loss",      (double)rec->loss);

    cJSON_AddItemToObject(root, "telemetry", telem);

    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!str) return ESP_FAIL;

    size_t out_len = strlen(str);
    if (out_len >= len) { free(str); return ESP_ERR_NO_MEM; }

    memcpy(buf, str, out_len + 1);
    free(str);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Send task — đọc từ SD, gửi batch, xóa khi thành công
// ─────────────────────────────────────────────
void task_telemetry_send(void *arg)
{
    ESP_LOGI(TAG, "Telemetry send task started");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_SEND_INTERVAL_MS));

        // ── Kiểm tra đủ điều kiện để gửi ──
        xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
        int count = g_telem_count;
        xSemaphoreGive(g_mutex_telemetry);

        if (count < TELEMETRY_BATCH_SIZE) {
            ESP_LOGD(TAG, "Not enough records yet (%d/%d)", count, TELEMETRY_BATCH_SIZE);
            continue;
        }

        if (!mqtt_is_connected()) {
            ESP_LOGW(TAG, "MQTT not connected — keeping %d records on SD", count);
            continue;
        }

        // ── Lấy container config ──
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        char container_id[MAX_CONTAINER_ID_LEN];
        strlcpy(container_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
        float humid_threshold = g_config.humid_threshold;
        float weight          = g_config.weight;
        xSemaphoreGive(g_mutex_globals);

        if (container_id[0] == '\0') {
            ESP_LOGW(TAG, "Container ID not set yet");
            continue;
        }

        // ── Đọc toàn bộ SD file vào heap ──
        char *raw_buf = malloc(SD_READ_BUF_SIZE);
        if (!raw_buf) {
            ESP_LOGE(TAG, "No memory for SD read buffer");
            continue;
        }

        size_t raw_len = 0;
        esp_err_t sd_ret = sd_telemetry_read_all(raw_buf, SD_READ_BUF_SIZE, &raw_len);
        if (sd_ret != ESP_OK || raw_len == 0) {
            ESP_LOGW(TAG, "SD read empty or error: %d", sd_ret);
            free(raw_buf);
            continue;
        }

        ESP_LOGI(TAG, "Read %u bytes from SD (%d records)", (unsigned)raw_len, count);

        // ── Parse từng dòng JSON và build array ──
        cJSON *arr = cJSON_CreateArray();
        float Q0 = weight;
        char *line = raw_buf;
        char *nl;
        int parsed = 0;

        while ((nl = strchr(line, '\n')) != NULL) {
            *nl = '\0';
            if (strlen(line) > 0) {
                cJSON *item = cJSON_Parse(line);
                if (item) {
                    // Cập nhật loss field theo chuỗi Q0 giảm dần
                    cJSON *telem = cJSON_GetObjectItem(item, "telemetry");
                    if (telem) {
                        cJSON *hum_item = cJSON_GetObjectItem(telem, "humidity");
                        float  hum = hum_item ? (float)hum_item->valuedouble : 0.0f;
                        float  loss = telemetry_calc_loss(Q0, hum, humid_threshold, weight);
                        cJSON_DeleteItemFromObject(telem, "loss");
                        cJSON_AddNumberToObject(telem, "loss", (double)loss);
                        Q0 = (Q0 - loss > 0.0f) ? (Q0 - loss) : 0.0f;
                    }
                    cJSON_AddItemToArray(arr, item);
                    parsed++;
                }
            }
            line = nl + 1;
        }
        // Dòng cuối (không có newline)
        if (strlen(line) > 0) {
            cJSON *item = cJSON_Parse(line);
            if (item) {
                cJSON *telem = cJSON_GetObjectItem(item, "telemetry");
                if (telem) {
                    cJSON *hum_item = cJSON_GetObjectItem(telem, "humidity");
                    float  hum = hum_item ? (float)hum_item->valuedouble : 0.0f;
                    float  loss = telemetry_calc_loss(Q0, hum, humid_threshold, weight);
                    cJSON_DeleteItemFromObject(telem, "loss");
                    cJSON_AddNumberToObject(telem, "loss", (double)loss);
                }
                cJSON_AddItemToArray(arr, item);
                parsed++;
            }
        }
        free(raw_buf);

        if (parsed == 0) {
            ESP_LOGW(TAG, "No valid records parsed from SD");
            cJSON_Delete(arr);
            continue;
        }

        // ── Cập nhật g_latest_telemetry.loss với loss của record cuối ──
        // (để fan control và BLE notify có giá trị đúng)
        cJSON *last = cJSON_GetArrayItem(arr, cJSON_GetArraySize(arr) - 1);
        if (last) {
            cJSON *telem = cJSON_GetObjectItem(last, "telemetry");
            if (telem) {
                cJSON *loss_item = cJSON_GetObjectItem(telem, "loss");
                if (loss_item) {
                    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
                    g_latest_telemetry.loss = (float)loss_item->valuedouble;
                    xSemaphoreGive(g_mutex_globals);
                }
            }
        }

        // ── Serialize array thành JSON string ──
        char *payload = cJSON_PrintUnformatted(arr);
        cJSON_Delete(arr);

        if (!payload) {
            ESP_LOGE(TAG, "Failed to serialise telemetry array");
            continue;
        }

        // ── Publish ──
        char topic[80];
        snprintf(topic, sizeof(topic), "device/%s/telemetry", container_id);

        esp_err_t pub_ret = mqtt_publish(topic, payload, 1, 0);
        free(payload);

        if (pub_ret == ESP_OK) {
            ESP_LOGI(TAG, "Telemetry batch (%d records) sent — clearing SD", parsed);

            // Xóa file SD
            sd_telemetry_clear();

            // Reset counter trong RAM
            xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
            g_telem_count = 0;
            xSemaphoreGive(g_mutex_telemetry);

        } else {
            ESP_LOGW(TAG, "Publish failed — %d records kept on SD, retry next cycle", parsed);
        }
    }
}