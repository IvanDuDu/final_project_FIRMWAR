#include "telemetry_engine.h"
#include "globals.h"
#include "mqtt_client_wrap.h"
#include "nvs_manager.h"
#include "actuators.h"

#include "esp_log.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "TELEM";

// ─────────────────────────────────────────────
//  Loss calculation
// ─────────────────────────────────────────────
float telemetry_calc_loss(float Q0, float humidity,
                           float threshold, float weight)
{
    if (humidity <= threshold) return 0.0f;
    float loss = Q0 * (humidity - threshold) / 100.0f * (weight / 20.0f);
    return loss;
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
    cJSON_AddNumberToObject(telem, "altitude",  rec->latitude);   // spec uses "altitude" for lat
    cJSON_AddNumberToObject(telem, "lattitude", rec->longitude);  // spec field name kept as-is
    cJSON_AddNumberToObject(telem, "loss",      (double)rec->loss);

    cJSON_AddItemToObject(root, "telemetry", telem);

    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!str) return ESP_FAIL;

    size_t out_len = strlen(str);
    if (out_len >= len) {
        free(str);
        return ESP_ERR_NO_MEM;
    }
    memcpy(buf, str, out_len + 1);
    free(str);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Send task
// ─────────────────────────────────────────────
void task_telemetry_send(void *arg)
{
    ESP_LOGI(TAG, "Telemetry send task started");

    // Track last sent count to detect new batches
    int last_sent_batch = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));  // check every minute

        xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
        int count = g_telem_count;
        xSemaphoreGive(g_mutex_telemetry);

        // Only proceed if we have at least BATCH_SIZE records
        // and count is a new multiple of BATCH_SIZE
        if (count < TELEMETRY_BATCH_SIZE) continue;

        int current_batch = count / TELEMETRY_BATCH_SIZE;
        if (current_batch <= last_sent_batch) continue;

        ESP_LOGI(TAG, "Sending telemetry batch (count=%d)", count);

        // Build container topic
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        char container_id[MAX_CONTAINER_ID_LEN];
        strlcpy(container_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
        float humid_threshold = g_config.humid_threshold;
        float weight          = g_config.weight;
        xSemaphoreGive(g_mutex_globals);

        char topic[80];
        snprintf(topic, sizeof(topic), "device/%s/telemetry", container_id);

        // Take a snapshot of the buffer under mutex
        xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
        int snap_count = g_telem_count;
        telemetry_record_t snapshot[MAX_TELEMETRY_BUFFER];
        memcpy(snapshot, g_send_telemetry,
               sizeof(telemetry_record_t) * snap_count);
        xSemaphoreGive(g_mutex_telemetry);

        // Calculate loss and fill each record's loss field
        float Q0 = weight;  // initial quality = full weight
        for (int i = 0; i < snap_count; i++) {
            float loss = telemetry_calc_loss(Q0,
                                              snapshot[i].humidity,
                                              humid_threshold,
                                              weight);
            snapshot[i].loss = loss;
            // Q0 decreases with each loss event
            Q0 = (Q0 - loss > 0.0f) ? (Q0 - loss) : 0.0f;
        }

        // Fan auto-control based on latest reading
        if (snap_count > 0) {
            telemetry_record_t *latest = &snapshot[snap_count - 1];

            xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
            // Update g_latest_telemetry loss field
            g_latest_telemetry.loss = latest->loss;
            xSemaphoreGive(g_mutex_globals);

            fan_auto_control(latest->sur_temp, latest->in_temp,
                             latest->humidity, humid_threshold);
        }

        // Serialise all records into a JSON array
        cJSON *arr = cJSON_CreateArray();
        char rec_buf[MAX_JSON_BUF];
        for (int i = 0; i < snap_count; i++) {
            if (telemetry_serialize(&snapshot[i], rec_buf, sizeof(rec_buf)) == ESP_OK) {
                cJSON *item = cJSON_Parse(rec_buf);
                if (item) cJSON_AddItemToArray(arr, item);
            }
        }
        char *payload = cJSON_PrintUnformatted(arr);
        cJSON_Delete(arr);

        if (!payload) {
            ESP_LOGE(TAG, "Failed to serialise telemetry batch");
            continue;
        }

        // Attempt publish
        esp_err_t ret = mqtt_publish(topic, payload, 1, 0);
        free(payload);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Telemetry batch sent, clearing buffer");
            xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
            memset(g_send_telemetry, 0,
                   sizeof(telemetry_record_t) * g_telem_count);
            g_telem_count = 0;
            xSemaphoreGive(g_mutex_telemetry);

            // Also persist cleared state to NVS
            nvs_save_telemetry_buffer(g_send_telemetry, 0);
            nvs_save_int32(NVS_KEY_TELEM_COUNT, 0);

            last_sent_batch = 0;
        } else {
            ESP_LOGW(TAG, "Publish failed — keeping buffer, will retry at next batch");
            last_sent_batch = current_batch - 1;  // allow retry next multiple
        }
    }
}