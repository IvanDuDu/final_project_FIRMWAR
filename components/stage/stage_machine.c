#include "stage_machine.h"
#include "globals.h"
#include "nvs_manager.h"
#include "ble_manager.h"
#include "sensor_manager.h"
#include "telemetry_engine.h"
#include "alert_manager.h"
#include "server_connect.h"
#include "mqtt_client_wrap.h"
#include "actuators.h"
#include "sensors.h"       // ds3231_get_timestamp
#include "sd_manager.h"    // sd_telemetry_read_all, sd_telemetry_clear

#include "driver/gpio.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "STAGE";

// ─────────────────────────────────────────────
//  Task handles (spawned during Stage 2)
// ─────────────────────────────────────────────
static TaskHandle_t h_sensor     = NULL;
static TaskHandle_t h_telem_send = NULL;
static TaskHandle_t h_alert      = NULL;
static TaskHandle_t h_heartbeat  = NULL;

// ─────────────────────────────────────────────
//  Heartbeat task
// ─────────────────────────────────────────────
static void task_heartbeat(void *arg)
{
    for (;;) {
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        char container_id[MAX_CONTAINER_ID_LEN];
        strlcpy(container_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
        xSemaphoreGive(g_mutex_globals);

        if (container_id[0] != '\0' && mqtt_is_connected()) {
            char topic[80];
            snprintf(topic, sizeof(topic), "device/%s/heartbeat", container_id);
            mqtt_publish(topic, "{\"status\":\"alive\"}", 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
    }
}

// ─────────────────────────────────────────────
//  Spawn / stop Stage 2 worker tasks
// ─────────────────────────────────────────────
static void stage2_start_tasks(void)
{
    xTaskCreatePinnedToCore(task_sensor_read,    "sensor_read",   4096, NULL, 4, &h_sensor,     1);
    xTaskCreatePinnedToCore(task_telemetry_send, "telem_send",    3072, NULL, 3, &h_telem_send, 1);
    xTaskCreatePinnedToCore(task_alert_handler,  "alert_handler", 2048, NULL, 6, &h_alert,      0);
    xTaskCreatePinnedToCore(task_heartbeat,      "heartbeat",     2048, NULL, 1, &h_heartbeat,  1);
    ESP_LOGI(TAG, "Stage 2 tasks spawned");
}

static void stage2_stop_tasks(void)
{
    if (h_sensor)     { vTaskDelete(h_sensor);     h_sensor     = NULL; }
    if (h_telem_send) { vTaskDelete(h_telem_send); h_telem_send = NULL; }
    if (h_alert)      { vTaskDelete(h_alert);      h_alert      = NULL; }
    if (h_heartbeat)  { vTaskDelete(h_heartbeat);  h_heartbeat  = NULL; }
    ESP_LOGI(TAG, "Stage 2 tasks stopped");
}

// ─────────────────────────────────────────────
//  STAGE 1: Provider setup
// ─────────────────────────────────────────────
static void run_stage1(void)
{
    ESP_LOGI(TAG, "=== STAGE 1: Provider Setup ===");

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_door_enable = false;
    g_stage       = STAGE_SETUP;
    xSemaphoreGive(g_mutex_globals);

    nvs_save_bool(NVS_KEY_DOOR_ENABLE, false);
    nvs_save_int32(NVS_KEY_STAGE, STAGE_SETUP);

    // Chờ CMD 00 từ Provider (BLE handler điền g_config)
    ESP_LOGI(TAG, "Waiting for Provider BLE CMD 00...");
    while (1) {
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        bool has_config = (g_config.container_id[0] != '\0');
        xSemaphoreGive(g_mutex_globals);
        if (has_config) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "CMD 00 received, containerID=%s", g_config.container_id);

    // Publish setupCmplt nếu đã có kết nối MQTT
    if (mqtt_is_connected()) {
        char topic[80];
        snprintf(topic, sizeof(topic), "device/%s/setupCmplt", g_config.container_id);
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "containerId", g_config.container_id);
        cJSON_AddStringToObject(root, "providerId",  g_config.provider_id);
        char *payload = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        if (payload) {
            mqtt_publish(topic, payload, 1, 0);
            free(payload);
        }
    }

    // Polling cảm biến cửa cho đến khi đóng (GPIO HIGH)
    ESP_LOGI(TAG, "Waiting for door to be closed...");
    while (1) {
        if (gpio_get_level(PIN_DOOR_SENSOR) == 1) {
            ESP_LOGI(TAG, "Door closed — enabling door alert");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_door_enable = true;
    g_stage       = STAGE_TRANSPORT;
    xSemaphoreGive(g_mutex_globals);

    nvs_save_bool(NVS_KEY_DOOR_ENABLE, true);
    nvs_save_int32(NVS_KEY_STAGE, STAGE_TRANSPORT);

    ESP_LOGI(TAG, "Stage 1 complete -> Stage 2");
}

// ─────────────────────────────────────────────
//  STAGE 2: Transport
// ─────────────────────────────────────────────
static void run_stage2(void)
{
    ESP_LOGI(TAG, "=== STAGE 2: Transport ===");

    // Chờ CMD 01 đầu tiên từ Shipper
    ESP_LOGI(TAG, "Waiting for first Shipper BLE CMD 01...");
    while (1) {
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        int route_cnt = g_route_count;
        xSemaphoreGive(g_mutex_globals);
        if (route_cnt > 0) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    bool on_board = g_route_history[g_route_count - 1].is_on_board;
    xSemaphoreGive(g_mutex_globals);

    // Nếu đất liền: chờ CMD 08 có WiFi credentials trước khi connect
    if (!on_board) {
        ESP_LOGI(TAG, "Land mode — waiting for CMD 08 WiFi credentials...");
        char ssid[MAX_SSID_LEN] = {0};
        while (ssid[0] == '\0') {
            nvs_load_string(NVS_KEY_WIFI_SSID, ssid, MAX_SSID_LEN);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // Kết nối server
    esp_err_t ret = server_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Server connect failed, will retry in background");
    }

    // Spawn các worker tasks
    stage2_start_tasks();

    // Main loop: chờ chuyển sang STAGE_RECEIVE
    ESP_LOGI(TAG, "Transport loop running...");
    while (1) {
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        app_stage_t current_stage = g_stage;
        bool new_on_board = (g_route_count > 0)
                             ? g_route_history[g_route_count - 1].is_on_board
                             : false;
        xSemaphoreGive(g_mutex_globals);

        if (current_stage == STAGE_RECEIVE) break;

        if (new_on_board != on_board) {
            ESP_LOGI(TAG, "Transport mode changed — updating network");
            server_connect_update_transport(new_on_board);
            on_board = new_on_board;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    stage2_stop_tasks();
    ESP_LOGI(TAG, "Stage 2 complete -> Stage 3");
}

// ─────────────────────────────────────────────
//  STAGE 3: Receive / Handover
//
//  Tính total_loss bằng cách đọc file telemetry từ SD card,
//  parse từng record và cộng dồn trường "loss".
//  Không còn dùng g_send_telemetry[] trong RAM vì telemetry
//  đã được chuyển sang lưu trên SD.
// ─────────────────────────────────────────────
static void run_stage3(void)
{
    ESP_LOGI(TAG, "=== STAGE 3: Customer Receive ===");

    // ── Tính total_loss từ SD card ──
    float total_loss = 0.0f;

    if (sd_manager_is_mounted()) {
        // Đọc toàn bộ file telemetry pending từ SD
        char *raw = malloc(64 * 1024);   // 64 KB buffer
        if (raw) {
            size_t raw_len = 0;
            if (sd_telemetry_read_all(raw, 64 * 1024, &raw_len) == ESP_OK && raw_len > 0) {
                // Parse từng dòng JSONL và cộng dồn loss
                char *line = raw;
                char *nl;
                while ((nl = strchr(line, '\n')) != NULL) {
                    *nl = '\0';
                    if (strlen(line) > 0) {
                        cJSON *rec = cJSON_Parse(line);
                        if (rec) {
                            cJSON *telem = cJSON_GetObjectItem(rec, "telemetry");
                            if (telem) {
                                cJSON *loss_item = cJSON_GetObjectItem(telem, "loss");
                                if (loss_item && cJSON_IsNumber(loss_item)) {
                                    total_loss += (float)loss_item->valuedouble;
                                }
                            }
                            cJSON_Delete(rec);
                        }
                    }
                    line = nl + 1;
                }
                // Xử lý dòng cuối không có newline
                if (strlen(line) > 0) {
                    cJSON *rec = cJSON_Parse(line);
                    if (rec) {
                        cJSON *telem = cJSON_GetObjectItem(rec, "telemetry");
                        if (telem) {
                            cJSON *loss_item = cJSON_GetObjectItem(telem, "loss");
                            if (loss_item && cJSON_IsNumber(loss_item)) {
                                total_loss += (float)loss_item->valuedouble;
                            }
                        }
                        cJSON_Delete(rec);
                    }
                }
            }
            free(raw);
        } else {
            ESP_LOGW(TAG, "No memory to read SD telemetry for loss calc");
        }
    } else {
        // SD không mount: fallback dùng g_latest_telemetry.loss
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        total_loss = g_latest_telemetry.loss;
        xSemaphoreGive(g_mutex_globals);
        ESP_LOGW(TAG, "SD not mounted — using last known loss: %.3f", total_loss);
    }

    // ── Lấy timestamp từ RTC ──
    // sensors.h đã được include ở đầu file
    char timestamp[32];
    if (ds3231_get_timestamp(timestamp, sizeof(timestamp)) != ESP_OK) {
        snprintf(timestamp, sizeof(timestamp), "00:00:00 01/01/2000 GMT+0");
    }

    // ── Publish /received ──
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    char container_id[MAX_CONTAINER_ID_LEN];
    strlcpy(container_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
    xSemaphoreGive(g_mutex_globals);

    char topic[80];
    snprintf(topic, sizeof(topic), "device/%s/received", container_id);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "timestamp", timestamp);
    cJSON_AddNumberToObject(root, "loss",      (double)total_loss);
    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (payload) {
        mqtt_publish(topic, payload, 1, 1);   // retain=1
        free(payload);
        ESP_LOGI(TAG, "Published /received: total_loss=%.3fkg", total_loss);
    }

    // ── Ngắt kết nối server ──
    server_disconnect();

    // ── Xóa dữ liệu SD ──
    sd_telemetry_clear();
    sd_alert_clear();

    // ── Xóa NVS namespace ──
    nvs_manager_erase_all();

    // ── Reset RAM state ──
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    memset(&g_config,        0, sizeof(g_config));
    memset(g_route_history,  0, sizeof(g_route_history));
    g_route_count    = 0;
    memset(g_send_telemetry, 0, sizeof(g_send_telemetry));
    g_telem_count    = 0;
    g_door_enable    = false;
    g_fan_enable     = false;
    g_uv_enable      = false;
    g_prev_humidity  = 0.0f;
    g_stage          = STAGE_SETUP;
    xSemaphoreGive(g_mutex_globals);

    // ── Tắt actuators ──
    fan_set_duty(0);
    uv_set(false);

    ESP_LOGI(TAG, "Stage 3 complete — device reset to Stage 1");
}

// ─────────────────────────────────────────────
//  Public entry point
// ─────────────────────────────────────────────
void stage_machine_run(void)
{
    // Khôi phục stage đã lưu trong NVS khi boot
    int32_t saved_stage = STAGE_SETUP;
    nvs_load_int32(NVS_KEY_STAGE, &saved_stage);

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_stage = (app_stage_t)saved_stage;
    xSemaphoreGive(g_mutex_globals);

    ESP_LOGI(TAG, "Resuming from stage %ld", saved_stage);

    // Nếu crash giữa Stage 2: khôi phục config và flags từ NVS.
    // g_telem_count được khôi phục từ đếm số dòng trên SD card
    // (không dùng nvs_load_telemetry_buffer nữa vì telemetry lưu trên SD).
    if (saved_stage == STAGE_TRANSPORT) {
        nvs_load_container_config(&g_config);
        nvs_load_route_history(g_route_history, &g_route_count);
        nvs_load_bool(NVS_KEY_DOOR_ENABLE, (bool*)&g_door_enable);
        nvs_load_bool(NVS_KEY_FAN_ENABLE,  (bool*)&g_fan_enable);
        nvs_load_bool(NVS_KEY_UV_ENABLE,   (bool*)&g_uv_enable);

        // Đếm lại số records còn pending trên SD
        xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
        g_telem_count = sd_telemetry_count();
        xSemaphoreGive(g_mutex_telemetry);

        ESP_LOGI(TAG, "Crash recovery: %d telemetry records on SD", g_telem_count);
    }

    // Stage machine loop (khởi động lại sau Stage 3)
    while (1) {
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        app_stage_t stage = g_stage;
        xSemaphoreGive(g_mutex_globals);

        switch (stage) {
        case STAGE_SETUP:
            run_stage1();
            break;
        case STAGE_TRANSPORT:
            run_stage2();
            break;
        case STAGE_RECEIVE:
            run_stage3();
            break;
        default:
            ESP_LOGE(TAG, "Unknown stage %d, resetting to Stage 1", stage);
            xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
            g_stage = STAGE_SETUP;
            xSemaphoreGive(g_mutex_globals);
            break;
        }
    }
}