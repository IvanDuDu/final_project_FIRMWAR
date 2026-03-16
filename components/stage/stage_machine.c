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

#include "driver/gpio.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>

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
//  Spawn Stage 2 worker tasks
// ─────────────────────────────────────────────
static void stage2_start_tasks(void)
{
    xTaskCreatePinnedToCore(task_sensor_read,    "sensor_read",   4096, NULL, 4, &h_sensor,     1);
    xTaskCreatePinnedToCore(task_telemetry_send, "telem_send",    3072, NULL, 3, &h_telem_send, 1);
    xTaskCreatePinnedToCore(task_alert_handler,  "alert_handler", 2048, NULL, 6, &h_alert,      0);
    xTaskCreatePinnedToCore(task_heartbeat,      "heartbeat",     1024, NULL, 1, &h_heartbeat,  1);
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

    // ── Wait for CMD 00 (BLE handler populates g_config) ──
    ESP_LOGI(TAG, "Waiting for Provider BLE CMD 00...");
    while (1) {
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        bool has_config = (g_config.container_id[0] != '\0');
        xSemaphoreGive(g_mutex_globals);
        if (has_config) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "CMD 00 received, containerID=%s", g_config.container_id);

    // ── Publish setupCmplt ────────────────────
    // (Server may not be connected yet — attempt anyway if connected)
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

    // ── Poll door sensor until CLOSED (GPIO 8 = HIGH) ──
    ESP_LOGI(TAG, "Waiting for door to be closed...");
    while (1) {
        int level = gpio_get_level(PIN_DOOR_SENSOR);
        if (level == 1) {
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

    ESP_LOGI(TAG, "Stage 1 complete → Stage 2");
}

// ─────────────────────────────────────────────
//  STAGE 2: Transport
// ─────────────────────────────────────────────
static void run_stage2(void)
{
    ESP_LOGI(TAG, "=== STAGE 2: Transport ===");

    // ── Wait for first Shipper CMD 01 ─────────
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

    // ── If land (isOnBoard=false), wait for CMD 08 with WiFi credentials ──
    if (!on_board) {
        ESP_LOGI(TAG, "Land mode — waiting for CMD 08 WiFi credentials...");
        char ssid[MAX_SSID_LEN] = {0};
        while (ssid[0] == '\0') {
            nvs_load_string(NVS_KEY_WIFI_SSID, ssid, MAX_SSID_LEN);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // ── Connect to server ─────────────────────
    esp_err_t ret = server_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Server connect failed, will retry in background");
    }

    // ── Spawn worker tasks ────────────────────
    stage2_start_tasks();

    // ── Main loop: wait for stage change to STAGE_RECEIVE ─
    ESP_LOGI(TAG, "Transport loop running...");
    while (1) {
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        app_stage_t current_stage = g_stage;

        // Check if shipper transport mode changed (new CMD 01 with different isOnBoard)
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
    ESP_LOGI(TAG, "Stage 2 complete → Stage 3");
}

// ─────────────────────────────────────────────
//  STAGE 3: Receive / Handover
// ─────────────────────────────────────────────
static void run_stage3(void)
{
    ESP_LOGI(TAG, "=== STAGE 3: Customer Receive ===");

    // Calculate final cumulative loss
    xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
    float total_loss = 0.0f;
    for (int i = 0; i < g_telem_count; i++) {
        total_loss += g_send_telemetry[i].loss;
    }
    char timestamp[32];
    xSemaphoreGive(g_mutex_telemetry);

    ds3231_get_timestamp(timestamp, sizeof(timestamp));

    // Publish /received
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
        mqtt_publish(topic, payload, 1, 1);  // retain=1 for receive confirmation
        free(payload);
        ESP_LOGI(TAG, "Published /received: total_loss=%.3fkg", total_loss);
    }

    // Disconnect from server
    server_disconnect();

    // Clear all NVS + RAM state
   // nvs_manager_erase_all();

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    memset(&g_config, 0, sizeof(g_config));
    memset(g_route_history, 0, sizeof(g_route_history));
    g_route_count   = 0;
    memset(g_send_telemetry, 0, sizeof(g_send_telemetry));
    g_telem_count   = 0;
    g_door_enable   = false;
    g_fan_enable    = false;
    g_uv_enable     = false;
    g_prev_humidity = 0.0f;
    g_stage         = STAGE_SETUP;
    xSemaphoreGive(g_mutex_globals);

    // Turn off actuators
    fan_set_duty(0);
    uv_set(false);

    ESP_LOGI(TAG, "Stage 3 complete — device reset to Stage 1");
}

// ─────────────────────────────────────────────
//  Public entry point
// ─────────────────────────────────────────────
void stage_machine_run(void)
{
    // Restore persisted stage on boot
    int32_t saved_stage = STAGE_SETUP;
    nvs_load_int32(NVS_KEY_STAGE, &saved_stage);

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_stage = (app_stage_t)saved_stage;
    xSemaphoreGive(g_mutex_globals);

    ESP_LOGI(TAG, "Resuming from stage %d", saved_stage);

    // If crashed mid-Stage2, reload persisted state
    if (saved_stage == STAGE_TRANSPORT) {
        nvs_load_container_config(&g_config);
        nvs_load_route_history(g_route_history, &g_route_count);
        nvs_load_telemetry_buffer(g_send_telemetry, &g_telem_count);
        nvs_load_bool(NVS_KEY_DOOR_ENABLE, (bool*)&g_door_enable);
        nvs_load_bool(NVS_KEY_FAN_ENABLE,  (bool*)&g_fan_enable);
        nvs_load_bool(NVS_KEY_UV_ENABLE,   (bool*)&g_uv_enable);
    }

    // Stage machine loop (restarts after Stage 3)
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