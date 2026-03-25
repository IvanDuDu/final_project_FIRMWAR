#include "globals.h"
#include "nvs_manager.h"
#include "ble_manager.h"
#include "sensor_manager.h"
#include "actuators.h"
#include "stage_machine.h"
#include "sd_manager.h"
#include "sim_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

// ─────────────────────────────────────────────
//  RTOS sync primitives
// ─────────────────────────────────────────────
static void rtos_primitives_init(void)
{
    g_sem_door_alert  = xSemaphoreCreateBinary();
    g_sem_collision   = xSemaphoreCreateBinary();
    g_mutex_telemetry = xSemaphoreCreateMutex();
    g_mutex_globals   = xSemaphoreCreateMutex();
    g_evt_network     = xEventGroupCreate();

    assert(g_sem_door_alert);
    assert(g_sem_collision);
    assert(g_mutex_telemetry);
    assert(g_mutex_globals);
    assert(g_evt_network);

    ESP_LOGI(TAG, "RTOS primitives created");
}

// ─────────────────────────────────────────────
//  app_main
// ─────────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "Container Monitor Firmware starting...");
    ESP_LOGI(TAG, "ESP32-S3 | ESP-IDF + FreeRTOS");

    // 1. RTOS sync
    rtos_primitives_init();

    // 2. NVS
    ESP_ERROR_CHECK(nvs_manager_init());
    ESP_LOGI(TAG, "[1/7] NVS OK");

    // 3. SD Card — mount sớm để telemetry và alerts có thể ghi ngay
    esp_err_t sd_ret = sd_manager_mount();
    if (sd_ret != ESP_OK) {
        ESP_LOGW(TAG, "[2/7] SD mount failed (%s) — data will not be persisted",
                 esp_err_to_name(sd_ret));
    } else {
        ESP_LOGI(TAG, "[2/7] SD card OK");
    }

    // 4. SIM module — hw_init LUÔN được gọi bất kể on_board hay không
    //    Lý do: module cần thời gian khởi động dài, và GPS luôn cần chạy.
    //    Data bearer (4G) sẽ được bật riêng bởi server_connect khi cần.
    esp_err_t sim_ret = sim_manager_hw_init();
    if (sim_ret != ESP_OK) {
        ESP_LOGW(TAG, "[3/7] SIM hw_init failed — GPS unavailable, 4G unavailable");
    } else {
        ESP_LOGI(TAG, "[3/7] SIM module ready (GNSS started)");
        // Spawn background monitor task: tự recover nếu module reset,
        // tự reconnect data nếu đang ở land mode và bị ngắt.
        xTaskCreatePinnedToCore(task_sim_monitor, "sim_monitor",
                                2048, NULL, 2, NULL, 1);
    }

    // 5. Sensors — gps_init() dùng UART2 đã sẵn sàng từ bước 4
    esp_err_t sens_ret = sensor_manager_init();
    if (sens_ret != ESP_OK) {
        ESP_LOGW(TAG, "[4/7] Sensor init partial — continuing");
    } else {
        ESP_LOGI(TAG, "[4/7] Sensors OK");
    }

    // 6. Actuators
    ESP_ERROR_CHECK(actuators_init());
    ESP_LOGI(TAG, "[5/7] Actuators OK");

    // 7. BLE
    ESP_ERROR_CHECK(ble_manager_init());
    ESP_LOGI(TAG, "[6/7] BLE OK — advertising");

    ESP_LOGI(TAG, "[7/7] Starting stage machine");

    // 8. Stage machine — blocks forever
    stage_machine_run();

    ESP_LOGE(TAG, "stage_machine_run() returned unexpectedly");
    esp_restart();
}