#include "globals.h"
#include "nvs_manager.h"
#include "ble_manager.h"
#include "sensor_manager.h"
#include "actuators.h"
#include "stage_machine.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

// ─────────────────────────────────────────────
//  Create RTOS synchronisation primitives
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
//  app_main — ESP-IDF entry point
// ─────────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "Container Monitor Firmware starting...");
    ESP_LOGI(TAG, "ESP32-S3 | ESP-IDF + FreeRTOS");

    // 1. RTOS sync primitives
    rtos_primitives_init();

    // 2. NVS
    ESP_ERROR_CHECK(nvs_manager_init());
    ESP_LOGI(TAG, "[1/5] NVS OK");

    // 3. Sensors (I2C buses + GPIO interrupts)
    //    Non-fatal: log and continue if a sensor is absent
    esp_err_t ret = sensor_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[2/5] Sensor init partial — continuing");
    } else {
        ESP_LOGI(TAG, "[2/5] Sensors OK");
    }

    // 4. Actuators (LEDC fan PWM + UV GPIO)
    ESP_ERROR_CHECK(actuators_init());
    ESP_LOGI(TAG, "[3/5] Actuators OK");

    // 5. BLE manager (always-on, advertises immediately)
    ESP_ERROR_CHECK(ble_manager_init());
    ESP_LOGI(TAG, "[4/5] BLE OK — advertising");

    ESP_LOGI(TAG, "[5/5] Starting stage machine");

    // 6. Stage machine — runs forever on the main task (Core 0)
    //    Network tasks are spawned on-demand inside stage2.
    stage_machine_run();

    // Should never reach here
    ESP_LOGE(TAG, "stage_machine_run() returned unexpectedly");
    esp_restart();
}