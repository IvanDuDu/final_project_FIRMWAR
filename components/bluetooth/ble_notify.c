#include "ble_notify.h"
#include "ble_manager.h"
#include "globals.h"

#include "esp_log.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "BLE_NOTIFY";

void ble_notify_on_connect(void)
{
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);

    // Gather latest shipper ID from routeHistory
    char shipper_id[MAX_SHIPPER_ID_LEN] = "N/A";
    if (g_route_count > 0) {
        strlcpy(shipper_id,
                g_route_history[g_route_count - 1].shipper_id,
                MAX_SHIPPER_ID_LEN);
    }

    // Snapshot latest telemetry (avoid holding mutex during JSON build)
    float humidity  = g_latest_telemetry.humidity;
    float sur_temp  = g_latest_telemetry.sur_temp;
    float in_temp   = g_latest_telemetry.in_temp;
    bool  fan_en    = g_fan_enable;
    bool  uv_en     = g_uv_enable;

    char container_id[MAX_CONTAINER_ID_LEN];
    char customer_id[MAX_CUSTOMER_ID_LEN];
    char provider_id[MAX_PROVIDER_ID_LEN];
    strlcpy(container_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
    strlcpy(customer_id,  g_config.customer_id,  MAX_CUSTOMER_ID_LEN);
    strlcpy(provider_id,  g_config.provider_id,  MAX_PROVIDER_ID_LEN);

    xSemaphoreGive(g_mutex_globals);

    // Build JSON
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "containerId",  container_id);
    cJSON_AddStringToObject(root, "providerId",   provider_id);
    cJSON_AddStringToObject(root, "customerId",   customer_id);
    cJSON_AddStringToObject(root, "shipperId",    shipper_id);
    cJSON_AddNumberToObject(root, "humidity",     humidity);
    cJSON_AddNumberToObject(root, "surTemp",      sur_temp);
    cJSON_AddNumberToObject(root, "roomTemp",     in_temp);
    cJSON_AddBoolToObject  (root, "fanEnable",    fan_en);
    cJSON_AddBoolToObject  (root, "uvEnable",     uv_en);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json_str) {
        ESP_LOGE(TAG, "JSON serialisation failed");
        return;
    }

    // Small delay to let CCCD subscription settle before notifying
    vTaskDelay(pdMS_TO_TICKS(300));

    esp_err_t ret = ble_manager_notify(json_str);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Notify on connect failed (central may not have subscribed yet)");
    } else {
        ESP_LOGI(TAG, "Status packet sent on connect");
    }

    free(json_str);
}