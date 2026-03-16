#include "server_connect.h"
#include "wifi_manager.h"
#include "sim_manager.h"
#include "mqtt_client_wrap.h"
#include "globals.h"

#include "esp_log.h"

static const char *TAG = "SRV_CONN";

// Track current transport to detect changes
static bool s_current_is_on_board = false;
static bool s_transport_active    = false;

// ─── Internal: connect underlying transport ───────────
static esp_err_t connect_transport(bool is_on_board)
{
    esp_err_t ret;
    if (is_on_board) {
        ESP_LOGI(TAG, "Transport: WiFi (sea mode)");
        ret = wifi_manager_connect();
    } else {
        ESP_LOGI(TAG, "Transport: 5G SIM (land mode)");
        ret = sim_manager_connect();
    }
    return ret;
}

static void disconnect_transport(bool is_on_board)
{
    if (is_on_board) {
        wifi_manager_disconnect();
    } else {
        sim_manager_disconnect();
    }
}

// ─────────────────────────────────────────────
esp_err_t server_connect(void)
{
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    bool on_board = (g_route_count > 0)
                    ? g_route_history[g_route_count - 1].is_on_board
                    : false;
    xSemaphoreGive(g_mutex_globals);

    esp_err_t ret = connect_transport(on_board);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Transport connect failed");
        return ret;
    }

    ret = mqtt_client_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed");
        return ret;
    }

    s_current_is_on_board = on_board;
    s_transport_active    = true;
    return ESP_OK;
}

esp_err_t server_connect_update_transport(bool is_on_board)
{
    if (!s_transport_active) {
        // First time — just connect
        return server_connect();
    }

    if (is_on_board == s_current_is_on_board) {
        ESP_LOGI(TAG, "Transport unchanged, no switch needed");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Transport change: %s → %s",
             s_current_is_on_board ? "WiFi" : "SIM",
             is_on_board ? "WiFi" : "SIM");

    mqtt_client_stop();
    disconnect_transport(s_current_is_on_board);

    esp_err_t ret = connect_transport(is_on_board);
    if (ret != ESP_OK) return ret;

    ret = mqtt_client_start();
    if (ret != ESP_OK) return ret;

    s_current_is_on_board = is_on_board;
    return ESP_OK;
}

void server_disconnect(void)
{
    mqtt_client_stop();
    disconnect_transport(s_current_is_on_board);
    s_transport_active = false;
}