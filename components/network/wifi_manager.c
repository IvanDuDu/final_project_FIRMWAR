#include "wifi_manager.h"
#include "globals.h"
#include "nvs_manager.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "WIFI_MGR";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    5
#define WIFI_CONNECT_TIMEOUT_MS  15000

static EventGroupHandle_t s_wifi_evt_grp = NULL;
static int s_retry_count = 0;
static bool s_initialised = false;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < WIFI_MAX_RETRIES) {
            s_retry_count++;
            ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d", s_retry_count, WIFI_MAX_RETRIES);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_evt_grp, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_evt_grp, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(g_evt_network, NET_EVT_WIFI_READY);
    }
}

esp_err_t wifi_manager_connect(void)
{
    char ssid[MAX_SSID_LEN] = {0};
    char pass[MAX_PASS_LEN] = {0};

    esp_err_t ret = nvs_load_string(NVS_KEY_WIFI_SSID, ssid, MAX_SSID_LEN);
    if (ret != ESP_OK || ssid[0] == '\0') {
        ESP_LOGE(TAG, "No WiFi SSID found in NVS");
        return ESP_ERR_NOT_FOUND;
    }
    nvs_load_string(NVS_KEY_WIFI_PASS, pass, MAX_PASS_LEN);

    if (!s_initialised) {
        s_wifi_evt_grp = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             wifi_event_handler, NULL, NULL);
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             wifi_event_handler, NULL, NULL);
        s_initialised = true;
    }

    wifi_config_t wifi_cfg = {0};
    strlcpy((char *)wifi_cfg.sta.ssid,     ssid, sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_retry_count = 0;
    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_evt_grp,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "WiFi connection failed");
    return ESP_FAIL;
}

void wifi_manager_disconnect(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    xEventGroupClearBits(g_evt_network, NET_EVT_WIFI_READY);
    ESP_LOGI(TAG, "WiFi disconnected");
}

bool wifi_manager_is_connected(void)
{
    return (xEventGroupGetBits(g_evt_network) & NET_EVT_WIFI_READY) != 0;
}