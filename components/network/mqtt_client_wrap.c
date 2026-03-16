#include "mqtt_client_wrap.h"
#include "globals.h"

#include "mqtt_client.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MQTT";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        xEventGroupSetBits(g_evt_network, NET_EVT_MQTT_READY);
        ESP_LOGI(TAG, "MQTT connected to broker");
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        xEventGroupClearBits(g_evt_network, NET_EVT_MQTT_READY);
        ESP_LOGW(TAG, "MQTT disconnected — client will auto-reconnect");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type: %d",
                 ev->error_handle->error_type);
        break;

    default:
        break;
    }
}

esp_err_t mqtt_client_start(void)
{
    if (s_client != NULL) {
        ESP_LOGW(TAG, "MQTT already started");
        return ESP_OK;
    }

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .broker.address.port = MQTT_PORT,
        .session.keepalive   = 60,
        .network.reconnect_timeout_ms = 5000,
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) return ESP_FAIL;

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                    mqtt_event_handler, NULL);
    esp_err_t ret = esp_mqtt_client_start(s_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %d", ret);
        return ret;
    }

    // Wait for connection (up to 10s)
    EventBits_t bits = xEventGroupWaitBits(g_evt_network, NET_EVT_MQTT_READY,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
    if (!(bits & NET_EVT_MQTT_READY)) {
        ESP_LOGE(TAG, "MQTT broker connection timeout");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void mqtt_client_stop(void)
{
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        s_connected = false;
        xEventGroupClearBits(g_evt_network, NET_EVT_MQTT_READY);
    }
}

esp_err_t mqtt_publish(const char *topic, const char *payload, int qos, int retain)
{
    if (!s_client || !s_connected) {
        ESP_LOGW(TAG, "MQTT not connected — cannot publish to %s", topic);
        return ESP_FAIL;
    }
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload,
                                          strlen(payload), qos, retain);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Publish failed on topic %s", topic);
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Published [%s] msg_id=%d", topic, msg_id);
    return ESP_OK;
}

bool mqtt_is_connected(void) { return s_connected; }




/*

 need a way to send all the unsend mqtt message, so when the connection is back, the message can be sent immediately.
 Otherwise, the message will be lost and never sent. 
The outbox is used to store the unsend message, and when the connection is back, the message in the outbox will be sent immediately.
 The outbox is implemented as a linked list, and each node in the linked list is a mqtt message.
  The mqtt message contains the topic, payload, qos, retain, and msg_id. 
  The outbox is protected by a mutex to ensure thread safety.
   The outbox is also protected by a semaphore to ensure that only one task can access the outbox at a time.
    The outbox is also protected by an event group to ensure that the task that sends 
    the message from the outbox is notified when there are messages in the outbox.

*/

