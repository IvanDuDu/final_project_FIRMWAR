#include "espnow_manager.h"
#include "globals.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "ESPNOW";

// ─────────────────────────────────────────────
//  Constants 
// ─────────────────────────────────────────────

#define MSG_TYPE_REQ  0
#define MSG_TYPE_ACK  1

// Dedup circular buffer size
#define DEDUP_BUF_SIZE  32
#define MAX_PATH_HOPS   16   // max MACs stored in a path trace
#define MAC_STR_LEN     18   // "AA:BB:CC:DD:EE:FF\0"
#define ESPNOW_PKT_MAX  250  // ESP-NOW max payload 250 bytes

// ─────────────────────────────────────────────
//  Module state
// ─────────────────────────────────────────────
static char  s_own_mac[MAC_STR_LEN];         // this device's MAC (ASCII)
static char  s_dedup[DEDUP_BUF_SIZE][24];    // recently seen msg_ids
static int   s_dedup_idx = 0;
static SemaphoreHandle_t s_mutex_dedup = NULL;

// LED auto-off timestamps (xTaskGetTickCount at time of activation)
static volatile TickType_t s_path_led_off_tick = 0;
static volatile TickType_t s_dest_led_off_tick = 0;

// volatile bool g_path_led_active = false;
// volatile bool g_dest_led_active = false;

// ─────────────────────────────────────────────
//  LED helpers
// ─────────────────────────────────────────────
static void leds_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_PATH) | (1ULL << PIN_LED_DEST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_LED_PATH, 0);
    gpio_set_level(PIN_LED_DEST, 0);
    ESP_LOGI(TAG, "Path LEDs init (PATH=GPIO%d, DEST=GPIO%d)", PIN_LED_PATH, PIN_LED_DEST);
}

static void led_path_on(void)
{
    gpio_set_level(PIN_LED_PATH, 1);
    g_path_led_active  = true;
    s_path_led_off_tick = xTaskGetTickCount() +
                          pdMS_TO_TICKS(PATH_LED_DURATION_MS);
    ESP_LOGI(TAG, "PATH LED on (auto-off in 5 min)");
}

static void led_dest_on(void)
{
    gpio_set_level(PIN_LED_DEST, 1);
    g_dest_led_active  = true;
    s_dest_led_off_tick = xTaskGetTickCount() +
                          pdMS_TO_TICKS(PATH_LED_DURATION_MS);
    ESP_LOGI(TAG, "DEST LED on (auto-off in 5 min)");
}

// ─────────────────────────────────────────────
//  LED timer task  (runs forever, checks every second)
// ─────────────────────────────────────────────
void task_led_timer(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        TickType_t now = xTaskGetTickCount();

        if (g_path_led_active &&
            (now - s_path_led_off_tick) < (TickType_t)(UINT32_MAX / 2)) {
            // Unsigned wrap-safe: if now >= off_tick the subtraction goes negative
            // For simplicity use direct comparison
        }
        // Simpler approach: store absolute off_tick and compare
        if (g_path_led_active && s_path_led_off_tick != 0 &&
            ((TickType_t)(now - s_path_led_off_tick) < pdMS_TO_TICKS(PATH_LED_DURATION_MS * 2))) {
            // Check if we've passed the deadline
            if (xTaskGetTickCount() >= s_path_led_off_tick) {
                gpio_set_level(PIN_LED_PATH, 0);
                g_path_led_active = false;
                s_path_led_off_tick = 0;
                ESP_LOGI(TAG, "PATH LED auto-off");
            }
        }

        if (g_dest_led_active && s_dest_led_off_tick != 0 &&
            xTaskGetTickCount() >= s_dest_led_off_tick) {
            gpio_set_level(PIN_LED_DEST, 0);
            g_dest_led_active = false;
            s_dest_led_off_tick = 0;
            ESP_LOGI(TAG, "DEST LED auto-off");
        }
    }
}

// ─────────────────────────────────────────────
//  Deduplication helpers
// ─────────────────────────────────────────────
static bool dedup_seen(const char *msg_id)
{
    xSemaphoreTake(s_mutex_dedup, portMAX_DELAY);
    bool found = false;
    for (int i = 0; i < DEDUP_BUF_SIZE; i++) {
        if (strcmp(s_dedup[i], msg_id) == 0) { found = true; break; }
    }
    if (!found) {
        strlcpy(s_dedup[s_dedup_idx], msg_id, sizeof(s_dedup[0]));
        s_dedup_idx = (s_dedup_idx + 1) % DEDUP_BUF_SIZE;
    }
    xSemaphoreGive(s_mutex_dedup);
    return found;
}

// ─────────────────────────────────────────────
//  MAC helpers
// ─────────────────────────────────────────────
static void mac_to_str(const uint8_t mac[6], char *out)
{
    snprintf(out, MAC_STR_LEN, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void str_to_mac(const char *str, uint8_t mac[6])
{
    unsigned int m[6] = {0};
    sscanf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
           &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]);
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
}

// ─────────────────────────────────────────────
//  Build REQ packet JSON
// ─────────────────────────────────────────────
static char *build_req_packet(const char *msg_id,
                               const char *target_id,
                               uint8_t ttl,
                               uint8_t hop,
                               cJSON *path_arr)   // may be NULL
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "t",   MSG_TYPE_REQ);
    cJSON_AddStringToObject(root, "id",  msg_id);
    cJSON_AddStringToObject(root, "dst", target_id);
    cJSON_AddStringToObject(root, "src", s_own_mac);
    cJSON_AddNumberToObject(root, "ttl", ttl);
    cJSON_AddNumberToObject(root, "hop", hop);

    cJSON *arr = path_arr ? cJSON_Duplicate(path_arr, true)
                          : cJSON_CreateArray();
    // Append our own MAC to the path
    cJSON_AddItemToArray(arr, cJSON_CreateString(s_own_mac));
    cJSON_AddItemToObject(root, "path", arr);

    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return str;  // caller must free
}

// ─────────────────────────────────────────────
//  Build ACK packet JSON
// ─────────────────────────────────────────────
static char *build_ack_packet(const char *msg_id,
                               const char *target_id,
                               cJSON *path_arr)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "t",     MSG_TYPE_ACK);
    cJSON_AddStringToObject(root, "id",    msg_id);
    cJSON_AddStringToObject(root, "dst",   target_id);
    cJSON_AddBoolToObject  (root, "found", true);
    cJSON *arr = path_arr ? cJSON_Duplicate(path_arr, true) : cJSON_CreateArray();
    cJSON_AddItemToObject(root, "path", arr);
    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return str;
}

// ─────────────────────────────────────────────
//  Send raw ESP-NOW packet (broadcast or unicast)
// ─────────────────────────────────────────────
static void espnow_send_raw(const uint8_t peer_mac[6], const char *json)
{
    size_t len = strlen(json);
    if (len > ESPNOW_PKT_MAX) {
        ESP_LOGW(TAG, "Packet too large (%d bytes), truncating", (int)len);
        len = ESPNOW_PKT_MAX;
    }

    // Register peer if not already (ESP-NOW requires peer registration)
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx   = ESP_IF_WIFI_STA;

    if (!esp_now_is_peer_exist(peer_mac)) {
        esp_now_add_peer(&peer);
    }

    esp_err_t ret = esp_now_send(peer_mac, (const uint8_t *)json, len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_send failed: %d", ret);
    }
}

// ─────────────────────────────────────────────
//  Broadcast REQ flood
// ─────────────────────────────────────────────
static void flood_req(const char *msg_id, const char *target_id,
                      uint8_t ttl, uint8_t hop, cJSON *path_arr)
{
    if (ttl == 0) return;

    char *pkt = build_req_packet(msg_id, target_id, ttl - 1, hop + 1, path_arr);
    if (!pkt) return;

    const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    espnow_send_raw(bcast, pkt);
    ESP_LOGI(TAG, "Flooded REQ [%s] dst=%s ttl=%d hop=%d",
             msg_id, target_id, ttl - 1, hop + 1);
    free(pkt);
}

// ─────────────────────────────────────────────
//  Send ACK unicast back one hop (to previous node in path)
// ─────────────────────────────────────────────
static void send_ack_hop(const char *msg_id, const char *target_id,
                          cJSON *path_arr)
{
    int sz = cJSON_GetArraySize(path_arr);
    if (sz < 2) return;  // no previous hop to send to

    // Previous hop is second-to-last entry in path
    cJSON *prev_item = cJSON_GetArrayItem(path_arr, sz - 2);
    if (!prev_item || !cJSON_IsString(prev_item)) return;

    uint8_t prev_mac[6];
    str_to_mac(prev_item->valuestring, prev_mac);

    char *pkt = build_ack_packet(msg_id, target_id, path_arr);
    if (!pkt) return;

    espnow_send_raw(prev_mac, pkt);
    ESP_LOGI(TAG, "ACK hop → %s", prev_item->valuestring);
    free(pkt);
}

// ─────────────────────────────────────────────
//  Receive callback (called from ESP-NOW internal task)
// ─────────────────────────────────────────────
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int data_len)
{
    if (data_len <= 0 || data_len > ESPNOW_PKT_MAX) return;

    // Null-terminate for JSON parsing
    char buf[ESPNOW_PKT_MAX + 1];
    memcpy(buf, data, data_len);
    buf[data_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) return;

    cJSON *t_item   = cJSON_GetObjectItem(root, "t");
    cJSON *id_item  = cJSON_GetObjectItem(root, "id");
    cJSON *dst_item = cJSON_GetObjectItem(root, "dst");
    cJSON *path_arr = cJSON_GetObjectItem(root, "path");

    if (!t_item || !id_item || !dst_item) {
        cJSON_Delete(root);
        return;
    }

    int  msg_type = t_item->valueint;
    const char *msg_id    = id_item->valuestring;
    const char *target_id = dst_item->valuestring;

    // ── Handle REQ ────────────────────────────────────────────────────────────
    if (msg_type == MSG_TYPE_REQ) {

        // Dedup check
        if (dedup_seen(msg_id)) {
            cJSON_Delete(root);
            return;
        }

        cJSON *ttl_item = cJSON_GetObjectItem(root, "ttl");
        cJSON *hop_item = cJSON_GetObjectItem(root, "hop");
        uint8_t ttl = ttl_item ? (uint8_t)ttl_item->valueint : 0;
        uint8_t hop = hop_item ? (uint8_t)hop_item->valueint : 0;

        // Get our container_id
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        char my_id[MAX_CONTAINER_ID_LEN];
        strlcpy(my_id, g_config.container_id, MAX_CONTAINER_ID_LEN);
        xSemaphoreGive(g_mutex_globals);

        // Build updated path including our MAC
        cJSON *updated_path = path_arr
                               ? cJSON_Duplicate(path_arr, true)
                               : cJSON_CreateArray();
        cJSON_AddItemToArray(updated_path, cJSON_CreateString(s_own_mac));

        bool is_target = (my_id[0] != '\0' &&
                          strcmp(my_id, target_id) == 0);

        if (is_target) {
            // ── We are the destination ───────────────────────────────────────
            ESP_LOGI(TAG, "*** I AM THE TARGET for [%s] ***", msg_id);
            led_dest_on();

            // Send ACK back along the path
            char *ack = build_ack_packet(msg_id, target_id, updated_path);
            if (ack) {
                int psz = cJSON_GetArraySize(updated_path);
                if (psz >= 2) {
                    cJSON *prev = cJSON_GetArrayItem(updated_path, psz - 2);
                    if (prev && cJSON_IsString(prev)) {
                        uint8_t prev_mac[6];
                        str_to_mac(prev->valuestring, prev_mac);
                        espnow_send_raw(prev_mac, ack);
                        ESP_LOGI(TAG, "ACK sent to %s", prev->valuestring);
                    }
                }
                // If we are also the originator (1-hop case)
                free(ack);
            }
        } else {
            // ── We are an intermediate node ──────────────────────────────────
            // Re-flood if TTL allows
            if (ttl > 0) {
                flood_req(msg_id, target_id, ttl, hop, updated_path);
            } else {
                ESP_LOGD(TAG, "TTL exhausted for [%s], dropping", msg_id);
            }
        }
        cJSON_Delete(updated_path);
    }

    // ── Handle ACK ────────────────────────────────────────────────────────────
    else if (msg_type == MSG_TYPE_ACK) {

        if (dedup_seen(msg_id)) {
            cJSON_Delete(root);
            return;
        }

        ESP_LOGI(TAG, "ACK received for [%s] — I am on the PATH", msg_id);

        // Light path LED — we are an intermediate node on the found path
        led_path_on();

        // Forward ACK to previous hop in path
        if (path_arr) {
            send_ack_hop(msg_id, target_id, path_arr);
        }
    }

    cJSON_Delete(root);
}

// ─────────────────────────────────────────────
//  Public: init
// ─────────────────────────────────────────────
esp_err_t espnow_manager_init(void)
{
    // Initialise LEDs
    leds_gpio_init();

    // Dedup mutex
    s_mutex_dedup = xSemaphoreCreateMutex();
    if (!s_mutex_dedup) return ESP_ERR_NO_MEM;

    // Cache our own MAC address
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MAC: %d", ret);
        return ret;
    }
    mac_to_str(mac, s_own_mac);
    ESP_LOGI(TAG, "Own MAC: %s", s_own_mac);

    // Initialise ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %d", ret);
        return ret;
    }

    // Register broadcast peer (required before sending to FF:FF:FF:FF:FF:FF)
    esp_now_peer_info_t bcast_peer = {0};
    memset(bcast_peer.peer_addr, 0xFF, 6);
    bcast_peer.channel = ESPNOW_CHANNEL;
    bcast_peer.encrypt = false;
    bcast_peer.ifidx   = ESP_IF_WIFI_STA;
    esp_now_add_peer(&bcast_peer);

    // Register receive callback
    ret = esp_now_register_recv_cb(espnow_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW pathfind manager ready (channel %d)", ESPNOW_CHANNEL);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Public: start a new pathfind flood from this node
// ─────────────────────────────────────────────
void espnow_pathfind_start(const char *target_container_id)
{
    if (!target_container_id || target_container_id[0] == '\0') {
        ESP_LOGW(TAG, "pathfind_start: empty target");
        return;
    }

    // Build a unique msg_id from our MAC + current tick
    char msg_id[24];
    snprintf(msg_id, sizeof(msg_id), "%c%c%c%c%05lu",
             s_own_mac[0], s_own_mac[1], s_own_mac[3], s_own_mac[4],
             (unsigned long)(xTaskGetTickCount() % 100000));

    // Mark as seen so we don't re-process our own flood
    dedup_seen(msg_id);

    ESP_LOGI(TAG, "Starting pathfind: target=%s msg_id=%s",
             target_container_id, msg_id);

    flood_req(msg_id, target_container_id,
              ESPNOW_TTL_DEFAULT, 0, NULL);
}