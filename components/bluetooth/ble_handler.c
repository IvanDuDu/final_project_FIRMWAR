#include "ble_handler.h"
#include "ble_manager.h"
#include "globals.h"

#include "esp_log.h"
#include "cJSON.h"
#include "nvs_manager.h"        // storage component
#include "mqtt_client_wrap.h"   // network component
#include "espnow_manager.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "BLE_HDL";

// ─────────────────────────────────────────────
//  Forward declarations (one per command)
// ─────────────────────────────────────────────
static void handle_cmd_00(cJSON *des);   // Provider setup
static void handle_cmd_01(cJSON *des);   // Shipper take-charge
static void handle_cmd_02(cJSON *des);   // Customer confirm received
static void handle_cmd_03(cJSON *des);   // Shipper toggle fan
static void handle_cmd_04(cJSON *des);   // Shipper toggle UV
static void handle_cmd_05(cJSON *des); // Shipper pathfind
static void handle_cmd_08(cJSON *des);   // Shipper set WiFi credentials

// ─────────────────────────────────────────────
//  Entry point
// ─────────────────────────────────────────────
void ble_handler_process(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGW(TAG, "JSON parse failed");
        return;
    }

    cJSON *cmd_item = cJSON_GetObjectItem(root, "command");
    cJSON *des_item = cJSON_GetObjectItem(root, "des");

    if (!cmd_item || !cJSON_IsString(cmd_item)) {
        ESP_LOGW(TAG, "Missing 'command' field");
        cJSON_Delete(root);
        return;
    }

    const char *cmd = cmd_item->valuestring;
    ESP_LOGI(TAG, "Received command: %s", cmd);

    if      (strcmp(cmd, "00") == 0) handle_cmd_00(des_item);
    else if (strcmp(cmd, "01") == 0) handle_cmd_01(des_item);
    else if (strcmp(cmd, "02") == 0) handle_cmd_02(des_item);
    else if (strcmp(cmd, "03") == 0) handle_cmd_03(des_item);
    else if (strcmp(cmd, "04") == 0) handle_cmd_04(des_item);
    else if (strcmp(cmd, "08") == 0) handle_cmd_08(des_item);
    else if (strcmp(cmd, "05") == 0) handle_cmd_05(des_item);
    else ESP_LOGW(TAG, "Unknown command: %s", cmd);

    cJSON_Delete(root);
}

// ─────────────────────────────────────────────
//  CMD 00 — Provider setup
// ─────────────────────────────────────────────
static void handle_cmd_00(cJSON *des)
{
    if (!des) { ESP_LOGW(TAG, "CMD00: missing des"); return; }

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);

    cJSON *item;
    #define COPY_STR(field, dest, maxlen) \
        item = cJSON_GetObjectItem(des, field); \
        if (item && cJSON_IsString(item)) \
            strlcpy(dest, item->valuestring, maxlen);

    COPY_STR("containerId",   g_config.container_id,    MAX_CONTAINER_ID_LEN)
    COPY_STR("customerId",    g_config.customer_id,     MAX_CUSTOMER_ID_LEN)
    COPY_STR("providerId",    g_config.provider_id,     MAX_PROVIDER_ID_LEN)
    COPY_STR("typeOfProduct", g_config.type_of_product, MAX_PRODUCT_TYPE_LEN)
    COPY_STR("from",          g_config.from,            MAX_LOCATION_LEN)
    COPY_STR("to",            g_config.to,              MAX_LOCATION_LEN)
    COPY_STR("relaseDate",    g_config.release_date,    32)

    item = cJSON_GetObjectItem(des, "humidThreshold");
    if (item && cJSON_IsNumber(item)) g_config.humid_threshold = (float)item->valuedouble;

    item = cJSON_GetObjectItem(des, "weight");
    if (item && cJSON_IsNumber(item)) g_config.weight = (float)item->valuedouble;

    #undef COPY_STR

    xSemaphoreGive(g_mutex_globals);

    // Persist to NVS
    nvs_save_container_config(&g_config);

    ESP_LOGI(TAG, "CMD00: container=%s customer=%s",
             g_config.container_id, g_config.customer_id);

    // Signal stage machine that setup data is ready
    // (stage1 task polls g_config.container_id being non-empty)
}

// ─────────────────────────────────────────────
//  CMD 01 — Shipper take-charge
// ─────────────────────────────────────────────
static void handle_cmd_01(cJSON *des)
{
    if (!des) { ESP_LOGW(TAG, "CMD01: missing des"); return; }

    route_entry_t entry = {0};

    cJSON *item = cJSON_GetObjectItem(des, "shipperId");
    if (item && cJSON_IsString(item))
        strlcpy(entry.shipper_id, item->valuestring, MAX_SHIPPER_ID_LEN);

    item = cJSON_GetObjectItem(des, "isOnBoard");
    if (item && cJSON_IsBool(item))
        entry.is_on_board = cJSON_IsTrue(item);

    item = cJSON_GetObjectItem(des, "timestamp");
    if (item && cJSON_IsString(item))
        strlcpy(entry.timestamp, item->valuestring, 32);

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);

    if (g_route_count < MAX_ROUTE_HISTORY) {
        g_route_history[g_route_count++] = entry;
    } else {
        // Shift left, discard oldest
        memmove(&g_route_history[0], &g_route_history[1],
                sizeof(route_entry_t) * (MAX_ROUTE_HISTORY - 1));
        g_route_history[MAX_ROUTE_HISTORY - 1] = entry;
    }

    bool on_board = entry.is_on_board;
    xSemaphoreGive(g_mutex_globals);

    // Persist route history
    nvs_save_route_history(g_route_history, g_route_count);

    ESP_LOGI(TAG, "CMD01: shipper=%s isOnBoard=%d", entry.shipper_id, on_board);

    // Notify network manager of possible transport mode change
    // (network_manager will re-evaluate callback based on latest routeHistory)
    xEventGroupSetBits(g_evt_network, on_board ? NET_EVT_WIFI_READY : NET_EVT_SIM_READY);

    // Publish switchShipper topic
    char json[256];
    snprintf(json, sizeof(json),
             "{\"shipperId\":\"%s\",\"isOnBoard\":%s,\"timestamp\":\"%s\"}",
             entry.shipper_id, on_board ? "true" : "false", entry.timestamp);
    char topic[80];
    snprintf(topic, sizeof(topic), "device/%s/switchShipper", g_config.container_id);
    // missing checking connection
    mqtt_publish(topic, json, 1, 0);
}

// ─────────────────────────────────────────────
//  CMD 02 — Customer confirm received
// ─────────────────────────────────────────────
static void handle_cmd_02(cJSON *des)
{
    if (!des) { ESP_LOGW(TAG, "CMD02: missing des"); return; }

    cJSON *item = cJSON_GetObjectItem(des, "customerId");
    if (!item || !cJSON_IsString(item)) {
        ESP_LOGW(TAG, "CMD02: missing customerId");
        return;
    }

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    bool match = (strcmp(item->valuestring, g_config.customer_id) == 0);
    xSemaphoreGive(g_mutex_globals);

    if (!match) {
        ESP_LOGW(TAG, "CMD02: customerID mismatch");
        return;
    }

    // Disable door alert
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_door_enable = false;
    xSemaphoreGive(g_mutex_globals);

    // Notify stage machine to proceed to STAGE_RECEIVE
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_stage = STAGE_RECEIVE;
    xSemaphoreGive(g_mutex_globals);

    ESP_LOGI(TAG, "CMD02: customer verified, transitioning to STAGE_RECEIVE");
}

// ─────────────────────────────────────────────
//  CMD 03 — Shipper toggle fan
// ─────────────────────────────────────────────
static void handle_cmd_03(cJSON *des)
{
    if (!des) { ESP_LOGW(TAG, "CMD03: missing des"); return; }

    cJSON *item = cJSON_GetObjectItem(des, "value");
    if (!item || !cJSON_IsNumber(item)) return;

    bool new_val = (item->valueint == 1);

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_fan_enable = new_val;
    xSemaphoreGive(g_mutex_globals);

    nvs_save_bool(NVS_KEY_FAN_ENABLE, new_val);
    ESP_LOGI(TAG, "CMD03: fanEnable=%d", new_val);
}

// ─────────────────────────────────────────────
//  CMD 04 — Shipper toggle UV
// ─────────────────────────────────────────────
static void handle_cmd_04(cJSON *des)
{
    if (!des) { ESP_LOGW(TAG, "CMD04: missing des"); return; }

    cJSON *item = cJSON_GetObjectItem(des, "value");
    if (!item || !cJSON_IsNumber(item)) return;

    bool new_val = (item->valueint == 1);

    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_uv_enable = new_val;
    xSemaphoreGive(g_mutex_globals);

    nvs_save_bool(NVS_KEY_UV_ENABLE, new_val);

    // Directly drive GPIO
    extern void uv_set(bool on);
    uv_set(new_val);

    ESP_LOGI(TAG, "CMD04: uvEnable=%d", new_val);
}

// ─────────────────────────────────────────────
//  CMD 05 — Shipper pathfind request
// ─────────────────────────────────────────────
static void handle_cmd_05(cJSON *des)
{
    if (!des) { ESP_LOGW(TAG, "CMD05: missing des"); return; }
 
    cJSON *item = cJSON_GetObjectItem(des, "targetId");
    if (!item || !cJSON_IsString(item)) {
        ESP_LOGW(TAG, "CMD05: missing targetId");
        return;
    }
 
    const char *target = item->valuestring;
    ESP_LOGI(TAG, "CMD05: pathfind → %s", target);
 
    // Kick off ESP-NOW flood from this node
    espnow_pathfind_start(target);
}

// ─────────────────────────────────────────────
//  CMD 08 — Shipper save WiFi credentials
// ─────────────────────────────────────────────
static void handle_cmd_08(cJSON *des)
{
    if (!des) { ESP_LOGW(TAG, "CMD08: missing des"); return; }

    char ssid[MAX_SSID_LEN] = {0};
    char pass[MAX_PASS_LEN] = {0};

    cJSON *item = cJSON_GetObjectItem(des, "ssid");
    if (item && cJSON_IsString(item)) strlcpy(ssid, item->valuestring, MAX_SSID_LEN);

    item = cJSON_GetObjectItem(des, "password");
    if (item && cJSON_IsString(item)) strlcpy(pass, item->valuestring, MAX_PASS_LEN);

    // Overwrite NVS each time
    nvs_save_string(NVS_KEY_WIFI_SSID, ssid);
    nvs_save_string(NVS_KEY_WIFI_PASS, pass);

    ESP_LOGI(TAG, "CMD08: WiFi credentials saved, ssid=%s", ssid);

    // Signal stage machine that WiFi credentials are ready
    xEventGroupSetBits(g_evt_network, NET_EVT_WIFI_READY);
}