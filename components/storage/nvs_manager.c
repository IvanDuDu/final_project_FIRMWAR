#include "nvs_manager.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "NVS";

// ─────────────────────────────────────────────
//  Init
// ─────────────────────────────────────────────
esp_err_t nvs_manager_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased and re-initialised");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) ESP_LOGE(TAG, "NVS init failed: %d", ret);
    return ret;
}

esp_err_t nvs_manager_erase_all(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    ret = nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "NVS namespace '%s' erased", NVS_NAMESPACE);
    return ret;
}

// ─────────────────────────────────────────────
//  Typed helpers
// ─────────────────────────────────────────────
esp_err_t nvs_save_string(const char *key, const char *value)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_str(h, key, value);
    if (ret == ESP_OK) nvs_commit(h);
    nvs_close(h);
    return ret;
}

esp_err_t nvs_load_string(const char *key, char *buf, size_t max_len)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) { buf[0] = '\0'; return ret; }
    size_t len = max_len;
    ret = nvs_get_str(h, key, buf, &len);
    nvs_close(h);
    if (ret != ESP_OK) buf[0] = '\0';
    return ret;
}

esp_err_t nvs_save_bool(const char *key, bool value)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_u8(h, key, value ? 1 : 0);
    if (ret == ESP_OK) nvs_commit(h);
    nvs_close(h);
    return ret;
}

esp_err_t nvs_load_bool(const char *key, bool *out)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) { *out = false; return ret; }
    uint8_t val = 0;
    ret = nvs_get_u8(h, key, &val);
    nvs_close(h);
    *out = (val != 0);
    return ret;
}

esp_err_t nvs_save_float(const char *key, float value)
{
    // Store float as raw 4-byte blob
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_blob(h, key, &value, sizeof(float));
    if (ret == ESP_OK) nvs_commit(h);
    nvs_close(h);
    return ret;
}

esp_err_t nvs_load_float(const char *key, float *out)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) { *out = 0.0f; return ret; }
    size_t len = sizeof(float);
    ret = nvs_get_blob(h, key, out, &len);
    nvs_close(h);
    if (ret != ESP_OK) *out = 0.0f;
    return ret;
}

esp_err_t nvs_save_int32(const char *key, int32_t value)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_i32(h, key, value);
    if (ret == ESP_OK) nvs_commit(h);
    nvs_close(h);
    return ret;
}

esp_err_t nvs_load_int32(const char *key, int32_t *out)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) { *out = 0; return ret; }
    ret = nvs_get_i32(h, key, out);
    nvs_close(h);
    if (ret != ESP_OK) *out = 0;
    return ret;
}

// ─────────────────────────────────────────────
//  Struct helpers — container_config_t
// ─────────────────────────────────────────────
esp_err_t nvs_save_container_config(const container_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    nvs_set_str(h, NVS_KEY_CONTAINER_ID, cfg->container_id);
    nvs_set_str(h, NVS_KEY_CUSTOMER_ID,  cfg->customer_id);
    nvs_set_str(h, NVS_KEY_PROVIDER_ID,  cfg->provider_id);
    nvs_set_blob(h, NVS_KEY_HUMID_THRESH, &cfg->humid_threshold, sizeof(float));
    nvs_set_blob(h, NVS_KEY_WEIGHT,       &cfg->weight,          sizeof(float));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Container config saved (id=%s)", cfg->container_id);
    return ESP_OK;
}

esp_err_t nvs_load_container_config(container_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) return ret;

    size_t len;

    len = MAX_CONTAINER_ID_LEN;
    nvs_get_str(h, NVS_KEY_CONTAINER_ID, cfg->container_id, &len);

    len = MAX_CUSTOMER_ID_LEN;
    nvs_get_str(h, NVS_KEY_CUSTOMER_ID, cfg->customer_id, &len);

    len = MAX_PROVIDER_ID_LEN;
    nvs_get_str(h, NVS_KEY_PROVIDER_ID, cfg->provider_id, &len);

    len = sizeof(float);
    nvs_get_blob(h, NVS_KEY_HUMID_THRESH, &cfg->humid_threshold, &len);

    len = sizeof(float);
    nvs_get_blob(h, NVS_KEY_WEIGHT, &cfg->weight, &len);

    nvs_close(h);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Struct helpers — route_history
// ─────────────────────────────────────────────
esp_err_t nvs_save_route_history(const route_entry_t *history, int count)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    size_t sz = sizeof(route_entry_t) * count;
    ret = nvs_set_blob(h, NVS_KEY_ROUTE_HISTORY, history, sz);
    if (ret == ESP_OK) {
        nvs_set_i32(h, "route_cnt", count);
        nvs_commit(h);
    }
    nvs_close(h);
    return ret;
}

esp_err_t nvs_load_route_history(route_entry_t *history, int *count)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) { *count = 0; return ret; }

    int32_t cnt = 0;
    nvs_get_i32(h, "route_cnt", &cnt);
    *count = cnt;

    if (cnt > 0) {
        size_t sz = sizeof(route_entry_t) * cnt;
        ret = nvs_get_blob(h, NVS_KEY_ROUTE_HISTORY, history, &sz);
    }
    nvs_close(h);
    return ret;
}

