#pragma once

#include "esp_err.h"
#include "globals.h"
#include <stdbool.h>

/** Must be called once at startup before any other NVS operation. */
esp_err_t nvs_manager_init(void);

/** Wipe entire container namespace (called at Stage 3 completion). */
esp_err_t nvs_manager_erase_all(void);

// ─── Typed helpers ───────────────────────────
esp_err_t nvs_save_string(const char *key, const char *value);
esp_err_t nvs_load_string(const char *key, char *buf, size_t max_len);

esp_err_t nvs_save_bool(const char *key, bool value);
esp_err_t nvs_load_bool(const char *key, bool *out);

esp_err_t nvs_save_float(const char *key, float value);
esp_err_t nvs_load_float(const char *key, float *out);

esp_err_t nvs_save_int32(const char *key, int32_t value);
esp_err_t nvs_load_int32(const char *key, int32_t *out);

// ─── Struct helpers ──────────────────────────
esp_err_t nvs_save_container_config(const container_config_t *cfg);
esp_err_t nvs_load_container_config(container_config_t *cfg);

esp_err_t nvs_save_route_history(const route_entry_t *history, int count);
esp_err_t nvs_load_route_history(route_entry_t *history, int *count);

