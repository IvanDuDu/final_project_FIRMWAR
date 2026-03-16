#include "actuators.h"
#include "globals.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ACTUATORS";

// ─────────────────────────────────────────────
//  Initialisation
// ─────────────────────────────────────────────
esp_err_t actuators_init(void)
{
    // ── Fan PWM via LEDC ──────────────────────
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = FAN_LEDC_TIMER,
        .duty_resolution = FAN_LEDC_RESOLUTION,
        .freq_hz         = FAN_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %d", ret);
        return ret;
    }

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = PIN_FAN_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = FAN_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = FAN_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %d", ret);
        return ret;
    }

    // Fans off at startup
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL);

    // ── UV LED GPIO ───────────────────────────
    gpio_config_t uv_cfg = {
        .pin_bit_mask = (1ULL << PIN_UV_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&uv_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UV GPIO config failed: %d", ret);
        return ret;
    }
    gpio_set_level(PIN_UV_LED, 0);  // UV off at startup

    ESP_LOGI(TAG, "Actuators initialised (fan=GPIO%d PWM, uv=GPIO%d)",
             PIN_FAN_PWM, PIN_UV_LED);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Fan control
// ─────────────────────────────────────────────
void fan_set_duty(uint32_t duty)
{
    if (duty > FAN_PWM_MAX) duty = FAN_PWM_MAX;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CHANNEL);
    ESP_LOGD(TAG, "Fan duty set to %lu / %d", duty, FAN_PWM_MAX);
}

void fan_auto_control(float sur_temp, float in_temp,
                      float humidity,  float humid_threshold)
{
    // ── Rule 1: fanEnable gate ────────────────
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    bool fan_en    = g_fan_enable;
    float prev_hum = g_prev_humidity;
    xSemaphoreGive(g_mutex_globals);

    if (!fan_en) {
        fan_set_duty(0);
        ESP_LOGD(TAG, "Fan disabled by operator");
        return;
    }

    float delta = sur_temp - in_temp;
    uint32_t duty = 0;

    // ── Rule 4: humidity still rising → auto-disable fan ─────
    if (prev_hum > 0.0f && humidity > prev_hum) {
        ESP_LOGW(TAG, "Humidity still rising (%.1f%% → %.1f%%), disabling fan",
                 prev_hum, humidity);
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        g_fan_enable = false;
        xSemaphoreGive(g_mutex_globals);
        fan_set_duty(0);
        // Update previous humidity
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        g_prev_humidity = humidity;
        xSemaphoreGive(g_mutex_globals);
        return;
    }

    // ── Rule 2b: temperature differential drives PWM ─────────
    if (delta > 2.0f) {
        float ratio = delta / 5.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        duty = (uint32_t)(ratio * FAN_PWM_MAX);
        ESP_LOGI(TAG, "Fan PWM: delta=%.1f°C → duty=%lu", delta, duty);
    }
    // ── Rule 3: humidity over threshold, no temp delta ───────
    else if (humidity > humid_threshold) {
        duty = (uint32_t)(0.80f * FAN_PWM_MAX);
        ESP_LOGI(TAG, "Fan PWM 80%%: humidity %.1f%% > threshold %.1f%%",
                 humidity, humid_threshold);
    }
    // ── Rule 2a: no trigger → stop ───────────────────────────
    else {
        duty = 0;
        ESP_LOGD(TAG, "Fan off: no trigger condition");
    }

    fan_set_duty(duty);

    // Update previous humidity for next cycle
    xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
    g_prev_humidity = humidity;
    xSemaphoreGive(g_mutex_globals);
}

// ─────────────────────────────────────────────
//  UV LED
// ─────────────────────────────────────────────
void uv_set(bool on)
{
    gpio_set_level(PIN_UV_LED, on ? 1 : 0);
    ESP_LOGI(TAG, "UV LED %s", on ? "ON" : "OFF");
}