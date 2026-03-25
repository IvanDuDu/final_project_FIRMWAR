#include "sensor_manager.h"
#include "sensors.h"
#include "actuators.h"
#include "globals.h"
#include "telemetry_engine.h"
#include "sd_manager.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SENS_MGR";

// ─────────────────────────────────────────────
//  GPIO Interrupt Service Routines
// ─────────────────────────────────────────────

static void IRAM_ATTR isr_door_open(void *arg)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(g_sem_door_alert, &hp);
    portYIELD_FROM_ISR(hp);
}

static void IRAM_ATTR isr_collision(void *arg)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(g_sem_collision, &hp);
    portYIELD_FROM_ISR(hp);
}

// ─────────────────────────────────────────────
//  GPIO interrupt config
// ─────────────────────────────────────────────
static esp_err_t gpio_interrupts_init(void)
{
    gpio_config_t door_cfg = {
        .pin_bit_mask = (1ULL << PIN_DOOR_SENSOR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    esp_err_t ret = gpio_config(&door_cfg);
    if (ret != ESP_OK) return ret;

    gpio_config_t mpu_cfg = {
        .pin_bit_mask = (1ULL << PIN_MPU_INT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ret = gpio_config(&mpu_cfg);
    if (ret != ESP_OK) return ret;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    gpio_isr_handler_add(PIN_DOOR_SENSOR, isr_door_open, NULL);
    gpio_isr_handler_add(PIN_MPU_INT,     isr_collision,  NULL);

    ESP_LOGI(TAG, "GPIO interrupts configured (door=GPIO%d, MPU_INT=GPIO%d)",
             PIN_DOOR_SENSOR, PIN_MPU_INT);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Public: init tất cả sensor
//
//  Thứ tự bắt buộc trong main.c:
//    1. sim_manager_hw_init()   ← khởi động UART2 + EC800K
//    2. sensor_manager_init()   ← gps_init() dùng UART2 đã sẵn sàng
// ─────────────────────────────────────────────
esp_err_t sensor_manager_init(void)
{
    esp_err_t ret;

    ret = i2c_bus1_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "I2C Bus1 init failed"); return ret; }
    ret = i2c_bus2_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "I2C Bus2 init failed"); return ret; }

    ds3231_init();
    sht30_init();
    gy906_init();

    mpu6050_init();
    mpu6050_enable_motion_interrupt();

    // GPS dùng UART2 của EC800K — module phải được hw_init trước
    ret = gps_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "GPS init failed — position will read as 0.0/0.0");
    }

    ret = gpio_interrupts_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO interrupt init failed");
        return ret;
    }

    ESP_LOGI(TAG, "Sensor manager initialised");
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Public: đọc tất cả sensor
// ─────────────────────────────────────────────
esp_err_t sensor_manager_read_all(telemetry_record_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(telemetry_record_t));

    bool any_fail = false;

    if (ds3231_get_timestamp(out->timestamp, sizeof(out->timestamp)) != ESP_OK) {
        snprintf(out->timestamp, sizeof(out->timestamp), "00:00:00 01/01/2000 GMT+0");
        any_fail = true;
    }

    if (sht30_read(&out->in_temp, &out->humidity) != ESP_OK) {
        ESP_LOGW(TAG, "SHT30 read failed");
        any_fail = true;
    }

    if (gy906_read_object_temp(&out->sur_temp) != ESP_OK) {
        ESP_LOGW(TAG, "GY-906 read failed");
        any_fail = true;
    }

    esp_err_t gps_ret = gps_read_position(&out->latitude, &out->longitude);
    if (gps_ret != ESP_OK && gps_ret != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "GPS read error: %d", gps_ret);
        any_fail = true;
    } else if (gps_ret == ESP_ERR_TIMEOUT) {
        ESP_LOGD(TAG, "GPS no fix — lat/lon = 0.0");
    }

    if (mpu6050_read_lean(&out->lean) != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 read failed");
        any_fail = true;
    }

    return any_fail ? ESP_FAIL : ESP_OK;
}

// ─────────────────────────────────────────────
//  FreeRTOS Task — đọc sensor + lưu vào SD card
// ─────────────────────────────────────────────
void task_sensor_read(void *arg)
{
    ESP_LOGI(TAG, "Sensor task started (interval=%d min)",
             SENSOR_READ_INTERVAL_MS / 60000);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));

        // ── Đọc cảm biến ──
        telemetry_record_t record = {0};
        sensor_manager_read_all(&record);

        // ── Lấy config dưới mutex ──
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        float humid_threshold = g_config.humid_threshold;
        float weight          = g_config.weight;
        float prev_loss       = g_latest_telemetry.loss;
        xSemaphoreGive(g_mutex_globals);

        // ── Tính loss ──
        float Q0   = (prev_loss > 0.0f) ? (weight - prev_loss) : weight;
        record.loss = telemetry_calc_loss(Q0, record.humidity, humid_threshold, weight);

        // ── Cập nhật g_latest_telemetry ──
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        g_latest_telemetry = record;
        xSemaphoreGive(g_mutex_globals);

        // ── Serialize và ghi vào SD ──
        char json_buf[MAX_JSON_BUF];
        if (telemetry_serialize(&record, json_buf, sizeof(json_buf)) == ESP_OK) {
            esp_err_t sd_ret = sd_telemetry_append(json_buf);
            if (sd_ret == ESP_OK) {
                // Cập nhật counter trong RAM (dùng cho logic gửi batch)    
                xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
                g_telem_count++;
                ESP_LOGI(TAG, "Telemetry record #%d written to SD", g_telem_count);
                xSemaphoreGive(g_mutex_telemetry);
            } else {
                ESP_LOGW(TAG, "SD write failed — record may be lost");
            }
        } else {
            ESP_LOGE(TAG, "Telemetry serialize failed");
        }

        // ── Fan auto-control dựa vào telemetry mới nhất ──
        fan_auto_control(record.sur_temp, record.in_temp,
                         record.humidity, humid_threshold);
    }
}