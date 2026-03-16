#include "sensor_manager.h"
#include "sensors.h"
#include "globals.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SENS_MGR";

// ─────────────────────────────────────────────
//  GPIO Interrupt Service Routines
// ─────────────────────────────────────────────

/* Door sensor ISR: triggered on falling edge (HIGH→LOW = door opened) */
static void IRAM_ATTR isr_door_open(void *arg)
{
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_sem_door_alert, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
}

/* MPU6050 INT ISR: triggered on rising edge (collision detected) */
static void IRAM_ATTR isr_collision(void *arg)
{
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_sem_collision, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
}

// ─────────────────────────────────────────────
//  GPIO configuration for interrupt pins
// ─────────────────────────────────────────────
static esp_err_t gpio_interrupts_init(void)
{
    // Door sensor: active-LOW when door is open
    gpio_config_t door_cfg = {
        .pin_bit_mask = (1ULL << PIN_DOOR_SENSOR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,  // trigger on falling edge
    };
    esp_err_t ret = gpio_config(&door_cfg);
    if (ret != ESP_OK) return ret;

    // MPU6050 INT: active-HIGH on motion detection
    gpio_config_t mpu_cfg = {
        .pin_bit_mask = (1ULL << PIN_MPU_INT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,  // trigger on rising edge
    };
    ret = gpio_config(&mpu_cfg);
    if (ret != ESP_OK) return ret;

    // Install shared GPIO ISR service (if not already installed)
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    gpio_isr_handler_add(PIN_DOOR_SENSOR, isr_door_open, NULL);
    gpio_isr_handler_add(PIN_MPU_INT,     isr_collision,  NULL);

    ESP_LOGI(TAG, "GPIO interrupts configured (door=GPIO%d, MPU_INT=GPIO%d)",
             PIN_DOOR_SENSOR, PIN_MPU_INT);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Public: init all sensors
// ─────────────────────────────────────────────
esp_err_t sensor_manager_init(void)
{
    esp_err_t ret;

    // I2C buses
    ret = i2c_bus1_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Bus1 init failed");
        return ret;
    }
    ret = i2c_bus2_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Bus2 init failed");
        return ret;
    }

    // Sensors on Bus1 (sequential)
    ret  = ds3231_init();
    ret |= sht30_init();
    ret |= gy906_init();
    ret |= gps_init();

    // MPU6050 on Bus2
    ret |= mpu6050_init();
    ret |= mpu6050_enable_motion_interrupt();

    // GPIO interrupt lines
    ret |= gpio_interrupts_init();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "One or more sensors failed to initialise");
    }
    return ret;
}

// ─────────────────────────────────────────────
//  Public: read all sensors into one record
// ─────────────────────────────────────────────
esp_err_t sensor_manager_read_all(telemetry_record_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(telemetry_record_t));

    bool any_fail = false;
    esp_err_t ret;

    // 1. Timestamp from RTC (Bus1 — first read avoids blocking GPS/SHT30)
    ret = ds3231_get_timestamp(out->timestamp, sizeof(out->timestamp));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DS3231 read failed, using empty timestamp");
        snprintf(out->timestamp, sizeof(out->timestamp), "00:00:00 01/01/2000 GMT+0");
        any_fail = true;
    }

    // 2. Internal temperature & humidity (Bus1)
    ret = sht30_read(&out->in_temp, &out->humidity);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SHT30 read failed");
        out->in_temp  = 0.0f;
        out->humidity = 0.0f;
        any_fail = true;
    }

    // 3. Surface temperature (Bus1)
    ret = gy906_read_object_temp(&out->sur_temp);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "GY-906 read failed");
        out->sur_temp = 0.0f;
        any_fail = true;
    }

    // 4. GPS position (Bus1 — returns defaults if module absent)
    ret = gps_read_position(&out->latitude, &out->longitude);
    if (ret != ESP_OK) {
        out->latitude  = 0.0;
        out->longitude = 0.0;
    }

    // 5. MPU6050 tilt (Bus2 — independent of Bus1 devices)
    ret = mpu6050_read_lean(&out->lean);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 read failed");
        out->lean = 0.0f;
        any_fail = true;
    }

    return any_fail ? ESP_FAIL : ESP_OK;
}

// ─────────────────────────────────────────────
//  FreeRTOS Task
// ─────────────────────────────────────────────
void task_sensor_read(void *arg)
{
    ESP_LOGI(TAG, "Sensor task started (interval=%d min)",
             SENSOR_READ_INTERVAL_MS / 60000);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        telemetry_record_t record = {0};
        esp_err_t ret = sensor_manager_read_all(&record);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Partial sensor read — record still stored");
        }

        // Update g_latest_telemetry (used by BLE notify, fan control)
        xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
        g_latest_telemetry = record;
        xSemaphoreGive(g_mutex_telemetry);

        // Append to send buffer
        xSemaphoreTake(g_mutex_telemetry, portMAX_DELAY);
        if (g_telem_count < MAX_TELEMETRY_BUFFER) {
            g_send_telemetry[g_telem_count] = record;
            g_telem_count++;
            ESP_LOGI(TAG, "Telemetry stored [%d/%d]", g_telem_count, MAX_TELEMETRY_BUFFER);
        } else {
            ESP_LOGW(TAG, "Telemetry buffer full — dropping oldest record");
            // Shift buffer left
            memmove(&g_send_telemetry[0], &g_send_telemetry[1],
                    sizeof(telemetry_record_t) * (MAX_TELEMETRY_BUFFER - 1));
            g_send_telemetry[MAX_TELEMETRY_BUFFER - 1] = record;
        }
        xSemaphoreGive(g_mutex_telemetry);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}