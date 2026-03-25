#include "sim_manager.h"
#include "globals.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "SIM_MGR";

// ─────────────────────────────────────────────
//  Cấu hình UART2
// ─────────────────────────────────────────────
#define SIM_UART_PORT           UART_NUM_2
#define SIM_UART_TX             17
#define SIM_UART_RX             18
#define SIM_UART_BAUD           115200
#define SIM_BUF_SIZE            512
#define SIM_CMD_TIMEOUT         5000    // ms mặc định per AT command
#define SIM_LONG_TIMEOUT        15000   // ms cho các lệnh chờ lâu (PDP)
#define SIM_INIT_MAX_RETRIES    5       // số lần retry khi hw_init
#define SIM_INIT_RETRY_DELAY_MS 3000    // delay giữa các lần retry
#define SIM_MONITOR_INTERVAL_MS 30000   // chu kỳ kiểm tra module

// ─────────────────────────────────────────────
//  State
// ─────────────────────────────────────────────
static bool s_uart_ready    = false;   // UART đã được cài đặt
static bool s_hw_ready      = false;   // Module AT phản hồi và GNSS bật
static bool s_data_connected = false;  // PDP bearer đang active

// ─────────────────────────────────────────────
//  UART init (idempotent)
// ─────────────────────────────────────────────
static void sim_uart_init(void)
{
    if (s_uart_ready) return;

    uart_config_t cfg = {
        .baud_rate  = SIM_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(SIM_UART_PORT, &cfg);
    uart_set_pin(SIM_UART_PORT, SIM_UART_TX, SIM_UART_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    esp_err_t ret = uart_driver_install(SIM_UART_PORT,
                                        SIM_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "UART2 already installed");
    }
    s_uart_ready = true;
    ESP_LOGI(TAG, "UART2 ready (TX=%d RX=%d baud=%d)",
             SIM_UART_TX, SIM_UART_RX, SIM_UART_BAUD);
}

// ─────────────────────────────────────────────
//  AT command helper
// ─────────────────────────────────────────────
static esp_err_t at_cmd(const char *cmd, const char *expect,
                         char *resp_buf, size_t resp_len, int timeout_ms)
{
    uart_flush_input(SIM_UART_PORT);

    uart_write_bytes(SIM_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(SIM_UART_PORT, "\r\n", 2);
    ESP_LOGD(TAG, ">> %s", cmd);

    if (!expect) return ESP_OK;

    char   buf[SIM_BUF_SIZE] = {0};
    int    total   = 0;
    int    elapsed = 0;
    const int step = 100;

    while (elapsed < timeout_ms) {
        int n = uart_read_bytes(SIM_UART_PORT,
                                (uint8_t *)(buf + total),
                                SIM_BUF_SIZE - total - 1,
                                pdMS_TO_TICKS(step));
        if (n > 0) {
            total += n;
            buf[total] = '\0';

            if (strstr(buf, expect)) {
                ESP_LOGD(TAG, "<< %s", buf);
                if (resp_buf && resp_len > 0)
                    strlcpy(resp_buf, buf, resp_len);
                return ESP_OK;
            }
            // Một số lệnh trả về CME ERROR mà không phải "ERROR" thuần
            if (strstr(buf, "ERROR")) {
                ESP_LOGW(TAG, "AT error for '%s': %s", cmd, buf);
                if (resp_buf && resp_len > 0)
                    strlcpy(resp_buf, buf, resp_len);
                return ESP_FAIL;
            }
        }
        elapsed += step;
    }
    ESP_LOGW(TAG, "Timeout '%s' (expected '%s'). Got: %s", cmd, expect, buf);
    if (resp_buf && resp_len > 0)
        strlcpy(resp_buf, buf, resp_len);
    return ESP_ERR_TIMEOUT;
}

// ─────────────────────────────────────────────
//  Internal: bật GNSS engine
// ─────────────────────────────────────────────
static esp_err_t gnss_start(void)
{
    // Tắt NMEA auto-output để tránh rác trong UART buffer
    at_cmd("AT+QGPSCFG=\"outport\",\"none\"", "OK", NULL, 0, 2000);

    // Cho phép đọc NMEA qua AT+QGPSGNMEA
    at_cmd("AT+QGPSCFG=\"nmeasrc\",1", "OK", NULL, 0, 2000);

    // Bật GNSS engine
    esp_err_t ret = at_cmd("AT+QGPS=1", "OK", NULL, 0, 3000);
    if (ret != ESP_OK) {
        // Kiểm tra xem đã bật chưa
        char resp[SIM_BUF_SIZE];
        at_cmd("AT+QGPS?", "+QGPS:", resp, sizeof(resp), 2000);
        if (strstr(resp, "+QGPS: 1")) {
            ESP_LOGI(TAG, "GNSS engine already running");
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Cannot start GNSS engine");
        return ret;
    }

    ESP_LOGI(TAG, "GNSS engine started");
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Phase 1: HW init — luôn gọi khi boot
// ─────────────────────────────────────────────
esp_err_t sim_manager_hw_init(void)
{
    sim_uart_init();

    ESP_LOGI(TAG, "EC800K hardware init (max %d retries)...", SIM_INIT_MAX_RETRIES);

    for (int attempt = 1; attempt <= SIM_INIT_MAX_RETRIES; attempt++) {
        ESP_LOGI(TAG, "AT ping attempt %d/%d", attempt, SIM_INIT_MAX_RETRIES);

        if (at_cmd("AT", "OK", NULL, 0, SIM_CMD_TIMEOUT) == ESP_OK) {
            // Module phản hồi
            at_cmd("ATE0", "OK", NULL, 0, SIM_CMD_TIMEOUT);   // tắt echo

            // Bật GNSS ngay lập tức — không cần chờ mạng
            gnss_start();

            s_hw_ready = true;
            ESP_LOGI(TAG, "EC800K ready (attempt %d)", attempt);
            return ESP_OK;
        }

        if (attempt < SIM_INIT_MAX_RETRIES) {
            ESP_LOGW(TAG, "Module not responding, retry in %d ms", SIM_INIT_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(SIM_INIT_RETRY_DELAY_MS));
        }
    }

    ESP_LOGE(TAG, "EC800K did not respond after %d attempts", SIM_INIT_MAX_RETRIES);
    s_hw_ready = false;
    return ESP_FAIL;
}

// ─────────────────────────────────────────────
//  Phase 2: Data connect — chỉ gọi khi cần 4G
// ─────────────────────────────────────────────
esp_err_t sim_manager_data_connect(void)
{
    if (!s_hw_ready) {
        ESP_LOGW(TAG, "Module not ready, attempting hw_init first");
        esp_err_t ret = sim_manager_hw_init();
        if (ret != ESP_OK) return ret;
    }

    if (s_data_connected) {
        ESP_LOGW(TAG, "Data already connected");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Connecting 4G data bearer...");

    // Kiểm tra SIM
    if (at_cmd("AT+CPIN?", "READY", NULL, 0, SIM_CMD_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "SIM card not ready");
        return ESP_FAIL;
    }

    // Chờ đăng ký mạng — tối đa 10 lần × 2s = 20s
    bool registered = false;
    for (int i = 0; i < 10 && !registered; i++) {
        char resp[SIM_BUF_SIZE];
        // Thử cả CREG (2G/3G) và CEREG (4G/LTE)
        if (at_cmd("AT+CEREG?", "+CEREG: 0,1", resp, sizeof(resp), SIM_CMD_TIMEOUT) == ESP_OK ||
            at_cmd("AT+CEREG?", "+CEREG: 0,5", resp, sizeof(resp), SIM_CMD_TIMEOUT) == ESP_OK ||
            at_cmd("AT+CREG?",  "+CREG: 0,1",  resp, sizeof(resp), SIM_CMD_TIMEOUT) == ESP_OK) {
            registered = true;
        } else {
            ESP_LOGD(TAG, "Waiting for network registration... (%d/10)", i + 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    if (!registered) {
        ESP_LOGE(TAG, "Network registration timeout");
        return ESP_FAIL;
    }

    // Cấu hình APN và bật PDP
    at_cmd("AT+CGDCONT=1,\"IP\",\"internet\"", "OK", NULL, 0, SIM_CMD_TIMEOUT);

    if (at_cmd("AT+CGACT=1,1", "OK", NULL, 0, SIM_LONG_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "PDP activation failed");
        return ESP_FAIL;
    }

    s_data_connected = true;
    xEventGroupSetBits(g_evt_network, NET_EVT_SIM_READY);
    ESP_LOGI(TAG, "4G data connected");
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Data disconnect — giữ GNSS chạy
// ─────────────────────────────────────────────
void sim_manager_data_disconnect(void)
{
    if (!s_data_connected) return;

    at_cmd("AT+CGACT=0,1", "OK", NULL, 0, SIM_CMD_TIMEOUT);
    s_data_connected = false;
    xEventGroupClearBits(g_evt_network, NET_EVT_SIM_READY);
    ESP_LOGI(TAG, "4G data disconnected (GNSS still running)");
}

// ─────────────────────────────────────────────
//  Backward-compat wrappers
// ─────────────────────────────────────────────
esp_err_t sim_manager_connect(void)   { return sim_manager_data_connect(); }
void      sim_manager_disconnect(void){ sim_manager_data_disconnect(); }
bool      sim_manager_is_connected(void) { return s_data_connected; }

// ─────────────────────────────────────────────
//  Background monitor task
// ─────────────────────────────────────────────
void task_sim_monitor(void *arg)
{
    ESP_LOGI(TAG, "SIM monitor task started (interval=%d s)",
             SIM_MONITOR_INTERVAL_MS / 1000);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SIM_MONITOR_INTERVAL_MS));

        if (!s_uart_ready) continue;

        // ── Kiểm tra module còn sống không ──
        esp_err_t alive = at_cmd("AT", "OK", NULL, 0, 3000);
        if (alive != ESP_OK) {
            ESP_LOGW(TAG, "Module unresponsive — attempting re-init");
            s_hw_ready       = false;
            s_data_connected = false;
            xEventGroupClearBits(g_evt_network, NET_EVT_SIM_READY);

            // Re-init
            if (sim_manager_hw_init() != ESP_OK) {
                ESP_LOGE(TAG, "Re-init failed, will retry next cycle");
                continue;
            }
        }

        // ── Nếu đang ở land mode mà data bị drop, reconnect ──
        xSemaphoreTake(g_mutex_globals, portMAX_DELAY);
        bool on_board = (g_route_count > 0)
                        ? g_route_history[g_route_count - 1].is_on_board
                        : true;   // mặc định: coi như on_board để không tự kết nối
        xSemaphoreGive(g_mutex_globals);

        if (!on_board && s_hw_ready && !s_data_connected) {
            ESP_LOGW(TAG, "Land mode but data disconnected — reconnecting");
            sim_manager_data_connect();
        }

        // ── Đảm bảo GNSS vẫn chạy ──
        if (s_hw_ready) {
            char resp[SIM_BUF_SIZE];
            esp_err_t gps_status = at_cmd("AT+QGPS?", "+QGPS: 1",
                                           resp, sizeof(resp), 2000);
            if (gps_status != ESP_OK) {
                ESP_LOGW(TAG, "GNSS not running — restarting");
                gnss_start();
            }
        }
    }
}