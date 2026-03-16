#include "sim_manager.h"
#include "globals.h"

#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "SIM_MGR";

// ─────────────────────────────────────────────
//  UART2 config  (TX/RX auto-assigned by board)
// ─────────────────────────────────────────────
#define SIM_UART_PORT   UART_NUM_2
#define SIM_UART_TX     17
#define SIM_UART_RX     18
#define SIM_UART_BAUD   115200
#define SIM_BUF_SIZE    512
#define SIM_CMD_TIMEOUT 5000   // ms per AT command

static bool s_connected = false;

// ─────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────
static void sim_uart_init(void)
{
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
    uart_driver_install(SIM_UART_PORT, SIM_BUF_SIZE * 2, 0, 0, NULL, 0);
}

/**
 * Send AT command and wait for expected response substring.
 * Returns ESP_OK if response contains expected token within timeout.
 */
static esp_err_t sim_send_cmd(const char *cmd, const char *expect, int timeout_ms)
{
    // Flush RX
    uart_flush_input(SIM_UART_PORT);

    uart_write_bytes(SIM_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(SIM_UART_PORT, "\r\n", 2);
    ESP_LOGD(TAG, ">> %s", cmd);

    char buf[SIM_BUF_SIZE] = {0};
    int  total   = 0;
    int  elapsed = 0;
    int  step    = 100;

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
                return ESP_OK;
            }
            if (strstr(buf, "ERROR")) {
                ESP_LOGW(TAG, "AT ERROR: %s", buf);
                return ESP_FAIL;
            }
        }
        elapsed += step;
    }
    ESP_LOGW(TAG, "Timeout waiting for '%s'. Got: %s", expect, buf);
    return ESP_ERR_TIMEOUT;
}

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────
esp_err_t sim_manager_connect(void)
{
    sim_uart_init();

    ESP_LOGI(TAG, "Starting SIM AT handshake...");

    // Basic sanity check
    if (sim_send_cmd("AT", "OK", SIM_CMD_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "Module not responding");
        return ESP_FAIL;
    }

    // Disable echo
    sim_send_cmd("ATE0", "OK", SIM_CMD_TIMEOUT);

    // Check SIM card ready
    if (sim_send_cmd("AT+CPIN?", "READY", SIM_CMD_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "SIM card not ready");
        return ESP_FAIL;
    }

    // Wait for network registration (CS or PS)
    bool registered = false;
    for (int i = 0; i < 10; i++) {
        if (sim_send_cmd("AT+CEREG?", "+CEREG: 0,1", SIM_CMD_TIMEOUT) == ESP_OK ||
            sim_send_cmd("AT+CEREG?", "+CEREG: 0,5", SIM_CMD_TIMEOUT) == ESP_OK) {
            registered = true;
            break;
        }
        ESP_LOGD(TAG, "Waiting for 5G registration... (%d/10)", i + 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    if (!registered) {
        ESP_LOGE(TAG, "5G network registration failed");
        return ESP_FAIL;
    }

    // Activate PDP context (generic APN — adjust per carrier)
    sim_send_cmd("AT+CGDCONT=1,\"IP\",\"internet\"", "OK", SIM_CMD_TIMEOUT);

    // Attach GPRS/data
    if (sim_send_cmd("AT+CGACT=1,1", "OK", 10000) != ESP_OK) {
        ESP_LOGE(TAG, "PDP context activation failed");
        return ESP_FAIL;
    }

    s_connected = true;
    xEventGroupSetBits(g_evt_network, NET_EVT_SIM_READY);
    ESP_LOGI(TAG, "5G data connected");
    return ESP_OK;
}

void sim_manager_disconnect(void)
{
    sim_send_cmd("AT+CGACT=0,1", "OK", SIM_CMD_TIMEOUT);
    s_connected = false;
    xEventGroupClearBits(g_evt_network, NET_EVT_SIM_READY);
    ESP_LOGI(TAG, "SIM data disconnected");
}

bool sim_manager_is_connected(void)
{
    return s_connected;
}