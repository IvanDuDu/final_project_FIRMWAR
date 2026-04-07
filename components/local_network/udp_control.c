#include "udp_control.h"
#include "ble_handler.h"      // reuse ble_handler_process for CMD 03/04
#include "globals.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdlib.h>
#include <errno.h>

static const char *TAG = "UDP_CTRL";


// ─────────────────────────────────────────────
//  Module state
// ─────────────────────────────────────────────
static TaskHandle_t s_task_handle = NULL;
static volatile bool s_running    = false;
static int s_sock_ctrl = -1;
static int s_sock_disc = -1;

// ─────────────────────────────────────────────
//  Get own MAC as ASCII string
// ─────────────────────────────────────────────
static void get_own_mac_str(char *out, size_t len)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ─────────────────────────────────────────────
//  Get own IP as ASCII string (from netif)
// ─────────────────────────────────────────────
static bool get_own_ip_str(char *out, size_t len)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return false;
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(netif, &info) != ESP_OK) return false;
    snprintf(out, len, IPSTR, IP2STR(&info.ip));
    return true;
}

// ─────────────────────────────────────────────
//  Validate: only CMD 03 and CMD 04 allowed over UDP
// ─────────────────────────────────────────────
static bool is_allowed_cmd(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return false;

    cJSON *cmd_item = cJSON_GetObjectItem(root, "command");
    if (!cmd_item || !cJSON_IsString(cmd_item)) {
        cJSON_Delete(root);
        return false;
    }

    bool allowed = (strcmp(cmd_item->valuestring, "03") == 0 ||
                    strcmp(cmd_item->valuestring, "04") == 0);
    cJSON_Delete(root);

    if (!allowed) {
        ESP_LOGW(TAG, "UDP command rejected (only 03/04 allowed over LAN)");
    }
    return allowed;
}

// ─────────────────────────────────────────────
//  Handle DISCOVER packet: if MAC matches, reply with our IP
// ─────────────────────────────────────────────
static void handle_discover(const char *json_str,
                             struct sockaddr_in *sender)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;

    cJSON *type_item = cJSON_GetObjectItem(root, "type");
    cJSON *mac_item  = cJSON_GetObjectItem(root, "mac");

    if (!type_item || !cJSON_IsString(type_item) ||
        strcmp(type_item->valuestring, "discover") != 0 ||
        !mac_item || !cJSON_IsString(mac_item)) {
        cJSON_Delete(root);
        return;
    }

    char own_mac[20];
    get_own_mac_str(own_mac, sizeof(own_mac));

    // Case-insensitive MAC compare
    char req_mac[20];
    strlcpy(req_mac, mac_item->valuestring, sizeof(req_mac));

    // Normalise both to upper
    for (char *p = req_mac; *p; p++) {
        if (*p >= 'a' && *p <= 'f') *p -= 32;
    }
    for (char *p = own_mac; *p; p++) {
        if (*p >= 'a' && *p <= 'f') *p -= 32;
    }

    if (strcmp(req_mac, own_mac) != 0) {
        // Not our MAC — ignore silently
        cJSON_Delete(root);
        return;
    }

    ESP_LOGI(TAG, "DISCOVER matched our MAC — sending OFFER");

    char own_ip[20] = "0.0.0.0";
    get_own_ip_str(own_ip, sizeof(own_ip));

    // Build offer reply
    cJSON *reply = cJSON_CreateObject();
    cJSON_AddStringToObject(reply, "type", "offer");
    cJSON_AddStringToObject(reply, "mac",  own_mac);
    cJSON_AddStringToObject(reply, "ip",   own_ip);
    char *reply_str = cJSON_PrintUnformatted(reply);
    cJSON_Delete(reply);
    cJSON_Delete(root);

    if (!reply_str) return;

    // Send unicast reply to the requester's source port (DISC_PORT)
    struct sockaddr_in dest = *sender;
    dest.sin_port = htons(UDP_DISC_PORT);

    sendto(s_sock_disc, reply_str, strlen(reply_str), 0,
           (struct sockaddr *)&dest, sizeof(dest));
    ESP_LOGI(TAG, "OFFER sent to %s (ip=%s)", req_mac, own_ip);
    free(reply_str);
}

// ─────────────────────────────────────────────
//  UDP receive task
// ─────────────────────────────────────────────
static void task_udp_control(void *arg)
{
    char buf[UDP_BUF_SIZE];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);

    ESP_LOGI(TAG, "UDP control task running (ctrl=%d disc=%d)",
             UDP_CTRL_PORT, UDP_DISC_PORT);

    while (s_running) {
        // Use select() to monitor both sockets with a timeout
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s_sock_ctrl, &rfds);
        FD_SET(s_sock_disc, &rfds);
        int maxfd = (s_sock_ctrl > s_sock_disc) ? s_sock_ctrl : s_sock_disc;

        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        int sel = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        if (sel < 0) {
            if (errno == EINTR) continue;
            ESP_LOGE(TAG, "select() error: %d", errno);
            break;
        }
        if (sel == 0) continue;   // timeout — loop back to check s_running

        // ── Control socket (CMD 03 / CMD 04) ────────────────────────────────
        if (FD_ISSET(s_sock_ctrl, &rfds)) {
            int n = recvfrom(s_sock_ctrl, buf, sizeof(buf) - 1, 0,
                             (struct sockaddr *)&sender, &sender_len);
            if (n > 0) {
                buf[n] = '\0';
                ESP_LOGI(TAG, "UDP CMD recv: %s", buf);

                if (is_allowed_cmd(buf)) {
                    // Reuse BLE handler — same JSON format, same logic
                    ble_handler_process(buf);
                }
            }
        }

        // ── Discovery socket ─────────────────────────────────────────────────
        if (FD_ISSET(s_sock_disc, &rfds)) {
            int n = recvfrom(s_sock_disc, buf, sizeof(buf) - 1, 0,
                             (struct sockaddr *)&sender, &sender_len);
            if (n > 0) {
                buf[n] = '\0';
                handle_discover(buf, &sender);
            }
        }
    }

    // Cleanup sockets
    if (s_sock_ctrl >= 0) { close(s_sock_ctrl); s_sock_ctrl = -1; }
    if (s_sock_disc >= 0) { close(s_sock_disc); s_sock_disc = -1; }

    ESP_LOGI(TAG, "UDP control task stopped");
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────
//  Helper: open a bound UDP socket on given port (enable broadcast)
// ─────────────────────────────────────────────
static int open_udp_socket(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        return -1;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() port %d failed: %d", port, errno);
        close(sock);
        return -1;
    }

    // Non-blocking is handled by select() — keep blocking mode for simplicity
    return sock;
}

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────
esp_err_t udp_control_start(void)
{
    if (s_running) {
        ESP_LOGW(TAG, "UDP control already running");
        return ESP_OK;
    }

    s_sock_ctrl = open_udp_socket(UDP_CTRL_PORT);
    if (s_sock_ctrl < 0) return ESP_FAIL;

    s_sock_disc = open_udp_socket(UDP_DISC_PORT);
    if (s_sock_disc < 0) {
        close(s_sock_ctrl);
        s_sock_ctrl = -1;
        return ESP_FAIL;
    }

    s_running = true;

    BaseType_t rc = xTaskCreatePinnedToCore(
        task_udp_control, "udp_ctrl",
        4096, NULL, 3, &s_task_handle, 0);

    if (rc != pdPASS) {
        s_running = false;
        close(s_sock_ctrl); s_sock_ctrl = -1;
        close(s_sock_disc); s_sock_disc = -1;
        ESP_LOGE(TAG, "Task creation failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP control started (CTRL:%d DISC:%d)",
             UDP_CTRL_PORT, UDP_DISC_PORT);
    return ESP_OK;
}

void udp_control_stop(void)
{
    if (!s_running) return;
    s_running = false;
    // Task will exit on next select() timeout (≤1s) and close sockets
    // Give it time to clean up
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "UDP control stopped");
}

bool udp_control_is_running(void)
{
    return s_running;
}