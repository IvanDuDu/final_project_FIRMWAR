#include "ble_manager.h"
#include "ble_handler.h"
#include "ble_notify.h"
#include "globals.h"

#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <string.h>

static const char *TAG = "BLE_MGR";

// ─────────────────────────────────────────────
//  UUIDs  (128-bit, little-endian byte arrays)
// ─────────────────────────────────────────────
// Service:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E  (Nordic UART-like)
// RX char:  6E400002-...  (Write – central → device)
// TX char:  6E400003-...  (Notify – device → central)

#define BLE_SVC_UUID128  0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,\
                         0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
#define BLE_RX_UUID128   0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,\
                         0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E
#define BLE_TX_UUID128   0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,\
                         0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E

static uint8_t s_own_addr_type;
static uint16_t s_conn_handle        = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_char_val_handle = 0;
static bool     s_notify_enabled     = false;

// ─────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────
static void ble_advertise(void);

// ─────────────────────────────────────────────
//  GATT characteristic access callbacks
// ─────────────────────────────────────────────

/* RX characteristic — central writes JSON command packets here */
static int ble_rx_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0 || len >= MAX_JSON_BUF) {
        ESP_LOGW(TAG, "RX packet size invalid: %u", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    char buf[MAX_JSON_BUF];
    uint16_t copied;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &copied);
    if (rc != 0) return BLE_ATT_ERR_UNLIKELY;
    buf[copied] = '\0';

    ESP_LOGD(TAG, "RX: %s", buf);
    ble_handler_process(buf);
    return 0;
}

/* TX characteristic — device notifies central */
static int ble_tx_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Read access not used; notify is pushed via ble_manager_notify()
    return 0;
}

// ─────────────────────────────────────────────
//  GATT service table
// ─────────────────────────────────────────────
//
//       CCCD subscribe events arrive via BLE_GAP_EVENT_SUBSCRIBE
//       in the GAP event handler below.
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(BLE_SVC_UUID128),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX – central writes JSON command packets here */
                .uuid      = BLE_UUID128_DECLARE(BLE_RX_UUID128),
                .access_cb = ble_rx_chr_access,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                /* TX – device notifies central.
                 * BLE_GATT_CHR_F_NOTIFY causes NimBLE to auto-create
                 * the CCCD descriptor (UUID 0x2902).
                 * When central writes to CCCD, BLE_GAP_EVENT_SUBSCRIBE fires. */
                .uuid       = BLE_UUID128_DECLARE(BLE_TX_UUID128),
                .access_cb  = ble_tx_chr_access,
                .val_handle = &s_tx_char_val_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }, // terminator
        },
    },
    { 0 }, // terminator
};

// ─────────────────────────────────────────────
//  GAP event handler
// ─────────────────────────────────────────────
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, handle=%u", s_conn_handle);
            // Send current device state to newly connected central
            ble_notify_on_connect();
        } else {
            ESP_LOGW(TAG, "Connection failed, status=%d", event->connect.status);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        ble_advertise();   // immediately restart advertising
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        // Central wrote to the CCCD of our TX characteristic
        if (event->subscribe.attr_handle == s_tx_char_val_handle) {
            s_notify_enabled = (event->subscribe.cur_notify == 1);
            ESP_LOGI(TAG, "Notify %s",
                     s_notify_enabled ? "enabled" : "disabled");
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGD(TAG, "Advertise complete, restarting");
        ble_advertise();
        break;

    default:
        break;
    }
    return 0;
}

// ─────────────────────────────────────────────
//  Advertising
// ─────────────────────────────────────────────
static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    struct ble_hs_adv_fields fields = {
        .flags                  = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .tx_pwr_lvl_is_present  = 1,
        .tx_pwr_lvl             = BLE_HS_ADV_TX_PWR_LVL_AUTO,
        .name                   = (uint8_t *)"ContainerMonitor",      // rename ble name to containerId
        .name_len               = (uint8_t)strlen("ContainerMonitor"),
        .name_is_complete       = 1,
    };

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields error: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv_start error: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started");
    }
}

// ─────────────────────────────────────────────
//  NimBLE host sync callback
// ─────────────────────────────────────────────
static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    assert(rc == 0);
    ble_advertise();
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset, reason=%d — resyncing", reason);
}

// ─────────────────────────────────────────────
//  NimBLE host task (runs on its own FreeRTOS task)
// ─────────────────────────────────────────────
static void nimble_host_task(void *param)
{
    nimble_port_run();          // blocks until nimble_port_stop()
    nimble_port_freertos_deinit();
}

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────
esp_err_t ble_manager_init(void)
{
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE HCI init failed: %d", ret);
        return ret;
    }
    

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("ContainerMonitor");     // rename ble name to containerId

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) return ESP_FAIL;
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) return ESP_FAIL;

    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "BLE manager initialised");
    return ESP_OK;
}

esp_err_t ble_manager_notify(const char *json)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_notify_enabled) {
        return ESP_FAIL;
    }
    if (s_tx_char_val_handle == 0) return ESP_FAIL;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(json, strlen(json));
    if (!om) return ESP_FAIL;

    int rc = ble_gattc_notify_custom(s_conn_handle, s_tx_char_val_handle, om);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

bool ble_manager_is_connected(void)
{
    return (s_conn_handle != BLE_HS_CONN_HANDLE_NONE);
}