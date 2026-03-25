#pragma once

#include "esp_err.h"
#include "globals.h"
#include <stdbool.h>

// ─────────────────────────────────────────────
//  SD Card pin definitions  (1-bit SDMMC)
// ─────────────────────────────────────────────
#define SD_PIN_CLK      14
#define SD_PIN_CMD      15
#define SD_PIN_D0       2

// Mount point và paths
#define SD_MOUNT_POINT          "/sdcard"
#define SD_DIR_TELEMETRY        "/sdcard/telemetry"
#define SD_DIR_ALERTS           "/sdcard/alerts"
#define SD_FILE_TELEMETRY       "/sdcard/telemetry/pending.jsonl"
#define SD_FILE_ALERTS          "/sdcard/alerts/pending.jsonl"

// Giới hạn kích thước file (bytes) trước khi rotate
#define SD_MAX_FILE_SIZE_BYTES  (512 * 1024)   // 512 KB

/**
 * @brief  Mount thẻ SD qua SDMMC 1-bit mode.
 *         Tạo thư mục /telemetry và /alerts nếu chưa có.
 *         Gọi một lần khi boot.
 */
esp_err_t sd_manager_mount(void);

/** Unmount thẻ SD (gọi trước khi tháo thẻ). */
void sd_manager_unmount(void);

/** Trả về true nếu thẻ SD đã được mount thành công. */
bool sd_manager_is_mounted(void);

// ─────────────────────────────────────────────
//  Telemetry
// ─────────────────────────────────────────────

/**
 * @brief  Ghi 1 bản tin telemetry JSON vào file pending.
 *         Mỗi bản tin nằm trên 1 dòng (JSONL format).
 * @param  json_line  Chuỗi JSON không có newline ở cuối.
 */
esp_err_t sd_telemetry_append(const char *json_line);

/**
 * @brief  Đọc toàn bộ nội dung file telemetry pending vào buffer.
 * @param  buf      Buffer đầu ra.
 * @param  buf_len  Kích thước buffer.
 * @param  out_len  Số byte thực sự đọc được.
 */
esp_err_t sd_telemetry_read_all(char *buf, size_t buf_len, size_t *out_len);

/**
 * @brief  Xóa file telemetry pending (sau khi đã gửi thành công).
 */
esp_err_t sd_telemetry_clear(void);

/**
 * @brief  Trả về số dòng (số record) hiện có trong file telemetry.
 */
int sd_telemetry_count(void);

// ─────────────────────────────────────────────
//  Alerts
// ─────────────────────────────────────────────

/**
 * @brief  Ghi 1 bản tin alert JSON vào file pending.
 */
esp_err_t sd_alert_append(const char *json_line);

/**
 * @brief  Đọc từng dòng alert, gọi callback cho mỗi dòng.
 *         Callback trả về ESP_OK = đã xử lý xong dòng đó.
 */
typedef esp_err_t (*sd_alert_line_cb_t)(const char *json_line, void *ctx);
esp_err_t sd_alert_flush(sd_alert_line_cb_t cb, void *ctx);

/**
 * @brief  Xóa file alert pending.
 */
esp_err_t sd_alert_clear(void);