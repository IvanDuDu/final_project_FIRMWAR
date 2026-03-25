#include "sd_manager.h"
#include "globals.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <dirent.h>

static const char *TAG = "SD_MGR";

static sdmmc_card_t *s_card   = NULL;
static bool          s_mounted = false;

// ─────────────────────────────────────────────
//  Mount
// ─────────────────────────────────────────────
esp_err_t sd_manager_mount(void)
{
    if (s_mounted) {
        ESP_LOGW(TAG, "Already mounted");
        return ESP_OK;
    }

    // SDMMC host — 1-bit mode
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags        = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk   = SD_PIN_CLK;
    slot.cmd   = SD_PIN_CMD;
    slot.d0    = SD_PIN_D0;
    // D1-D7, CD, WP không dùng
    slot.d1 = GPIO_NUM_NC; slot.d2 = GPIO_NUM_NC;
    slot.d3 = GPIO_NUM_NC; slot.d4 = GPIO_NUM_NC;
    slot.d5 = GPIO_NUM_NC; slot.d6 = GPIO_NUM_NC;
    slot.d7 = GPIO_NUM_NC;
    slot.cd = GPIO_NUM_NC; slot.wp = GPIO_NUM_NC;

    esp_vfs_fat_sdmmc_mount_config_t mcfg = {
        .format_if_mount_failed   = false,
        .max_files                = 10,
        .allocation_unit_size     = 16 * 1024,
        .disk_status_check_enable = false,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot,
                                             &mcfg, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_mounted = true;
    ESP_LOGI(TAG, "SD card mounted at %s  (%.0f MB)",
             SD_MOUNT_POINT,
             (double)((uint64_t)s_card->csd.capacity * s_card->csd.sector_size) / (1024 * 1024));

    // Tạo thư mục nếu chưa có
    mkdir(SD_DIR_TELEMETRY, 0755);
    mkdir(SD_DIR_ALERTS,    0755);

    return ESP_OK;
}

void sd_manager_unmount(void)
{
    if (!s_mounted) return;
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card    = NULL;
    s_mounted = false;
    ESP_LOGI(TAG, "SD card unmounted");
}

bool sd_manager_is_mounted(void)
{
    return s_mounted;
}

// ─────────────────────────────────────────────
//  Internal: đếm số dòng trong file
// ─────────────────────────────────────────────
static int count_lines(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0;

    int   lines = 0;
    char  ch;
    while ((ch = fgetc(f)) != EOF) {
        if (ch == '\n') lines++;
    }
    fclose(f);
    return lines;
}

// ─────────────────────────────────────────────
//  Internal: append 1 dòng vào file
// ─────────────────────────────────────────────
static esp_err_t append_line(const char *path, const char *line)
{
    if (!s_mounted) {
        ESP_LOGW(TAG, "SD not mounted — cannot write %s", path);
        return ESP_ERR_INVALID_STATE;
    }

    FILE *f = fopen(path, "a");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s for append", path);
        return ESP_FAIL;
    }

    fprintf(f, "%s\n", line);
    fclose(f);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Telemetry
// ─────────────────────────────────────────────
esp_err_t sd_telemetry_append(const char *json_line)
{
    return append_line(SD_FILE_TELEMETRY, json_line);
}

esp_err_t sd_telemetry_read_all(char *buf, size_t buf_len, size_t *out_len)
{
    *out_len = 0;
    if (!s_mounted) return ESP_ERR_INVALID_STATE;

    FILE *f = fopen(SD_FILE_TELEMETRY, "r");
    if (!f) {
        // File chưa có = không có pending records
        return ESP_OK;
    }

    size_t n = fread(buf, 1, buf_len - 1, f);
    fclose(f);
    buf[n] = '\0';
    *out_len = n;
    return ESP_OK;
}

esp_err_t sd_telemetry_clear(void)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    // Truncate bằng cách mở lại với "w"
    FILE *f = fopen(SD_FILE_TELEMETRY, "w");
    if (!f) return ESP_FAIL;
    fclose(f);
    ESP_LOGI(TAG, "Telemetry file cleared");
    return ESP_OK;
}

int sd_telemetry_count(void)
{
    if (!s_mounted) return 0;
    return count_lines(SD_FILE_TELEMETRY);
}

// ─────────────────────────────────────────────
//  Alerts
// ─────────────────────────────────────────────
esp_err_t sd_alert_append(const char *json_line)
{
    return append_line(SD_FILE_ALERTS, json_line);
}

esp_err_t sd_alert_flush(sd_alert_line_cb_t cb, void *ctx)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    if (!cb)        return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(SD_FILE_ALERTS, "r");
    if (!f) return ESP_OK;   // Không có alert pending

    // Đọc từng dòng và gọi callback
    // Những dòng callback xử lý thành công sẽ bị bỏ qua trong lần flush tiếp theo
    // Cách đơn giản nhất: đọc hết, gọi callback từng dòng,
    // nếu TẤT CẢ thành công thì xóa file, còn không thì ghi lại phần chưa xử lý.

    // Đọc toàn bộ vào RAM tạm (alerts thường nhỏ)
    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    rewind(f);

    if (fsz <= 0) {
        fclose(f);
        return ESP_OK;
    }

    char *all = malloc(fsz + 1);
    if (!all) { fclose(f); return ESP_ERR_NO_MEM; }

    fread(all, 1, fsz, f);
    fclose(f);
    all[fsz] = '\0';

    // Tạo file tạm chứa những dòng CHƯA xử lý được
    char tmp_path[64];
    snprintf(tmp_path, sizeof(tmp_path), "%s/pending.tmp", SD_DIR_ALERTS);
    FILE *tmp = fopen(tmp_path, "w");

    char *line = all;
    char *nl;
    bool  any_failed = false;

    while ((nl = strchr(line, '\n')) != NULL) {
        *nl = '\0';
        if (strlen(line) > 0) {
            esp_err_t r = cb(line, ctx);
            if (r != ESP_OK) {
                // Ghi lại vào file tạm
                if (tmp) fprintf(tmp, "%s\n", line);
                any_failed = true;
            }
        }
        line = nl + 1;
    }
    // Xử lý dòng cuối không có newline
    if (strlen(line) > 0) {
        esp_err_t r = cb(line, ctx);
        if (r != ESP_OK) {
            if (tmp) fprintf(tmp, "%s\n", line);
            any_failed = true;
        }
    }

    if (tmp) fclose(tmp);
    free(all);

    if (!any_failed) {
        // Xóa file gốc và file tạm
        remove(SD_FILE_ALERTS);
        remove(tmp_path);
        ESP_LOGI(TAG, "All alerts flushed and cleared");
    } else {
        // Thay thế file gốc bằng file tạm (các dòng chưa gửi được)
        remove(SD_FILE_ALERTS);
        rename(tmp_path, SD_FILE_ALERTS);
        ESP_LOGW(TAG, "Some alerts could not be sent, kept in file");
    }

    return ESP_OK;
}

esp_err_t sd_alert_clear(void)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    FILE *f = fopen(SD_FILE_ALERTS, "w");
    if (!f) return ESP_FAIL;
    fclose(f);
    return ESP_OK;
}