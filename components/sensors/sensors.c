#include "sensors.h"
#include "globals.h"

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SENSORS";

// ─────────────────────────────────────────────
//  I2C helpers
// ─────────────────────────────────────────────
#define I2C_TIMEOUT_MS   50

static esp_err_t i2c_write(i2c_port_t port, uint8_t addr,
                            const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read(i2c_port_t port, uint8_t addr,
                           uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write_then_read(i2c_port_t port, uint8_t addr,
                                      const uint8_t *wbuf, size_t wlen,
                                      uint8_t *rbuf, size_t rlen)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, wbuf, wlen, true);
    i2c_master_start(cmd);   // repeated start
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (rlen > 1)
        i2c_master_read(cmd, rbuf, rlen - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, rbuf + rlen - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ─────────────────────────────────────────────
//  I2C Bus 1  (SHT30, GY906, DS3231, GPS)
// ─────────────────────────────────────────────
esp_err_t i2c_bus1_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C1_SDA,
        .scl_io_num       = I2C1_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_PORT_BUS1, &cfg);
    if (ret != ESP_OK) return ret;
    ret = i2c_driver_install(I2C_PORT_BUS1, I2C_MODE_MASTER, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C bus1 already installed");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "I2C Bus1 initialised (port=%d)", I2C_PORT_BUS1);
    return ret;
}

// ─────────────────────────────────────────────
//  I2C Bus 2  (MPU6050 only)
// ─────────────────────────────────────────────
esp_err_t i2c_bus2_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C2_SDA,
        .scl_io_num       = I2C2_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_PORT_BUS2, &cfg);
    if (ret != ESP_OK) return ret;
    ret = i2c_driver_install(I2C_PORT_BUS2, I2C_MODE_MASTER, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C bus2 already installed");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "I2C Bus2 initialised (port=%d)", I2C_PORT_BUS2);
    return ret;
}

// ═════════════════════════════════════════════
//  SHT30  —  internal temperature & humidity
// ═════════════════════════════════════════════
#define SHT30_CMD_MEAS_HIGHREP  0x2C, 0x06

esp_err_t sht30_init(void)
{
    // SHT30 needs no special init; just verify it ACKs on the bus
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT30_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_BUS1, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "SHT30 not found on bus1");
    else
        ESP_LOGI(TAG, "SHT30 OK");
    return ret;
}

/* CRC-8 for SHT30 (polynomial 0x31, init 0xFF) */
static uint8_t sht30_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

esp_err_t sht30_read(float *temperature_c, float *humidity_pct)
{
    // Send measurement command (high repeatability, clock stretching)
    const uint8_t cmd[2] = {0x2C, 0x06};
    esp_err_t ret = i2c_write(I2C_PORT_BUS1, SHT30_I2C_ADDR, cmd, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 write cmd failed: %d", ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // measurement time ~15ms

    uint8_t raw[6] = {0};
    ret = i2c_read(I2C_PORT_BUS1, SHT30_I2C_ADDR, raw, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 read failed: %d", ret);
        return ret;
    }

    // Verify CRC
    if (sht30_crc8(&raw[0], 2) != raw[2] ||
        sht30_crc8(&raw[3], 2) != raw[5]) {
        ESP_LOGE(TAG, "SHT30 CRC error");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_t = ((uint16_t)raw[0] << 8) | raw[1];
    uint16_t raw_h = ((uint16_t)raw[3] << 8) | raw[4];

    *temperature_c = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    *humidity_pct  = 100.0f  * ((float)raw_h / 65535.0f);
    return ESP_OK;
}

// ═════════════════════════════════════════════
//  GY-906 / MLX90614  —  surface temperature
// ═════════════════════════════════════════════
#define GY906_RAM_TOBJ1  0x07

esp_err_t gy906_init(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (GY906_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_BUS1, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "GY-906 not found on bus1");
    else
        ESP_LOGI(TAG, "GY-906 OK");
    return ret;
}

esp_err_t gy906_read_object_temp(float *temperature_c)
{
    // MLX90614 uses SMBus read: send 1-byte command, read 3 bytes (2 data + PEC)
    uint8_t cmd_byte = GY906_RAM_TOBJ1;
    uint8_t raw[3] = {0};

    esp_err_t ret = i2c_write_then_read(I2C_PORT_BUS1,
                                         GY906_I2C_ADDR,
                                         &cmd_byte, 1,
                                         raw, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GY906 read failed: %d", ret);
        return ret;
    }

    uint16_t data = ((uint16_t)(raw[1] & 0x7F) << 8) | raw[0];
    // Temperature in Kelvin × 50, convert to Celsius
    *temperature_c = (data * 0.02f) - 273.15f;
    return ESP_OK;
}

// ═════════════════════════════════════════════
//  MPU6050  —  tilt in Oxz plane + collision ISR
//  Resides on I2C_PORT_BUS2 (I2C_NUM_1)
// ═════════════════════════════════════════════
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_INT_ENABLE   0x38
#define MPU_REG_MOT_THR      0x1F
#define MPU_REG_MOT_DUR      0x20
#define MPU_REG_INT_PIN_CFG  0x37

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write(I2C_PORT_BUS2, MPU6050_I2C_ADDR, buf, 2);
}

static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_then_read(I2C_PORT_BUS2, MPU6050_I2C_ADDR, &reg, 1, buf, len);
}

esp_err_t mpu6050_init(void)
{
    // Wake up device (clear sleep bit)
    esp_err_t ret = mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "MPU6050 init failed"); return ret; }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set full-scale range ±2g
    ret = mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 OK (bus2)");
    return ESP_OK;
}

esp_err_t mpu6050_read_lean(float *lean_deg)
{
    uint8_t raw[6] = {0};
    esp_err_t ret = mpu_read_regs(MPU_REG_ACCEL_XOUT_H, raw, 6);
    if (ret != ESP_OK) return ret;

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    // int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);  // not used for Oxz
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

    // Tilt angle in Oxz plane: angle between resultant (ax,az) and +X axis
    // +X pointing upward → 0 deg = upright
    *lean_deg = atan2f((float)az, (float)ax) * (180.0f / (float)M_PI);
    return ESP_OK;
}

esp_err_t mpu6050_enable_motion_interrupt(void)
{
    // INT pin active-high, push-pull, latched until status read
    esp_err_t ret = mpu_write_reg(MPU_REG_INT_PIN_CFG, 0x20);
    if (ret != ESP_OK) return ret;

    // Motion threshold ≈ 0.5g (32 × 16mg = 512mg)
    ret = mpu_write_reg(MPU_REG_MOT_THR, 32);
    if (ret != ESP_OK) return ret;

    // Motion duration: 1 ms
    ret = mpu_write_reg(MPU_REG_MOT_DUR, 1);
    if (ret != ESP_OK) return ret;

    // Enable motion interrupt bit (bit 6)
    ret = mpu_write_reg(MPU_REG_INT_ENABLE, 0x40);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 motion interrupt enabled");
    return ESP_OK;
}

// ═════════════════════════════════════════════
//  DS3231 RTC
// ═════════════════════════════════════════════
#define DS3231_REG_SECONDS  0x00

static uint8_t bcd2dec(uint8_t bcd) { return (bcd >> 4) * 10 + (bcd & 0x0F); }

esp_err_t ds3231_init(void)
{
    uint8_t reg = DS3231_REG_SECONDS;
    uint8_t buf[1];
    esp_err_t ret = i2c_write_then_read(I2C_PORT_BUS1,
                                         DS3231_I2C_ADDR,
                                         &reg, 1, buf, 1);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "DS3231 not found");
    else
        ESP_LOGI(TAG, "DS3231 OK");
    return ret;
}

esp_err_t ds3231_get_timestamp(char *buf, size_t len)
{
    uint8_t reg = DS3231_REG_SECONDS;
    uint8_t raw[7] = {0};
    esp_err_t ret = i2c_write_then_read(I2C_PORT_BUS1,
                                         DS3231_I2C_ADDR,
                                         &reg, 1, raw, 7);
    if (ret != ESP_OK) {
        snprintf(buf, len, "00:00:00 01/01/2000 GMT+0");
        return ret;
    }

    uint8_t sec  = bcd2dec(raw[0] & 0x7F);
    uint8_t min  = bcd2dec(raw[1] & 0x7F);
    uint8_t hour = bcd2dec(raw[2] & 0x3F);
    uint8_t day  = bcd2dec(raw[4] & 0x3F);
    uint8_t mon  = bcd2dec(raw[5] & 0x1F);
    uint16_t yr  = 2000 + bcd2dec(raw[6]);

    snprintf(buf, len, "%02u:%02u:%02u %02u/%02u/%04u GMT+0",
             hour, min, sec, day, mon, yr);
    return ESP_OK;
}

// ═════════════════════════════════════════════
//  GPS — Quectel EC800K built-in GNSS
//
//  Giao tiếp qua cùng UART2 với moduleSIM.
//  Chuỗi AT command sử dụng:
//
//  gps_init():
//    AT+QGPSCFG="outport","none"   — tắt output NMEA tự động ra cổng
//    AT+QGPSCFG="nmeasrc",1        — cho phép đọc NMEA qua AT+QGPSGNMEA
//    AT+QGPS=1                     — bật GNSS engine (standalone mode)
//
//  gps_read_position():
//    AT+QGPSLOC=2                  — lấy vị trí (format=2: decimal degrees)
//    Response: +QGPSLOC: <UTC>,<lat>,<lon>,<hdop>,<alt>,<fix>,<cog>,<spkm>,<spkn>,<date>,<nsat>
// ═════════════════════════════════════════════

// UART2 shared with SIM module — use the same port/pins
#define GPS_UART_PORT   UART_NUM_2
#define GPS_CMD_TIMEOUT 3000    // ms — QGPSLOC có thể mất vài giây
#define GPS_BUF_SIZE    512

static bool s_gps_ready = false;

// Tái sử dụng helper gửi AT command đơn giản (không depend sim_manager)
static esp_err_t gps_send_at(const char *cmd, const char *expect,
                              char *resp_buf, size_t resp_len, int timeout_ms)
{
    uart_flush_input(GPS_UART_PORT);
    uart_write_bytes(GPS_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(GPS_UART_PORT, "\r\n", 2);
    ESP_LOGD(TAG, "GPS>> %s", cmd);

    if (!expect) return ESP_OK;

    char buf[GPS_BUF_SIZE] = {0};
    int  total   = 0;
    int  elapsed = 0;

    while (elapsed < timeout_ms) {
        int n = uart_read_bytes(GPS_UART_PORT,
                                (uint8_t *)(buf + total),
                                sizeof(buf) - total - 1,
                                pdMS_TO_TICKS(100));
        if (n > 0) total += n;
        buf[total] = '\0';

        if (strstr(buf, expect)) {
            if (resp_buf && resp_len > 0) {
                strlcpy(resp_buf, buf, resp_len);
            }
            ESP_LOGD(TAG, "GPS<< %s", buf);
            return ESP_OK;
        }
        // Error responses
        if (strstr(buf, "+CME ERROR") || strstr(buf, "ERROR")) {
            ESP_LOGW(TAG, "GPS AT error: %s", buf);
            return ESP_FAIL;
        }
        elapsed += 100;
    }
    ESP_LOGW(TAG, "GPS AT timeout: %s", cmd);
    return ESP_ERR_TIMEOUT;
}

// Parse +QGPSLOC response (format=2, decimal degrees)
// Response: +QGPSLOC: <UTC>,<lat>,<lon>,<hdop>,<alt>,<fix>,<cog>,<spkm>,<spkn>,<date>,<nsat>
static esp_err_t parse_qgpsloc(const char *resp, double *lat, double *lon)
{
    const char *prefix = "+QGPSLOC:";
    char *p = strstr(resp, prefix);
    if (!p) return ESP_FAIL;
    p += strlen(prefix);

    // Skip leading space
    while (*p == ' ') p++;

    // Fields: UTC, lat, lon, ...
    double utc_val, lat_val, lon_val;
    // sscanf with %*[^,],%lf,%lf would skip first field
    // safer: find first comma (UTC), then parse lat, lon
    char *comma1 = strchr(p, ',');
    if (!comma1) return ESP_FAIL;
    comma1++;  // skip past UTC field

    if (sscanf(comma1, "%lf,%lf", &lat_val, &lon_val) != 2)
        return ESP_FAIL;

    *lat = lat_val;
    *lon = lon_val;
    return ESP_OK;
}

esp_err_t gps_init(void)
{
    // UART2 đã được khởi tạo bởi sim_manager_init().
    // Chỉ cần gửi chuỗi AT config GNSS.

    ESP_LOGI(TAG, "Initialising EC800K GNSS...");

    // 1. Tắt NMEA auto-output ra cổng (tránh rác trong UART buffer)
    esp_err_t ret = gps_send_at("AT+QGPSCFG=\"outport\",\"none\"",
                                 "OK", NULL, 0, 2000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "QGPSCFG outport failed (may already be set): %d", ret);
    }

    // 2. Bật mode đọc NMEA qua AT+QGPSGNMEA
    ret = gps_send_at("AT+QGPSCFG=\"nmeasrc\",1",
                      "OK", NULL, 0, 2000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "QGPSCFG nmeasrc failed: %d", ret);
    }

    // 3. Bật GNSS engine — standalone mode
    //    Nếu đã bật trước đó sẽ trả về error, không sao
    ret = gps_send_at("AT+QGPS=1", "OK", NULL, 0, 3000);
    if (ret != ESP_OK) {
        // Kiểm tra xem đã bật chưa
        char resp[GPS_BUF_SIZE];
        gps_send_at("AT+QGPS?", "+QGPS:", resp, sizeof(resp), 2000);
        if (strstr(resp, "+QGPS: 1")) {
            ESP_LOGI(TAG, "GNSS engine already running");
            ret = ESP_OK;
        } else {
            ESP_LOGE(TAG, "Cannot start GNSS engine");
            return ret;
        }
    }

    s_gps_ready = true;
    ESP_LOGI(TAG, "EC800K GNSS engine started");
    return ESP_OK;
}

esp_err_t gps_read_position(double *latitude, double *longitude)
{
    // Safe defaults
    *latitude  = 0.0;
    *longitude = 0.0;

    if (!s_gps_ready) {
        ESP_LOGW(TAG, "GPS not initialised");
        return ESP_ERR_INVALID_STATE;
    }

    char resp[GPS_BUF_SIZE] = {0};

    // AT+QGPSLOC=2  — format 2 = decimal degrees (±DD.DDDDD)
    // Nếu chưa có fix sẽ trả về +CME ERROR: 516
    esp_err_t ret = gps_send_at("AT+QGPSLOC=2",
                                 "+QGPSLOC:", resp, sizeof(resp),
                                 GPS_CMD_TIMEOUT);
    if (ret != ESP_OK) {
        // +CME ERROR: 516 = no fix yet, không phải lỗi cứng
        if (strstr(resp, "516")) {
            ESP_LOGD(TAG, "GPS: no fix yet (error 516)");
        } else {
            ESP_LOGW(TAG, "GPS read failed: %d", ret);
        }
        return ret;
    }

    ret = parse_qgpsloc(resp, latitude, longitude);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "GPS parse failed, raw: %s", resp);
        return ret;
    }

    ESP_LOGD(TAG, "GPS fix: lat=%.6f lon=%.6f", *latitude, *longitude);
    return ESP_OK;
}