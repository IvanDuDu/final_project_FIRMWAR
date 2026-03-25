#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────
//  I2C bus initialisation
//  Call once at startup before any sensor read.
// ─────────────────────────────────────────────

/**
 * @brief  Initialise I2C Bus 0 (SDA=21, SCL=22) for
 *         SHT30, GY-906, DS3231, GPS.
 */
esp_err_t i2c_bus1_init(void);

/**
 * @brief  Initialise I2C Bus 1 (SDA=21, SCL=22) for MPU6050.
 *         Uses a separate ESP-IDF i2c driver instance (I2C_NUM_1)
 *         so address 0x68 on this bus never conflicts with DS3231.
 */
esp_err_t i2c_bus2_init(void);

// ─────────────────────────────────────────────
//  SHT30  (I2C Bus 1, addr 0x44)
//  Internal temperature & humidity
// ─────────────────────────────────────────────
esp_err_t sht30_init(void);
esp_err_t sht30_read(float *temperature_c, float *humidity_pct);

// ─────────────────────────────────────────────
//  GY-906 / MLX90614  (I2C Bus 1, addr 0x5A)
//  Non-contact surface temperature
// ─────────────────────────────────────────────
esp_err_t gy906_init(void);
esp_err_t gy906_read_object_temp(float *temperature_c);

// ─────────────────────────────────────────────
//  MPU6050  (I2C Bus 2, addr 0x68)
//  Tilt angle in Oxz plane (degrees from +X axis pointing skyward)
//  Interrupt wired to GPIO 9 for collision detection.
// ─────────────────────────────────────────────
esp_err_t mpu6050_init(void);

/**
 * @brief  Read tilt angle in the Oxz plane.
 * @param  lean_deg  Output angle in degrees; 0 = perfectly upright (+X up).
 */
esp_err_t mpu6050_read_lean(float *lean_deg);

/**
 * @brief  Configure MPU6050 motion interrupt on INT pin.
 *         Threshold: 2g  (raw value ~32; 1 LSB ≈ 16mg at ±2g range).
 */
esp_err_t mpu6050_enable_motion_interrupt(void);

// ─────────────────────────────────────────────
//  DS3231 RTC  (I2C Bus 1, addr 0x68)
// ─────────────────────────────────────────────
esp_err_t ds3231_init(void);

/**
 * @brief  Read current time and format into buf as
 *         "HH:MM:SS DD/MM/YYYY GMT+0"
 */
esp_err_t ds3231_get_timestamp(char *buf, size_t len);

// ─────────────────────────────────────────────
//  GPS — Quectel EC800K built-in GNSS
//  Giao tiếp qua UART2 (shared với SIM module).
//  Dùng AT+QGPS và AT+QGPSLOC=2 (decimal degrees).
//
//  Lưu ý: gps_init() phải gọi SAU sim_manager_init()
//  vì UART2 được khởi tạo bởi sim_manager.
// ─────────────────────────────────────────────

/**
 * @brief  Bật GNSS engine của EC800K.
 *         Gửi: AT+QGPSCFG="outport","none"
 *              AT+QGPSCFG="nmeasrc",1
 *              AT+QGPS=1
 *         Nếu GNSS đã đang chạy, hàm vẫn trả về ESP_OK.
 *         Phải gọi sau sim_manager_init().
 */
esp_err_t gps_init(void);

/**
 * @brief  Đọc vị trí GPS hiện tại qua AT+QGPSLOC=2.
 *
 * @param  latitude   Vĩ độ (decimal degrees, ±90).
 * @param  longitude  Kinh độ (decimal degrees, ±180).
 * @return ESP_OK     Fix thành công, latitude/longitude đã được điền.
 *         ESP_ERR_TIMEOUT  Chưa có GPS fix (CME ERROR 516) — caller nên
 *                          dùng giá trị 0.0 và đánh dấu gps_valid=false.
 */
esp_err_t gps_read_position(double *latitude, double *longitude);