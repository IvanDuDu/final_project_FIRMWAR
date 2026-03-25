#pragma once

#include "esp_err.h"
#include <stdbool.h>

// ─────────────────────────────────────────────
//  Phase 1 — Hardware init (luôn gọi lúc boot)
// ─────────────────────────────────────────────

/**
 * @brief  Khởi động UART2 và thực hiện AT handshake cơ bản với EC800K.
 *         Bật GNSS engine (AT+QGPS=1).
 *         Hàm này LUÔN được gọi khi boot, bất kể on_board hay không.
 *         Không kết nối data bearer.
 *
 *         Nếu module chưa phản hồi, hàm sẽ retry tối đa SIM_INIT_MAX_RETRIES
 *         với delay giữa các lần retry.
 *
 * @return ESP_OK nếu module phản hồi và GNSS được bật.
 */
esp_err_t sim_manager_hw_init(void);

// ─────────────────────────────────────────────
//  Phase 2 — Data connection (chỉ gọi khi cần mạng 4G)
// ─────────────────────────────────────────────

/**
 * @brief  Đăng ký mạng và bật PDP context để có IP.
 *         Gọi sau sim_manager_hw_init() khi isOnBoard == false.
 * @return ESP_OK nếu có IP.
 */
esp_err_t sim_manager_data_connect(void);

/**
 * @brief  Hủy PDP context (ngắt kết nối data), giữ UART và GNSS chạy.
 *         Gọi khi chuyển từ đất liền sang tàu (isOnBoard -> true).
 */
void sim_manager_data_disconnect(void);

// ─────────────────────────────────────────────
//  Backward-compat wrappers (dùng bởi server_connect.c)
// ─────────────────────────────────────────────

/** Alias của sim_manager_data_connect() — để server_connect.c không cần sửa. */
esp_err_t sim_manager_connect(void);

/** Alias của sim_manager_data_disconnect(). */
void sim_manager_disconnect(void);

/** Trả về true nếu PDP data bearer đang active. */
bool sim_manager_is_connected(void);

// ─────────────────────────────────────────────
//  Background monitor task
// ─────────────────────────────────────────────

/**
 * @brief  FreeRTOS task: định kỳ kiểm tra module còn sống không (AT ping).
 *         Nếu module không phản hồi, tự động re-init.
 *         Nếu data bearer bị drop mà đang ở land mode, tự reconnect.
 *         Spawn 1 lần sau sim_manager_hw_init().
 */
void task_sim_monitor(void *arg);