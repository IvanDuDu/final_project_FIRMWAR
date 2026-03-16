#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// ─────────────────────────────────────────────
//  HARDWARE PIN DEFINITIONS
// ─────────────────────────────────────────────
#define PIN_FAN_PWM         6
#define PIN_UV_LED          7
#define PIN_DOOR_SENSOR     8
#define PIN_MPU_INT         9

#define I2C1_SDA            21
#define I2C1_SCL            22
#define I2C2_SDA            21      // MPU6050 on separate I2C bus instance
#define I2C2_SCL            22

#define I2C_PORT_BUS1       I2C_NUM_0
#define I2C_PORT_BUS2       I2C_NUM_1
#define I2C_FREQ_HZ         400000

// ─────────────────────────────────────────────
//  I2C DEVICE ADDRESSES
// ─────────────────────────────────────────────
#define SHT30_I2C_ADDR      0x44
#define GY906_I2C_ADDR      0x5A
#define DS3231_I2C_ADDR     0x68
#define MPU6050_I2C_ADDR    0x68    // on I2C_BUS2 — no conflict
#define GPS_I2C_ADDR        0x00    // placeholder, no module yet

// ─────────────────────────────────────────────
//  LEDC / PWM
// ─────────────────────────────────────────────
#define FAN_LEDC_CHANNEL    LEDC_CHANNEL_0
#define FAN_LEDC_TIMER      LEDC_TIMER_0
#define FAN_LEDC_FREQ_HZ    25000
#define FAN_LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define FAN_PWM_MAX         1023    // 2^10 - 1

// ─────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────
#define SENSOR_READ_INTERVAL_MS     (15 * 60 * 1000)
#define TELEMETRY_SEND_INTERVAL_MS  (60 * 60 * 1000)
#define HEARTBEAT_INTERVAL_MS       (5  * 60 * 1000)
#define TELEMETRY_BATCH_SIZE        4

// ─────────────────────────────────────────────
//  MQTT BROKER
// ─────────────────────────────────────────────
#define MQTT_BROKER_URI     "mqtt://192.168.1.9"
#define MQTT_PORT           1883

// ─────────────────────────────────────────────
//  NVS KEYS
// ─────────────────────────────────────────────
#define NVS_NAMESPACE           "container"
#define NVS_KEY_CONTAINER_ID    "ctr_id"
#define NVS_KEY_CUSTOMER_ID     "cust_id"
#define NVS_KEY_PROVIDER_ID     "prov_id"
#define NVS_KEY_HUMID_THRESH    "hum_thr"
#define NVS_KEY_WEIGHT          "weight"
#define NVS_KEY_WIFI_SSID       "wifi_ssid"
#define NVS_KEY_WIFI_PASS       "wifi_pass"
#define NVS_KEY_STAGE           "stage"
#define NVS_KEY_DOOR_ENABLE     "door_en"
#define NVS_KEY_FAN_ENABLE      "fan_en"
#define NVS_KEY_UV_ENABLE       "uv_en"
#define NVS_KEY_ROUTE_HISTORY   "route_hist"
#define NVS_KEY_SEND_TELEMETRY  "send_telem"
#define NVS_KEY_TELEM_COUNT     "telem_cnt"

// ─────────────────────────────────────────────
//  LIMITS
// ─────────────────────────────────────────────
#define MAX_CONTAINER_ID_LEN    32
#define MAX_CUSTOMER_ID_LEN     32
#define MAX_PROVIDER_ID_LEN     32
#define MAX_SHIPPER_ID_LEN      32
#define MAX_ROUTE_HISTORY       16
#define MAX_TELEMETRY_BUFFER    32
#define MAX_SSID_LEN            64
#define MAX_PASS_LEN            64
#define MAX_PRODUCT_TYPE_LEN    32
#define MAX_LOCATION_LEN        64
#define MAX_JSON_BUF            1024

// ─────────────────────────────────────────────
//  STAGE ENUM
// ─────────────────────────────────────────────
typedef enum {
    STAGE_SETUP     = 1,
    STAGE_TRANSPORT = 2,
    STAGE_RECEIVE   = 3,
} app_stage_t;

// ─────────────────────────────────────────────
//  TELEMETRY STRUCTS
// ─────────────────────────────────────────────
typedef struct {
    char      timestamp[32];    // "HH:MM:SS DD/MM/YYYY GMT+0"
    float     in_temp;          // SHT30 internal temperature
    float     sur_temp;         // GY-906 surface temperature
    float     humidity;         // SHT30 humidity
    float     lean;             // MPU6050 tilt angle (Oxz plane, deg)
    double    latitude;
    double    longitude;
    float     loss;             // calculated loss kg
} telemetry_record_t;

typedef struct {
    char    shipper_id[MAX_SHIPPER_ID_LEN];
    bool    is_on_board;        // true = sea (WiFi), false = land (5G)
    char    timestamp[32];
} route_entry_t;

// ─────────────────────────────────────────────
//  CONTAINER CONFIG (from CMD 00)
// ─────────────────────────────────────────────
typedef struct {
    char    container_id[MAX_CONTAINER_ID_LEN];
    char    customer_id[MAX_CUSTOMER_ID_LEN];
    char    provider_id[MAX_PROVIDER_ID_LEN];
    float   humid_threshold;
    char    type_of_product[MAX_PRODUCT_TYPE_LEN];
    float   weight;             // kg
    char    from[MAX_LOCATION_LEN];
    char    to[MAX_LOCATION_LEN];
    char    release_date[32];
} container_config_t;

// ─────────────────────────────────────────────
//  GLOBAL STATE  (defined in globals.c)
// ─────────────────────────────────────────────
extern volatile bool        g_door_enable;
extern volatile bool        g_fan_enable;
extern volatile bool        g_uv_enable;
extern volatile app_stage_t g_stage;

extern container_config_t   g_config;
extern route_entry_t        g_route_history[MAX_ROUTE_HISTORY];
extern int                  g_route_count;
extern telemetry_record_t   g_send_telemetry[MAX_TELEMETRY_BUFFER];
extern int                  g_telem_count;

// last sensor reading (for BLE notify + fan control)
extern telemetry_record_t   g_latest_telemetry;
extern float                g_prev_humidity;     // for fan auto-disable logic

// ─────────────────────────────────────────────
//  RTOS SYNC PRIMITIVES (defined in globals.c)
// ─────────────────────────────────────────────
extern SemaphoreHandle_t    g_sem_door_alert;
extern SemaphoreHandle_t    g_sem_collision;
extern SemaphoreHandle_t    g_mutex_telemetry;
extern SemaphoreHandle_t    g_mutex_globals;
extern EventGroupHandle_t   g_evt_network;

// Network event bits
#define NET_EVT_WIFI_READY  BIT0
#define NET_EVT_SIM_READY   BIT1
#define NET_EVT_MQTT_READY  BIT2