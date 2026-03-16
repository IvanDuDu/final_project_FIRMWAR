#include "globals.h"
#include <string.h>

// ─────────────────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────────────────
volatile bool           g_door_enable   = false;
volatile bool           g_fan_enable    = false;
volatile bool           g_uv_enable     = false;
volatile app_stage_t    g_stage         = STAGE_SETUP;

container_config_t      g_config        = {0};
route_entry_t           g_route_history[MAX_ROUTE_HISTORY] = {0};
int                     g_route_count   = 0;
telemetry_record_t      g_send_telemetry[MAX_TELEMETRY_BUFFER] = {0};
int                     g_telem_count   = 0;
telemetry_record_t      g_latest_telemetry = {0};
float                   g_prev_humidity = 0.0f;

// ─────────────────────────────────────────────
//  RTOS SYNC PRIMITIVES
// ─────────────────────────────────────────────
SemaphoreHandle_t   g_sem_door_alert    = NULL;
SemaphoreHandle_t   g_sem_collision     = NULL;
SemaphoreHandle_t   g_mutex_telemetry   = NULL;
SemaphoreHandle_t   g_mutex_globals     = NULL;
EventGroupHandle_t  g_evt_network       = NULL;