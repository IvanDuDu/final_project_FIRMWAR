#pragma once
#include "esp_err.h"
#include "globals.h"

/**
 * @brief  Run the top-level application stage machine.
 *         Blocks in the current stage until transition conditions are met,
 *         then advances to the next stage.
 *         Called once from app_main after all subsystems are initialised.
 *
 *  Stage 1 → Stage 2: CMD 00 received AND door sensor reads CLOSED.
 *  Stage 2 → Stage 3: CMD 02 received with correct customerID
 *                     (sets g_stage = STAGE_RECEIVE internally via ble_handler).
 *  Stage 3 → Stage 1: After cleanup, reset and restart stage machine.
 */
void stage_machine_run(void);