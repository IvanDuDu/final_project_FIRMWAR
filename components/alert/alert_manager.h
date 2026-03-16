// alert_manager.h
#pragma once
#include "esp_err.h"

/**
 * @brief  FreeRTOS task: blocks on g_sem_door_alert and g_sem_collision.
 *         On semaphore, checks enable flags and publishes MQTT warning.
 *         Task handles both alert types using a polling loop over both
 *         semaphores with short timeout.
 */
void task_alert_handler(void *arg);