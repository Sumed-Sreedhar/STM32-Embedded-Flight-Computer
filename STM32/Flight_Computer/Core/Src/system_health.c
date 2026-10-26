/*
 * system_health.c
 *
 *  Created on: 29-Jul-2026
 *      Author: sumed
 */

#include "system_health.h"
#include "led.h"

#include "FreeRTOS.h"
#include "task.h"

#define SENSOR_STARTUP_GRACE_MS       1000U
#define SENSOR_STALE_TIMEOUT_MS        500U
#define SENSOR_FAILURE_LATCH_COUNT       3U
#define SENSOR_RECOVERY_SUCCESS_COUNT  10U

typedef struct
{
    uint32_t last_success_tick;
    uint8_t has_success;
    uint8_t consecutive_failures;
    uint8_t consecutive_successes;
    uint8_t fault_latched;
} sensor_health_record_t;

volatile health_state_t current_health_state = NORMAL;

static sensor_health_record_t sensor_health[SENSOR_COUNT];
static health_state_t startup_health_floor = NORMAL;
static health_state_t test_health_state = NORMAL;

void SystemHealth_CaptureStartupResult(void)
{
    startup_health_floor = current_health_state;
}

void SystemHealth_ReportSensor(sensor_id_t sensor, HAL_StatusTypeDef result)
{
    uint32_t now;

    if ((uint32_t)sensor >= (uint32_t)SENSOR_COUNT)
    {
        return;
    }

    now = HAL_GetTick();

    taskENTER_CRITICAL();

    if (result == HAL_OK)
    {
        sensor_health[sensor].last_success_tick = now;
        sensor_health[sensor].has_success = 1U;
        sensor_health[sensor].consecutive_failures = 0U;

        if (sensor_health[sensor].consecutive_successes <
            SENSOR_RECOVERY_SUCCESS_COUNT)
        {
            sensor_health[sensor].consecutive_successes++;
        }

        if (sensor_health[sensor].consecutive_successes >=
            SENSOR_RECOVERY_SUCCESS_COUNT)
        {
            sensor_health[sensor].fault_latched = 0U;
        }
    }
    else
    {
        sensor_health[sensor].consecutive_successes = 0U;

        if (sensor_health[sensor].consecutive_failures <
            SENSOR_FAILURE_LATCH_COUNT)
        {
            sensor_health[sensor].consecutive_failures++;
        }

        if (sensor_health[sensor].consecutive_failures >=
            SENSOR_FAILURE_LATCH_COUNT)
        {
            sensor_health[sensor].fault_latched = 1U;
        }
    }

    taskEXIT_CRITICAL();
}

static uint8_t SensorIsFaulted(
    const sensor_health_record_t *record,
    uint32_t now)
{
    if (record->fault_latched != 0U)
    {
        return 1U;
    }

    if (record->has_success == 0U)
    {
        return (now >= SENSOR_STARTUP_GRACE_MS) ? 1U : 0U;
    }

    if ((uint32_t)(now - record->last_success_tick) >
        SENSOR_STALE_TIMEOUT_MS)
    {
        return 1U;
    }

    return 0U;
}

void SystemHealth_Update(void)
{
    sensor_health_record_t snapshot[SENSOR_COUNT];
    health_state_t next_health;
    uint32_t now = HAL_GetTick();
    uint32_t i;
    uint8_t bmp_fault;
    uint8_t mpu_fault;
    uint8_t mlx_fault;

    taskENTER_CRITICAL();
    for (i = 0U; i < (uint32_t)SENSOR_COUNT; i++)
    {
        snapshot[i] = sensor_health[i];
    }
    taskEXIT_CRITICAL();

    next_health = startup_health_floor;

    if (test_health_state > next_health)
    {
        next_health = test_health_state;
    }

    bmp_fault = SensorIsFaulted(&snapshot[SENSOR_BMP280], now);
    mpu_fault = SensorIsFaulted(&snapshot[SENSOR_MPU9250], now);
    mlx_fault = SensorIsFaulted(&snapshot[SENSOR_MLX90393], now);

    if (mpu_fault != 0U)
    {
        next_health = CRITICAL_FAULT;
    }
    else if ((bmp_fault != 0U || mlx_fault != 0U) &&
             next_health < FAULT)
    {
        next_health = FAULT;
    }

    current_health_state = next_health;
}

void SystemHealth_CycleTestFault(void)
{
    if (test_health_state == NORMAL)
    {
        test_health_state = FAULT;
    }
    else if (test_health_state == FAULT)
    {
        test_health_state = CRITICAL_FAULT;
    }
    else
    {
        test_health_state = NORMAL;
    }
}

void handle_system_health(void)
{
    switch (current_health_state)
    {
    case NORMAL:
        led_normal_on();
        led_fault_off();
        break;

    case FAULT:
        led_normal_off();
        led_fault_on();
        break;

    case CRITICAL_FAULT:
        led_normal_off();
        led_fault_blink();
        break;

    default:
        current_health_state = CRITICAL_FAULT;
        break;
    }
}

