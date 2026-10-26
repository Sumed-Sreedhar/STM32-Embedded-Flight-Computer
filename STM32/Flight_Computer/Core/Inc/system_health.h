/*
 * system_health.h
 *
 *  Created on: 29-Jul-2026
 *      Author: sumed
 */

#ifndef INC_SYSTEM_HEALTH_H_
#define INC_SYSTEM_HEALTH_H_

#include "main.h"

typedef enum{
	NORMAL=0,
	FAULT,
	CRITICAL_FAULT
}health_state_t;

typedef enum
{
	SENSOR_BMP280 = 0,
    SENSOR_MPU9250,
    SENSOR_MLX90393,
	SENSOR_COUNT
} sensor_id_t;

void handle_system_health(void);

extern volatile health_state_t current_health_state;

void SystemHealth_CaptureStartupResult(void);

void SystemHealth_ReportSensor(sensor_id_t sensor, HAL_StatusTypeDef result);

void SystemHealth_Update(void);

void SystemHealth_CycleTestFault(void);


#endif /* INC_SYSTEM_HEALTH_H_ */
