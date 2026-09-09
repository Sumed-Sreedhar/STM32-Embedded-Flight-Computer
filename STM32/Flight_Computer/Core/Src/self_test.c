/*
 * self_test.c
 *
 *  Created on: 31-Jul-2026
 *      Author: sumed
 */

#include "self_test.h"
#include "system_health.h"
#include "BMP280.h"
#include "MPU9250.h"
#include "MLX90393.h"
#include "sensor_data.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

void BMP280_SelfTest(void)
{
	uint8_t chip_id;

	if (BMP280_ReadID(&chip_id) != HAL_OK)
	{
	    current_health_state = CRITICAL_FAULT;
	    return;
	}

	if(chip_id != BMP280_CHIP_ID)
	{
	    current_health_state = CRITICAL_FAULT;
	    return;
	}

	if (BMP280_Init() != HAL_OK)
	{
	    current_health_state = FAULT;
	    return;
	}

	if(BMP280_LoadCalibration() != HAL_OK)
		{
		    current_health_state = FAULT;
		    return;
		}


	if (BMP280_ReadRaw(&sensor_data.temperature_raw, &sensor_data.pressure_raw) != HAL_OK)
	{
	    current_health_state = FAULT;
	    return;
	}

	current_health_state = NORMAL;
}

void MPU9250_SelfTest(void)
{
	uint8_t chip_id;

	if (MPU9250_Who_AM_I(&chip_id) != HAL_OK)
	{
	    current_health_state = CRITICAL_FAULT;
	    return;
	}

	if(chip_id != MPU9250_CHIP_ID)
	{
		current_health_state = CRITICAL_FAULT;
		return;
	}

	if (MPU9250_Init() != HAL_OK)
	{
	    current_health_state = FAULT;
	    return;
	}

	if (MPU9250_ReadRaw(&sensor_data.accel_raw_x, &sensor_data.accel_raw_y, &sensor_data.accel_raw_z, &sensor_data.gyro_raw_x, &sensor_data.gyro_raw_y, &sensor_data.gyro_raw_z, &sensor_data.temp_raw) != HAL_OK)
	{
	    current_health_state = FAULT;
	    return;
	}

	current_health_state = NORMAL;
}

void MLX90393_SelfTest(void)
{

	if (MLX90393_I2C_Test() != HAL_OK)
	{
	    current_health_state = CRITICAL_FAULT;
	    return;
	}

	if (MLX90393_Configure() != HAL_OK)
	{
	    current_health_state = FAULT;
	    return;
	}

	if (MLX90393_StartMeasurement() != HAL_OK)
	{
	    current_health_state = FAULT;
	    return;
	}

	current_health_state = NORMAL;
}
