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
#include "sensor_data.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

void BMP280_SelfTest(void)
{
	if (BMP280_FindAddress() != 0x76)
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
	if (MPU9250_FindAddress() != 0x68)
	{
	    HAL_UART_Transmit(&huart2, (uint8_t*)"MPU ADDRESS FAIL\r\n", 18, HAL_MAX_DELAY);
	    current_health_state = CRITICAL_FAULT;
	    return;
	}

	if (MPU9250_Init() != HAL_OK)
	{
	    HAL_UART_Transmit(&huart2, (uint8_t*)"MPU INIT FAIL\r\n", 15, HAL_MAX_DELAY);
	    current_health_state = FAULT;
	    return;
	}

	//if (MPU9250_Who_AM_I() != HAL_OK)
	{
	    HAL_UART_Transmit(&huart2, (uint8_t*)"MPU WHOAMI FAIL\r\n", 17, HAL_MAX_DELAY);
	    current_health_state = FAULT;
	    return;
	}

	if (MPU9250_ReadRaw(&sensor_data.accel_raw_x, &sensor_data.accel_raw_y, &sensor_data.accel_raw_z, &sensor_data.gyro_raw_x, &sensor_data.gyro_raw_y, &sensor_data.gyro_raw_z, &sensor_data.temp_raw) != HAL_OK)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"MPU RAW READ FAIL\r\n", 19, HAL_MAX_DELAY);
	    current_health_state = FAULT;
	    return;
	}

	current_health_state = NORMAL;
}
