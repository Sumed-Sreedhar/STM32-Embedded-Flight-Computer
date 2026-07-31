/*
 * BMP280.c
 *
 *  Created on: 30-Jul-2026
 *      Author: sumed
 */
#include "main.h"
#include "BMP280.h"
#include "system_health.h"
#include "string.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

#define BMP280_ADDRESS (0x76 << 1)

uint8_t BMP280_FindAddress(void)
{
	if(HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDRESS, 3, 100) == HAL_OK)
	{
		return 0x76;
	}
	return 0x00;
}

HAL_StatusTypeDef BMP280_Init(void)
{
	uint8_t ctrl_meas =
	    (0b010 << 5) |
	    (0b011 << 2) |
	    (0b11);

	uint8_t config =
	    (0b000 << 5) |
	    (0b010 << 2);

	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDRESS, 0xF5, I2C_MEMADD_SIZE_8BIT, &config, 1, 100);
	if(status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDRESS, 0xF4, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, 100);
	if(status != HAL_OK)
	{
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef BMP280_ReadRaw(int32_t *adc_T, int32_t *adc_P)
{
	HAL_StatusTypeDef status;
	uint8_t data[6];

	status = HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, 0xF7, I2C_MEMADD_SIZE_8BIT, data, 6, 100);

	if (status != HAL_OK)
	{
		return status;
	}

	*adc_P =
	    ((int32_t)data[0] << 12) |
	    ((int32_t)data[1] << 4)  |
	    ((int32_t)data[2] >> 4);

	*adc_T =
	    ((int32_t)data[3] << 12) |
	    ((int32_t)data[4] << 4)  |
	    ((int32_t)data[5] >> 4);
	return status;
}


