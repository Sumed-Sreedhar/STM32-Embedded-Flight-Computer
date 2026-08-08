/*
 * MPU9250.c
 *
 *  Created on: 01-Aug-2026
 *      Author: sumed
 */


#include "MPU9250.h"
#include "main.h"
#include "sensor_data.h"

extern I2C_HandleTypeDef hi2c1;

#define MPU9250_ADDRESS (0x68 << 1)
#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_INT_ENABLE       0x38
#define MPU9250_PWR_MGMT_1       0x6B
#define MPU9250_PWR_MGMT_2       0x6C
#define MPU9250_WHO_AM_I         0x75
#define MPU9250_ACCEL_XOUT_H     0x3B
#define MPU9250_TIMEOUT 100

uint8_t MPU9250_FindAddress(void){
	if (HAL_I2C_IsDeviceReady(&hi2c1, MPU9250_ADDRESS, 3, 100) == HAL_OK)	{		return 0x68;	}	return 0x00;}

HAL_StatusTypeDef MPU9250_Who_AM_I(void)
{
	uint8_t who_am_i;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, MPU9250_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		return status;
	}

	if (who_am_i != 0x71)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef MPU9250_Init(void){
	uint8_t PWR_MGMT_1 = 0x01; // Set clock source to be PLL with x-axis gyroscope reference and disable sleep mode
	uint8_t PWR_MGMT_2 = 0x00; // Enable all axes
	uint8_t CONFIG = 0x03; // 41 Hz DLPF chosen for 100 Hz sensor task and Mahony filter
	uint8_t SMPLRT_DIV = 0x09; // 100 Hz sample rate
	uint8_t GYRO_CONFIG = 0x00; // 250 dps full scale
	uint8_t ACCEL_CONFIG = 0x00; // 2g full scale
	uint8_t ACCEL_CONFIG2 = 0x03; // 41 Hz DLPF chosen for 100 Hz sensor task and Mahony filter
	uint8_t INT_ENABLE = 0x00; // Disable interrupts

	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &PWR_MGMT_1, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &PWR_MGMT_2, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_CONFIG, I2C_MEMADD_SIZE_8BIT, &CONFIG, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &SMPLRT_DIV, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &GYRO_CONFIG, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &ACCEL_CONFIG, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, &ACCEL_CONFIG2, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &INT_ENABLE, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}

	return HAL_OK;}

HAL_StatusTypeDef MPU9250_ReadRaw(int16_t *accel_raw_x,int16_t *accel_raw_y,int16_t *accel_raw_z, int16_t *gyro_raw_x,int16_t *gyro_raw_y, int16_t *gyro_raw_z, int16_t *temp_raw)
{

	uint8_t data[14];
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 14, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		return status;
	}
	// Accelerometer raw values
	*accel_raw_x = (int16_t)((data[0] << 8) | data[1]);
	*accel_raw_y = (int16_t)((data[2] << 8) | data[3]);
	*accel_raw_z = (int16_t)((data[4] << 8) | data[5]);

	// Temperature raw value
	*temp_raw = (int16_t)((data[6] << 8) | data[7]);

	// Gyroscope raw values
	*gyro_raw_x = (int16_t)((data[8] << 8) | data[9]);
	*gyro_raw_y = (int16_t)((data[10] << 8) | data[11]);
	*gyro_raw_z = (int16_t)((data[12] << 8) | data[13]);

	return HAL_OK;
}
