/*
 * MPU9250.c
 *
 *  Created on: 01-Aug-2026
 *      Author: sumed
 */


#include "MPU9250.h"
#include "main.h"
#include "sensor_data.h"

extern SPI_HandleTypeDef hspi2;


void MPU9250_CS_Low(void)
{
	HAL_GPIO_WritePin(CS_MPU_GPIO_Port, CS_MPU_Pin, GPIO_PIN_RESET);
}

void MPU9250_CS_High(void)
{
	HAL_GPIO_WritePin(CS_MPU_GPIO_Port, CS_MPU_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef MPU9250_Who_AM_I(uint8_t *chip_id)
{
	HAL_StatusTypeDef status;

	MPU9250_CS_Low();
	uint8_t reg = MPU9250_WHO_AM_I | 0x80; // Set MSB for read operation

	status = HAL_SPI_Transmit(&hspi2, &reg, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Receive(&hspi2, chip_id, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
	    MPU9250_CS_High();
	    return status;
	}

	MPU9250_CS_High();
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
	uint8_t reg_pwr_mgmt = MPU9250_PWR_MGMT_1 & 0x7F; // Set MSB for write operation
	uint8_t reg_pwr_mgmt_2 = MPU9250_PWR_MGMT_2 & 0x7F; // Set MSB for write operation
	uint8_t reg_config = MPU9250_CONFIG & 0x7F; // Set MSB for write operation
	uint8_t reg_smplrt_div = MPU9250_SMPLRT_DIV & 0x7F; // Set MSB for write operation
	uint8_t reg_gyro_config = MPU9250_GYRO_CONFIG & 0x7F; // Set MSB for write operation
	uint8_t reg_accel_config = MPU9250_ACCEL_CONFIG & 0x7F; // Set MSB for write operation
	uint8_t reg_accel_config2 = MPU9250_ACCEL_CONFIG2 & 0x7F; // Set MSB for write operation
	uint8_t reg_int_enable = MPU9250_INT_ENABLE & 0x7F; // Set MSB for write operation

	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_pwr_mgmt, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &PWR_MGMT_1, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	MPU9250_CS_High();
	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_pwr_mgmt_2, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &PWR_MGMT_2, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	MPU9250_CS_High();
	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_config, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &CONFIG, 1, MPU9250_TIMEOUT);

	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	MPU9250_CS_High();
	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_smplrt_div, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &SMPLRT_DIV, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	MPU9250_CS_High();
	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_gyro_config, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &GYRO_CONFIG, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	MPU9250_CS_High();
	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_accel_config, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &ACCEL_CONFIG, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	MPU9250_CS_High();
	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_accel_config2, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &ACCEL_CONFIG2, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	MPU9250_CS_High();
	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_int_enable, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Transmit(&hspi2, &INT_ENABLE, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}


	MPU9250_CS_High();
	return status;}

HAL_StatusTypeDef MPU9250_ReadRaw(int16_t *accel_raw_x,int16_t *accel_raw_y,int16_t *accel_raw_z, int16_t *gyro_raw_x,int16_t *gyro_raw_y, int16_t *gyro_raw_z, int16_t *temp_raw)
{

	uint8_t data[14];
	uint8_t reg = MPU9250_ACCEL_XOUT_H | 0x80; // Set MSB for read operation
	HAL_StatusTypeDef status;

	MPU9250_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg, 1, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
		return status;
	}

	status = HAL_SPI_Receive(&hspi2, data, 14, MPU9250_TIMEOUT);
	if (status != HAL_OK)
	{
		MPU9250_CS_High();
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

	MPU9250_CS_High();
	return HAL_OK;
}
