/*
 * BMP280.c
 *
 *  Created on: 30-Jul-2026
 *      Author: sumed
 */
#include "main.h"
#include "BMP280.h"

extern SPI_HandleTypeDef hspi2;

static uint16_t dig_T1, dig_P1;
static int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

void BMP280_CS_Low(void)
{
    HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);
}

void BMP280_CS_High(void)
{
    HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef BMP280_ReadID(uint8_t *chip_id)
{
	HAL_StatusTypeDef status;

	BMP280_CS_Low();
	uint8_t reg = BMP280_REG_ID | 0x80; // Set MSB for read operation

	status = HAL_SPI_Transmit(&hspi2, &reg, 1, BMP280_TIMEOUT);

	if (status != HAL_OK)
	{
		BMP280_CS_High();
		return status;
	}

	status = HAL_SPI_Receive(&hspi2, chip_id, 1, BMP280_TIMEOUT);

	if (status != HAL_OK)
	{
		BMP280_CS_High();
		return status;
	}

	BMP280_CS_High();
	return status;

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
	uint8_t reg_ctrl = CTRL_MEAS & 0x7F; // Set MSB for write operation
	uint8_t reg_config = CONFIG & 0x7F; // Set MSB for write operation

	BMP280_CS_Low();

	status = HAL_SPI_Transmit(&hspi2, &reg_config, 1, BMP280_TIMEOUT);
	if(status != HAL_OK)
		{
		BMP280_CS_High();
		return status;
		}

	status = HAL_SPI_Transmit(&hspi2, &config, 1, BMP280_TIMEOUT);
	if(status != HAL_OK)
	{
		BMP280_CS_High();
		return status;
	}

	BMP280_CS_High();

	BMP280_CS_Low();
	status = HAL_SPI_Transmit(&hspi2, &reg_ctrl, 1, BMP280_TIMEOUT);
	if(status != HAL_OK)
		{
		BMP280_CS_High();
		return status;
		}

	status = HAL_SPI_Transmit(&hspi2, &ctrl_meas, 1, BMP280_TIMEOUT);
	if(status != HAL_OK)
	{
		BMP280_CS_High();
		return status;
	}

	BMP280_CS_High();
	return status;
}

HAL_StatusTypeDef BMP280_ReadRaw(int32_t *adc_T, int32_t *adc_P)
{
	HAL_StatusTypeDef status;
	uint8_t data[6];
	uint8_t reg = PRESSURE_DATA_START | 0x80; // Set MSB for read operation

	BMP280_CS_Low();
	status = HAL_SPI_Transmit(&hspi2, &reg, 1, BMP280_TIMEOUT);

	if (status != HAL_OK)
	{
		BMP280_CS_High();
		return status;
	}

	status = HAL_SPI_Receive(&hspi2, data, 6, BMP280_TIMEOUT);

	if (status != HAL_OK)
	{
		BMP280_CS_High();
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

	BMP280_CS_High();
	return status;
}

HAL_StatusTypeDef BMP280_LoadCalibration(void)
{
	uint8_t calib_data[24];
	HAL_StatusTypeDef status;
	uint8_t calib_reg = CALIBRATION_DATA_START | 0x80; // Set MSB for read operation

	BMP280_CS_Low();
	status = HAL_SPI_Transmit(&hspi2, &calib_reg, 1, BMP280_TIMEOUT);
	if (status != HAL_OK)
	{
		BMP280_CS_High();
		return status;
	}

	status = HAL_SPI_Receive(&hspi2, calib_data, 24, BMP280_TIMEOUT);
	if (status != HAL_OK)
	{
		BMP280_CS_High();
		return status;
	}

	dig_T1 = (uint16_t)(calib_data[0] | (calib_data[1] << 8));
	dig_T2 = (int16_t)(calib_data[2] | (calib_data[3] << 8));
	dig_T3 = (int16_t)(calib_data[4] | (calib_data[5] << 8));

	dig_P1 = (uint16_t)(calib_data[6] | (calib_data[7] << 8));
	dig_P2 = (int16_t)(calib_data[8] | (calib_data[9] << 8));
	dig_P3 = (int16_t)(calib_data[10] | (calib_data[11] << 8));
	dig_P4 = (int16_t)(calib_data[12] | (calib_data[13] << 8));
	dig_P5 = (int16_t)(calib_data[14] | (calib_data[15] << 8));
	dig_P6 = (int16_t)(calib_data[16] | (calib_data[17] << 8));
	dig_P7 = (int16_t)(calib_data[18] | (calib_data[19] << 8));
	dig_P8 = (int16_t)(calib_data[20] | (calib_data[21] << 8));
	dig_P9 = (int16_t)(calib_data[22] | (calib_data[23] << 8));

	BMP280_CS_High();
	return status;
}




// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t BMP280_CompensateTemp(int32_t adc_T)
{
	int32_t var1, var2, T;

	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;

	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

	t_fine = var1 + var2;

	T = (t_fine * 5 + 128) >> 8;

	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BMP280_CompensatePressure(int32_t adc_P)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}
