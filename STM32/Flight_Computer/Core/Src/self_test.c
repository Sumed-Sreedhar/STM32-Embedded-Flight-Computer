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

volatile SelfTestState_t current_selftest_state = SELF_TEST_IDLE;
volatile BMP_TestResult_t bmp_test_result = BMP_OK;
volatile MPU_TestResult_t mpu_test_result = MPU_OK;
volatile MLX_TestResult_t mlx_test_result = MLX_OK;


void SelfTest(void)
{
	bmp_test_result = BMP_OK;
	mpu_test_result = MPU_OK;
	mlx_test_result = MLX_OK;

	uint8_t bmp_failed = 0;
	uint8_t mpu_failed = 0;
	uint8_t mlx_failed = 0;
	uint8_t critical_fault_flag = 0;
	uint8_t fault_flag = 0;

	/* BMP Self Test */

	current_selftest_state = SELF_TEST_BMP;
	uint8_t bmp_chip_id;

	if (BMP280_ReadID(&bmp_chip_id) != HAL_OK)
	{
		bmp_failed = 1;
		critical_fault_flag = 1;
	    bmp_test_result = BMP_READID_FAIL;
	}

	if(bmp_chip_id != BMP280_CHIP_ID && bmp_failed == 0)
	{
		bmp_failed = 1;
		critical_fault_flag = 1;
	    bmp_test_result = BMP_ID_FAIL;
	}

	if (BMP280_Init() != HAL_OK && bmp_failed == 0)
	{
		bmp_failed = 1;
		fault_flag = 1;
	    bmp_test_result = BMP_INIT_FAIL;
	}

	if(BMP280_LoadCalibration() != HAL_OK && bmp_failed == 0)
	{
		bmp_failed = 1;
		fault_flag = 1;
	    bmp_test_result = BMP_LOAD_FAIL;
	}


	if (BMP280_ReadRaw(&sensor_data.temperature_raw, &sensor_data.pressure_raw) != HAL_OK  && bmp_failed == 0)
	{
		bmp_failed = 1;
		fault_flag = 1;
	    bmp_test_result = BMP_READ_FAIL;
	}

	/* MPU Self Test */

	current_selftest_state = SELF_TEST_MPU;
	uint8_t mpu_chip_id;

	if (MPU9250_Who_AM_I(&mpu_chip_id) != HAL_OK)
	{
		mpu_failed = 1;
		critical_fault_flag = 1;
	    mpu_test_result = MPU_READID_FAIL;
	}

	if(mpu_chip_id != MPU9250_CHIP_ID && mpu_failed == 0)
	{
		mpu_failed = 1;
		critical_fault_flag = 1;
		mpu_test_result = MPU_ID_FAIL;
	}

	if (MPU9250_Init() != HAL_OK && mpu_failed == 0)
	{
		mpu_failed = 1;
		fault_flag =1;
	    mpu_test_result = MPU_INIT_FAIL;
	}

	if (MPU9250_ReadRaw(&sensor_data.accel_raw_x, &sensor_data.accel_raw_y, &sensor_data.accel_raw_z,
			&sensor_data.gyro_raw_x, &sensor_data.gyro_raw_y, &sensor_data.gyro_raw_z,
			&sensor_data.temp_raw) != HAL_OK && mpu_failed == 0)
	{
		mpu_failed = 1;
		fault_flag = 1;
	    mpu_test_result = MPU_READ_FAIL;
	}

	/* MLX Self Test */

	current_selftest_state = SELF_TEST_MLX;

	if (MLX90393_I2C_Test() != HAL_OK)
	{
		mlx_failed = 1;
		critical_fault_flag = 1;
	    mlx_test_result = MLX_I2C_FAIL;
	}

	if (MLX90393_Configure() != HAL_OK && mlx_failed == 0)
	{
		mlx_failed = 1;
		fault_flag = 1;
	    mlx_test_result = MLX_CONFIG_FAIL;
	}

	current_selftest_state = SELF_TEST_COMPLETE;

	if(critical_fault_flag)
	{
		current_health_state = CRITICAL_FAULT;
	}
	else if(fault_flag)
	{
		current_health_state = FAULT;
	}
	else
	{
		current_health_state = NORMAL;
	}

}


