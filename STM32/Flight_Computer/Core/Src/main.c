/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_health.h"
#include "system_state.h"
#include "self_test.h"
#include "button.h"
#include "led.h"
#include "BMP280.h"
#include "MPU9250.h"
#include "MLX90393.h"
#include "sensor_data.h"
#include "string.h"
#include "stdio.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  char msg[] = "MAIN STARTED\r\n";
  HAL_UART_Transmit(&huart2,
                    (uint8_t *)msg,
                    strlen(msg),
                    HAL_MAX_DELAY);

  current_system_state = SELF_TEST;

  BMP280_SelfTest();
  MPU9250_SelfTest();
  MLX90393_SelfTest();

  /* BM280 Variables */
  int32_t temp_c;
  uint32_t pressure_pa;

  /* MPU9250 Variables */
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float temp_raw;

  uint32_t BMP_last_transmit_time = 0;
  uint32_t MPU_last_transmit_time =0;
  uint32_t MLX_last_transmit_time =0;

  char BMP_data[300];
  char mpu_data[300];
  char mlx_data[300];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  handle_system_state();
	  handle_system_health();

	  /* Read temperature and pressure data from BMP280 sensor */
	  BMP280_ReadRaw(&sensor_data.temperature_raw, &sensor_data.pressure_raw);
	  temp_c = BMP280_CompensateTemp(sensor_data.temperature_raw);
	  pressure_pa = BMP280_CompensatePressure(sensor_data.pressure_raw)/256;

	  sprintf(BMP_data, "\r\n BMP280 Temperature: %ld.%02ld C, Pressure: %lu Pa\r\n"
			  "\r\n BMP280_Raw_temp: %lu, BMP280_Raw_Pressure: %lu \r\n",
	          temp_c / 100,
	          temp_c % 100,
	          pressure_pa,
			  sensor_data.temperature_raw,
			  sensor_data.pressure_raw);

	  if(HAL_GetTick() - BMP_last_transmit_time >= 1000) // Transmit every 1 second
	  {
		  HAL_UART_Transmit(&huart2, (uint8_t *)BMP_data, strlen(BMP_data) , HAL_MAX_DELAY);
		  BMP_last_transmit_time = HAL_GetTick();
	  }

	  /* Read IMU data from MPU9250 sensor */

	  MPU9250_ReadRaw(&sensor_data.accel_raw_x, &sensor_data.accel_raw_y, &sensor_data.accel_raw_z,
			  &sensor_data.gyro_raw_x, &sensor_data.gyro_raw_y, &sensor_data.gyro_raw_z,
			  &sensor_data.temp_raw);

	  accel_x = sensor_data.accel_raw_x/16384.0f;
	  accel_y = sensor_data.accel_raw_y/16384.0f;
	  accel_z = sensor_data.accel_raw_z/16384.0f;
	  gyro_x = sensor_data.gyro_raw_x/131.0f;
	  gyro_y = sensor_data.gyro_raw_y/131.0f;
	  gyro_z = sensor_data.gyro_raw_z/131.0f;
	  temp_raw = (sensor_data.temp_raw/333.87) + 21; // Convert raw temperature to degrees Celsius

	  sprintf(mpu_data, "\r\n Accel_raw X: %d, Accel_raw Y: %d, Accel_raw Z: %d,"
	          "\r\n Gyro_raw X: %d, Gyro_raw Y: %d, Gyro_raw Z: %d,"
	          " \r\n Temp_raw: %d,"
	          "\r\n Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f,"
	          " \r\n Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f,"
	          " \r\n Temp: %.2f C\r\n",
			  sensor_data.accel_raw_x, sensor_data.accel_raw_y, sensor_data.accel_raw_z,
			  sensor_data.gyro_raw_x, sensor_data.gyro_raw_y, sensor_data.gyro_raw_z,
			  sensor_data.temp_raw,
			  accel_x, accel_y, accel_z,
			  gyro_x, gyro_y, gyro_z,
			  temp_raw);

	  if(HAL_GetTick() - MPU_last_transmit_time >= 1000) // Transmit every 1 second
	  {
	  	  HAL_UART_Transmit(&huart2, (uint8_t *)mpu_data, strlen(mpu_data) , HAL_MAX_DELAY);
	  	  MPU_last_transmit_time = HAL_GetTick();
	  }

	  /* Read magnetometer data from MLX90393 sensor */

	  MLX90393_ReadMeasurement(&sensor_data.mag_raw_x, &sensor_data.mag_raw_y, &sensor_data.mag_raw_z);

	  sprintf(mlx_data, "\r\n Mag_raw X: %d, Mag_raw Y: %d, Mag_raw Z: %d\r\n",
			  sensor_data.mag_raw_x, sensor_data.mag_raw_y, sensor_data.mag_raw_z);

	  MLX90393_RawTo_uT(&sensor_data.mag_raw_x, &sensor_data.mag_raw_y, &sensor_data.mag_raw_z,
			  &sensor_data.mag_x_uT,
			  &sensor_data.mag_y_uT,
			  &sensor_data.mag_z_uT);

	  sprintf(mlx_data + strlen(mlx_data), "Mag X: %.2f uT, Mag Y: %.2f uT, Mag Z: %.2f uT\r\n",
			  sensor_data.mag_x_uT, sensor_data.mag_y_uT, sensor_data.mag_z_uT);

	  if(HAL_GetTick() - MLX_last_transmit_time >= 1000) // Transmit every 1 second
	  {
		  HAL_UART_Transmit(&huart2, (uint8_t *)mlx_data, strlen(mlx_data) , HAL_MAX_DELAY);
		  MLX_last_transmit_time = HAL_GetTick();
	  }



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
