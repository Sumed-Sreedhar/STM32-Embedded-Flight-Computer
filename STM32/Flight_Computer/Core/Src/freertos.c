/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "system_health.h"
#include "system_state.h"
#include "BMP280.h"
#include "MPU9250.h"
#include "MLX90393.h"
#include "sensor_data.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for StateEstTask */
osThreadId_t StateEstTaskHandle;
const osThreadAttr_t StateEstTask_attributes = {
  .name = "StateEstTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for FaultManageTask */
osThreadId_t FaultManageTaskHandle;
const osThreadAttr_t FaultManageTask_attributes = {
  .name = "FaultManageTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for LoggerTask */
osThreadId_t LoggerTaskHandle;
const osThreadAttr_t LoggerTask_attributes = {
  .name = "LoggerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartSensorTask(void *argument);
void StartStateEstTask(void *argument);
void StartGPSTask(void *argument);
void StartFaultManageTask(void *argument);
void StartLoggerTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of StateEstTask */
  StateEstTaskHandle = osThreadNew(StartStateEstTask, NULL, &StateEstTask_attributes);

  /* creation of GPSTask */
  GPSTaskHandle = osThreadNew(StartGPSTask, NULL, &GPSTask_attributes);

  /* creation of FaultManageTask */
  FaultManageTaskHandle = osThreadNew(StartFaultManageTask, NULL, &FaultManageTask_attributes);

  /* creation of LoggerTask */
  LoggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &LoggerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {
	  osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */

    int32_t temp_c;
    uint32_t pressure_pa;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temp_raw;

    uint32_t BMP_last_transmit_time = 0;
    uint32_t MPU_last_transmit_time = 0;
    uint32_t MLX_last_transmit_time = 0;

    char BMP_data[300];
    char mpu_data[300];
    char mlx_data[300];


  /* Infinite loop */
  for(;;)
  {
      /* Read BMP280 */
      BMP280_ReadRaw(&sensor_data.temperature_raw,
                     &sensor_data.pressure_raw);

      temp_c = BMP280_CompensateTemp(sensor_data.temperature_raw);

      pressure_pa =
          BMP280_CompensatePressure(sensor_data.pressure_raw) / 256;

      sprintf(BMP_data,
              "\r\nBMP280 Temperature: %ld.%02ld C, Pressure: %lu Pa\r\n"
              "BMP280_Raw_temp: %lu, BMP280_Raw_Pressure: %lu\r\n",
              (long)(temp_c / 100),
              (long)(temp_c % 100),
              (unsigned long)pressure_pa,
              (unsigned long)sensor_data.temperature_raw,
              (unsigned long)sensor_data.pressure_raw);

      if (HAL_GetTick() - BMP_last_transmit_time >= 1000)
      {
          HAL_UART_Transmit(&huart2,
                            (uint8_t *)BMP_data,
                            strlen(BMP_data),
                            HAL_MAX_DELAY);

          BMP_last_transmit_time = HAL_GetTick();
      }

      /* Read MPU9250 */
      MPU9250_ReadRaw(&sensor_data.accel_raw_x,
                      &sensor_data.accel_raw_y,
                      &sensor_data.accel_raw_z,
                      &sensor_data.gyro_raw_x,
                      &sensor_data.gyro_raw_y,
                      &sensor_data.gyro_raw_z,
                      &sensor_data.temp_raw);

      accel_x = sensor_data.accel_raw_x / 16384.0f;
      accel_y = sensor_data.accel_raw_y / 16384.0f;
      accel_z = sensor_data.accel_raw_z / 16384.0f;

      gyro_x = sensor_data.gyro_raw_x / 131.0f;
      gyro_y = sensor_data.gyro_raw_y / 131.0f;
      gyro_z = sensor_data.gyro_raw_z / 131.0f;

      temp_raw = (sensor_data.temp_raw / 333.87f) + 21.0f;

      sprintf(mpu_data,
              "\r\nAccel_raw X: %d, Y: %d, Z: %d\r\n"
              "Gyro_raw X: %d, Y: %d, Z: %d\r\n"
              "Temp_raw: %d\r\n"
              "Accel X: %.2f, Y: %.2f, Z: %.2f\r\n"
              "Gyro X: %.2f, Y: %.2f, Z: %.2f\r\n"
              "Temp: %.2f C\r\n",
              sensor_data.accel_raw_x,
              sensor_data.accel_raw_y,
              sensor_data.accel_raw_z,
              sensor_data.gyro_raw_x,
              sensor_data.gyro_raw_y,
              sensor_data.gyro_raw_z,
              sensor_data.temp_raw,
              (double)accel_x,
              (double)accel_y,
              (double)accel_z,
              (double)gyro_x,
              (double)gyro_y,
              (double)gyro_z,
              (double)temp_raw);

      if (HAL_GetTick() - MPU_last_transmit_time >= 1000)
      {
          HAL_UART_Transmit(&huart2,
                            (uint8_t *)mpu_data,
                            strlen(mpu_data),
                            HAL_MAX_DELAY);

          MPU_last_transmit_time = HAL_GetTick();
      }

      /* Read MLX90393 */
      MLX90393_StartMeasurement();

      MLX90393_ReadMeasurement(&sensor_data.mag_raw_x,
                               &sensor_data.mag_raw_y,
                               &sensor_data.mag_raw_z);

      MLX90393_RawTo_uT(&sensor_data.mag_raw_x,
                        &sensor_data.mag_raw_y,
                        &sensor_data.mag_raw_z,
                        &sensor_data.mag_x_uT,
                        &sensor_data.mag_y_uT,
                        &sensor_data.mag_z_uT);

      sprintf(mlx_data,
              "\r\nMag_raw X: %d, Y: %d, Z: %d\r\n"
              "Mag X: %.2f uT, Y: %.2f uT, Z: %.2f uT\r\n",
              sensor_data.mag_raw_x,
              sensor_data.mag_raw_y,
              sensor_data.mag_raw_z,
              (double)sensor_data.mag_x_uT,
              (double)sensor_data.mag_y_uT,
              (double)sensor_data.mag_z_uT);

      if (HAL_GetTick() - MLX_last_transmit_time >= 1000)
      {
          HAL_UART_Transmit(&huart2,
                            (uint8_t *)mlx_data,
                            strlen(mlx_data),
                            HAL_MAX_DELAY);

          MLX_last_transmit_time = HAL_GetTick();
      }

      osDelay(1);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartStateEstTask */
/**
* @brief Function implementing the StateEstTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStateEstTask */
void StartStateEstTask(void *argument)
{
  /* USER CODE BEGIN StartStateEstTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartStateEstTask */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the GPSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument)
{
  /* USER CODE BEGIN StartGPSTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartFaultManageTask */
/**
* @brief Function implementing the FaultManageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFaultManageTask */
void StartFaultManageTask(void *argument)
{
  /* USER CODE BEGIN StartFaultManageTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFaultManageTask */
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the LoggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartLoggerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLoggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

