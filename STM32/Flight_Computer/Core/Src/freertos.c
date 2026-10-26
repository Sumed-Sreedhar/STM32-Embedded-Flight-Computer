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
#include "button.h"

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
  .stack_size = 128 * 4,
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
	defaultTaskHandle = osThreadNew(
	    StartDefaultTask, NULL, &defaultTask_attributes);
	if (defaultTaskHandle == NULL)
	{
	    Error_Handler();
	}

  /* creation of SensorTask */
	SensorTaskHandle = osThreadNew(
	    StartSensorTask, NULL, &SensorTask_attributes);
	if (SensorTaskHandle == NULL)
	{
	    Error_Handler();
	}

  /* creation of StateEstTask */
	StateEstTaskHandle = osThreadNew(
	    StartStateEstTask, NULL, &StateEstTask_attributes);
	if (StateEstTaskHandle == NULL)
	{
	    Error_Handler();
	}

  /* creation of GPSTask */
	GPSTaskHandle = osThreadNew(
	    StartGPSTask, NULL, &GPSTask_attributes);
	if (GPSTaskHandle == NULL)
	{
	    Error_Handler();
	}

  /* creation of FaultManageTask */
	FaultManageTaskHandle = osThreadNew(
	    StartFaultManageTask, NULL, &FaultManageTask_attributes);
	if (FaultManageTaskHandle == NULL)
	{
	    Error_Handler();
	}

  /* creation of LoggerTask */
	LoggerTaskHandle = osThreadNew(
	    StartLoggerTask, NULL, &LoggerTask_attributes);
	if (LoggerTaskHandle == NULL)
	{
	    Error_Handler();
	}

	if (xPortGetFreeHeapSize() < 256U)
	{
	    Error_Handler();
	}
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

    int32_t temp_centi_c;
    uint32_t pressure_q24_8;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mpu_temperature_c;

    uint32_t bmp_last_transmit_time = HAL_GetTick();
    uint32_t mpu_last_transmit_time = HAL_GetTick();
    uint32_t mlx_last_transmit_time = HAL_GetTick();
    uint32_t resource_report_start = HAL_GetTick();
    uint8_t resources_reported = 0U;

    char bmp_data[180];
    char mpu_data[240];
    char mlx_data[160];
    char resource_data[220];

  /* Infinite loop */
  for(;;)
  {
      HAL_StatusTypeDef status;
       uint32_t now;
       int length;

       /* BMP280 */
       status = BMP280_ReadRaw(
           &sensor_data.temperature_raw,
           &sensor_data.pressure_raw);

       SystemHealth_ReportSensor(SENSOR_BMP280, status);

       if (status == HAL_OK)
       {
           temp_centi_c =
               BMP280_CompensateTemp(sensor_data.temperature_raw);

           pressure_q24_8 =
               BMP280_CompensatePressure(sensor_data.pressure_raw);

           sensor_data.temperature = (float)temp_centi_c / 100.0f;
           sensor_data.pressure = (float)pressure_q24_8 / 256.0f;

           now = HAL_GetTick();
           if ((uint32_t)(now - bmp_last_transmit_time) >= 1000U)
           {
               length = snprintf(
                   bmp_data,
                   sizeof(bmp_data),
                   "\r\nBMP280 Temperature: %.2f C, Pressure: %.2f Pa\r\n"
                   "BMP280 raw temp: %ld, raw pressure: %ld\r\n",
                   (double)sensor_data.temperature,
                   (double)sensor_data.pressure,
                   (long)sensor_data.temperature_raw,
                   (long)sensor_data.pressure_raw);

               if (length > 0)
               {
                   if ((size_t)length >= sizeof(bmp_data))
                   {
                       length = (int)sizeof(bmp_data) - 1;
                   }

                   HAL_UART_Transmit(
                       &huart2,
                       (uint8_t *)bmp_data,
                       (uint16_t)length,
                       HAL_MAX_DELAY);
               }

               bmp_last_transmit_time = now;
           }
       }

       /* MPU9250 */
       status = MPU9250_ReadRaw(
           &sensor_data.accel_raw_x,
           &sensor_data.accel_raw_y,
           &sensor_data.accel_raw_z,
           &sensor_data.gyro_raw_x,
           &sensor_data.gyro_raw_y,
           &sensor_data.gyro_raw_z,
           &sensor_data.temp_raw);

       SystemHealth_ReportSensor(SENSOR_MPU9250, status);

       if (status == HAL_OK)
       {
           accel_x = (float)sensor_data.accel_raw_x / 16384.0f;
           accel_y = (float)sensor_data.accel_raw_y / 16384.0f;
           accel_z = (float)sensor_data.accel_raw_z / 16384.0f;

           gyro_x = (float)sensor_data.gyro_raw_x / 131.0f;
           gyro_y = (float)sensor_data.gyro_raw_y / 131.0f;
           gyro_z = (float)sensor_data.gyro_raw_z / 131.0f;

           mpu_temperature_c =
               ((float)sensor_data.temp_raw / 333.87f) + 21.0f;

           now = HAL_GetTick();
           if ((uint32_t)(now - mpu_last_transmit_time) >= 1000U)
           {
               length = snprintf(
                   mpu_data,
                   sizeof(mpu_data),
                   "\r\nMPU raw accel: %d, %d, %d\r\n"
                   "MPU raw gyro: %d, %d, %d\r\n"
                   "MPU accel g: %.2f, %.2f, %.2f\r\n"
                   "MPU gyro dps: %.2f, %.2f, %.2f\r\n"
                   "MPU temperature: %.2f C\r\n",
                   sensor_data.accel_raw_x,
                   sensor_data.accel_raw_y,
                   sensor_data.accel_raw_z,
                   sensor_data.gyro_raw_x,
                   sensor_data.gyro_raw_y,
                   sensor_data.gyro_raw_z,
                   (double)accel_x,
                   (double)accel_y,
                   (double)accel_z,
                   (double)gyro_x,
                   (double)gyro_y,
                   (double)gyro_z,
                   (double)mpu_temperature_c);

               if (length > 0)
               {
                   if ((size_t)length >= sizeof(mpu_data))
                   {
                       length = (int)sizeof(mpu_data) - 1;
                   }

                   HAL_UART_Transmit(
                       &huart2,
                       (uint8_t *)mpu_data,
                       (uint16_t)length,
                       HAL_MAX_DELAY);
               }

               mpu_last_transmit_time = now;
           }
       }

       /* MLX90393 */
       status = MLX90393_StartMeasurement();

       if (status == HAL_OK)
       {
           osDelay(20U);

           status = MLX90393_ReadMeasurement(
               &sensor_data.mag_raw_x,
               &sensor_data.mag_raw_y,
               &sensor_data.mag_raw_z);
       }

       SystemHealth_ReportSensor(SENSOR_MLX90393, status);

       if (status == HAL_OK)
       {
           MLX90393_RawTo_uT(
               &sensor_data.mag_raw_x,
               &sensor_data.mag_raw_y,
               &sensor_data.mag_raw_z,
               &sensor_data.mag_x_uT,
               &sensor_data.mag_y_uT,
               &sensor_data.mag_z_uT);

           now = HAL_GetTick();
           if ((uint32_t)(now - mlx_last_transmit_time) >= 1000U)
           {
               length = snprintf(
                   mlx_data,
                   sizeof(mlx_data),
                   "\r\nMLX raw: %d, %d, %d\r\n"
                   "MLX magnetic field: %.2f, %.2f, %.2f uT\r\n",
                   sensor_data.mag_raw_x,
                   sensor_data.mag_raw_y,
                   sensor_data.mag_raw_z,
                   (double)sensor_data.mag_x_uT,
                   (double)sensor_data.mag_y_uT,
                   (double)sensor_data.mag_z_uT);

               if (length > 0)
               {
                   if ((size_t)length >= sizeof(mlx_data))
                   {
                       length = (int)sizeof(mlx_data) - 1;
                   }

                   HAL_UART_Transmit(
                       &huart2,
                       (uint8_t *)mlx_data,
                       (uint16_t)length,
                       HAL_MAX_DELAY);
               }

               mlx_last_transmit_time = now;
           }
       }

       now = HAL_GetTick();
       if ((resources_reported == 0U) &&
           ((uint32_t)(now - resource_report_start) >= 2000U))
       {
           length = snprintf(
               resource_data,
               sizeof(resource_data),
               "\r\nRTOS free heap: %lu bytes\r\n"
               "Stack headroom (words): Sensor=%lu State=%lu GPS=%lu "
               "Fault=%lu Logger=%lu Default=%lu\r\n",
               (unsigned long)xPortGetFreeHeapSize(),
               (unsigned long)uxTaskGetStackHighWaterMark(SensorTaskHandle),
               (unsigned long)uxTaskGetStackHighWaterMark(StateEstTaskHandle),
               (unsigned long)uxTaskGetStackHighWaterMark(GPSTaskHandle),
               (unsigned long)uxTaskGetStackHighWaterMark(FaultManageTaskHandle),
               (unsigned long)uxTaskGetStackHighWaterMark(LoggerTaskHandle),
               (unsigned long)uxTaskGetStackHighWaterMark(defaultTaskHandle));

           if (length > 0)
           {
               if ((size_t)length >= sizeof(resource_data))
               {
                   length = (int)sizeof(resource_data) - 1;
               }

               HAL_UART_Transmit(
                   &huart2,
                   (uint8_t *)resource_data,
                   (uint16_t)length,
                   HAL_MAX_DELAY);
           }

           resources_reported = 1U;
       }

       osDelay(1U);
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

	uint32_t button_events;

  /* Infinite loop */
  for(;;)
  {
	  button_events = GPIO_Button_TakePendingEvents();

      if (button_events != 0U)
      {
          GPIO_Button_ProcessEvents(button_events);
      }

      SystemHealth_Update();
      handle_system_health();
      handle_system_state();

      osDelay(100U);
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

