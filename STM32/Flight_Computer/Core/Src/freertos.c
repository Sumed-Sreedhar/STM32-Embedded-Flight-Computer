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
static osMessageQueueId_t sensorFrameQueue;

/* Debugger-visible indicators that StateEstTask is receiving sensor frames. */
static sensor_frame_t estimator_latest_frame;
static volatile uint32_t estimator_frame_count;
static volatile float estimator_dt_seconds;

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
  sensorFrameQueue = osMessageQueueNew(1U, sizeof(sensor_frame_t), NULL);
  if (sensorFrameQueue == NULL)
  {
    Error_Handler();
  }
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
    sensor_frame_t frame = {0};
    sensor_frame_t discarded_frame;
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t now;
    uint32_t baro_last_tick = HAL_GetTick() - 40U;
    uint32_t mag_last_start_tick = HAL_GetTick() - 20U;
    uint32_t mag_start_tick = 0U;
    uint8_t mag_pending = 0U;
    HAL_StatusTypeDef status;

    (void)argument;

    for (;;)
    {
        /* IMU: sample once per 10 ms cycle. */
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
            frame.accel_g[0] = (float)sensor_data.accel_raw_x / 16384.0f;
            frame.accel_g[1] = (float)sensor_data.accel_raw_y / 16384.0f;
            frame.accel_g[2] = (float)sensor_data.accel_raw_z / 16384.0f;

            frame.gyro_dps[0] = (float)sensor_data.gyro_raw_x / 131.0f;
            frame.gyro_dps[1] = (float)sensor_data.gyro_raw_y / 131.0f;
            frame.gyro_dps[2] = (float)sensor_data.gyro_raw_z / 131.0f;

            frame.imu_tick_ms = HAL_GetTick();
            frame.valid_mask |= SENSOR_FRAME_IMU_VALID;
        }

        /* BMP280: sample at 25 Hz. */
        now = HAL_GetTick();
        if ((uint32_t)(now - baro_last_tick) >= 40U)
        {
            int32_t temperature_centi_c;
            uint32_t pressure_q24_8;

            baro_last_tick = now;
            status = BMP280_ReadRaw(
                &sensor_data.temperature_raw,
                &sensor_data.pressure_raw);

            SystemHealth_ReportSensor(SENSOR_BMP280, status);

            if (status == HAL_OK)
            {
                temperature_centi_c =
                    BMP280_CompensateTemp(sensor_data.temperature_raw);
                pressure_q24_8 =
                    BMP280_CompensatePressure(sensor_data.pressure_raw);

                frame.temperature_c = (float)temperature_centi_c / 100.0f;
                frame.pressure_pa = (float)pressure_q24_8 / 256.0f;
                sensor_data.temperature = frame.temperature_c;
                sensor_data.pressure = frame.pressure_pa;

                frame.baro_tick_ms = HAL_GetTick();
                frame.valid_mask |= SENSOR_FRAME_BARO_VALID;
            }
        }

        /* Finish an MLX90393 conversion without blocking the IMU schedule. */
        now = HAL_GetTick();
        if ((mag_pending != 0U) &&
            ((uint32_t)(now - mag_start_tick) >= 20U))
        {
            status = MLX90393_ReadMeasurement(
                &sensor_data.mag_raw_x,
                &sensor_data.mag_raw_y,
                &sensor_data.mag_raw_z);

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

                frame.mag_uT[0] = sensor_data.mag_x_uT;
                frame.mag_uT[1] = sensor_data.mag_y_uT;
                frame.mag_uT[2] = sensor_data.mag_z_uT;
                frame.mag_tick_ms = HAL_GetTick();
                frame.valid_mask |= SENSOR_FRAME_MAG_VALID;
            }

            mag_pending = 0U;
        }

        now = HAL_GetTick();
        if ((mag_pending == 0U) &&
            ((uint32_t)(now - mag_last_start_tick) >= 20U))
        {
            status = MLX90393_StartMeasurement();
            mag_last_start_tick = now;

            if (status == HAL_OK)
            {
                mag_pending = 1U;
                mag_start_tick = now;
            }
            else
            {
                SystemHealth_ReportSensor(SENSOR_MLX90393, status);
            }
        }

        /* Keep only the newest frame if StateEstTask falls behind. */
        if (osMessageQueuePut(sensorFrameQueue, &frame, 0U, 0U) != osOK)
        {
            (void)osMessageQueueGet(
                sensorFrameQueue, &discarded_frame, NULL, 0U);
            (void)osMessageQueuePut(sensorFrameQueue, &frame, 0U, 0U);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10U));
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
    sensor_frame_t frame;
    uint32_t previous_imu_tick_ms = 0U;

    (void)argument;

    for (;;)
    {
        if (osMessageQueueGet(
                sensorFrameQueue,
                &frame,
                NULL,
                osWaitForever) == osOK)
        {
            uint32_t now = HAL_GetTick();

            if (((frame.valid_mask & SENSOR_FRAME_IMU_VALID) != 0U) &&
                ((uint32_t)(now - frame.imu_tick_ms) <= 100U))
            {
                if (previous_imu_tick_ms != 0U)
                {
                    uint32_t elapsed_ms =
                        frame.imu_tick_ms - previous_imu_tick_ms;
                    estimator_dt_seconds = (float)elapsed_ms / 1000.0f;
                }

                previous_imu_tick_ms = frame.imu_tick_ms;
                estimator_latest_frame = frame;
                estimator_frame_count++;
            }
        }
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

