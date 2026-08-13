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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_TEMP_MSB    0xFA

#define BMP280_RESET_VALUE     0xB6

#define CS_BMP_Pin             GPIO_PIN_0
#define CS_BMP_GPIO_Port       GPIOC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;

uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

float temperature;
float pressure;

float t_fine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void BMP280_CS_Low(void)
{
    HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);
}

void BMP280_CS_High(void)
{
    HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);
}


HAL_StatusTypeDef BMP280_WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];

    tx[0] = reg & 0x7F;   // Write: bit 7 = 0
    tx[1] = data;

    BMP280_CS_Low();

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);

    BMP280_CS_High();

    return status;
}


HAL_StatusTypeDef BMP280_ReadRegisters(uint8_t reg,
                                       uint8_t *data,
                                       uint16_t length)
{
    uint8_t tx = reg | 0x80;   // Read: bit 7 = 1

    BMP280_CS_Low();

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY);

    if (status == HAL_OK)
    {
        status =
            HAL_SPI_Receive(&hspi2, data, length, HAL_MAX_DELAY);
    }

    BMP280_CS_High();

    return status;
}


uint8_t BMP280_ReadID(void)
{
    uint8_t id = 0;

    BMP280_ReadRegisters(BMP280_REG_ID, &id, 1);

    return id;
}


void BMP280_Reset(void)
{
    BMP280_WriteRegister(BMP280_REG_RESET, BMP280_RESET_VALUE);

    /*
     * Bosch specifies a short delay after software reset
     * before accessing the device again.
     */
    HAL_Delay(5);
}


void BMP280_ReadCalibration(void)
{
    uint8_t calib[24];

    BMP280_ReadRegisters(BMP280_REG_CALIB, calib, 24);

    /*
     * Temperature calibration
     */

    dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);

    dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);

    dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);


    /*
     * Pressure calibration
     */

    dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);

    dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);

    dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);

    dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);

    dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);

    dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);

    dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);

    dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);

    dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}


void BMP280_Configure(void)
{
    /*
     * ctrl_meas = 0x57
     *
     * osrs_t = 010 -> temperature oversampling x2
     * osrs_p = 101 -> pressure oversampling x16
     * mode   = 11  -> normal mode
     */

    BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, 0x57);


    /*
     * config = 0x68
     *
     * t_sb     = 011 -> standby 250 ms
     * filter   = 010 -> IIR filter coefficient 4
     * spi3w_en = 0   -> normal 4-wire SPI
     */

    BMP280_WriteRegister(BMP280_REG_CONFIG, 0x68);
}


void BMP280_ReadRaw(int32_t *raw_temp, int32_t *raw_pressure)
{
    uint8_t data[6];

    BMP280_ReadRegisters(BMP280_REG_PRESS_MSB, data, 6);

    /*
     * Pressure:
     *
     * 0xF7 = MSB
     * 0xF8 = LSB
     * 0xF9 = XLSB
     */

    *raw_pressure =
        ((int32_t)data[0] << 12) |
        ((int32_t)data[1] << 4)  |
        ((int32_t)data[2] >> 4);


    /*
     * Temperature:
     *
     * 0xFA = MSB
     * 0xFB = LSB
     * 0xFC = XLSB
     */

    *raw_temp =
        ((int32_t)data[3] << 12) |
        ((int32_t)data[4] << 4)  |
        ((int32_t)data[5] >> 4);
}


float BMP280_CompensateTemperature(int32_t adc_T)
{
    float var1;
    float var2;
    float T;

    var1 =
        (((float)adc_T / 16384.0f) -
         ((float)dig_T1 / 1024.0f))
        * (float)dig_T2;

    var2 =
        (((float)adc_T / 131072.0f) -
         ((float)dig_T1 / 8192.0f));

    var2 =
        var2 * var2 * (float)dig_T3;

    t_fine = var1 + var2;

    T = t_fine / 5120.0f;

    return T;
}


float BMP280_CompensatePressure(int32_t adc_P)
{
    float var1;
    float var2;
    float p;

    var1 = (t_fine / 2.0f) - 64000.0f;

    var2 = var1 * var1 * ((float)dig_P6 / 32768.0f);

    var2 =
        var2 +
        var1 * ((float)dig_P5 * 2.0f);

    var2 =
        (var2 / 4.0f) +
        ((float)dig_P4 * 65536.0f);

    var1 =
        ((float)dig_P3 * var1 * var1 / 524288.0f +
         (float)dig_P2 * var1) / 524288.0f;

    var1 =
        (1.0f + var1 / 32768.0f) *
        (float)dig_P1;

    if (var1 == 0.0f)
    {
        return 0.0f;
    }

    p = 1048576.0f - (float)adc_P;

    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;

    var1 =
        (float)dig_P9 * p * p / 2147483648.0f;

    var2 =
        p * ((float)dig_P8 / 32768.0f);

    p =
        p +
        (var1 + var2 + (float)dig_P7) / 16.0f;

    return p;
}


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)ptr,
                      len,
                      HAL_MAX_DELAY);

    return len;
}

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


  uint8_t id;

  int32_t raw_temperature;
  int32_t raw_pressure;


  /*
   * CS must be HIGH when the sensor is idle.
   */

  BMP280_CS_High();

  HAL_Delay(100);


  /*
   * Check chip ID
   */

  printf("\r\n=============================\r\n");
  printf("BMP280 SPI TEST\r\n");
  printf("=============================\r\n");

  id = BMP280_ReadID();

  printf("CHIP ID = 0x%02X\r\n", id);

  if (id != 0x58)
  {
      printf("BMP280 NOT DETECTED!\r\n");

      while (1)
      {
          HAL_Delay(1000);
      }
  }

  printf("BMP280 detected successfully!\r\n");


  /*
   * Software reset
   */

  printf("Resetting BMP280...\r\n");

  BMP280_Reset();


  /*
   * Read factory calibration coefficients
   */

  printf("Reading calibration data...\r\n");

  BMP280_ReadCalibration();


  printf("\r\nCalibration coefficients:\r\n");

  printf("T1 = %u\r\n", dig_T1);
  printf("T2 = %d\r\n", dig_T2);
  printf("T3 = %d\r\n", dig_T3);

  printf("P1 = %u\r\n", dig_P1);
  printf("P2 = %d\r\n", dig_P2);
  printf("P3 = %d\r\n", dig_P3);
  printf("P4 = %d\r\n", dig_P4);
  printf("P5 = %d\r\n", dig_P5);
  printf("P6 = %d\r\n", dig_P6);
  printf("P7 = %d\r\n", dig_P7);
  printf("P8 = %d\r\n", dig_P8);
  printf("P9 = %d\r\n", dig_P9);


  /*
   * Configure measurement mode.
   */

  printf("\r\nConfiguring BMP280...\r\n");

  BMP280_Configure();


  /*
   * Read back the control registers.
   * This verifies that our SPI writes actually worked.
   */

  uint8_t ctrl_meas;
  uint8_t config;

  BMP280_ReadRegisters(BMP280_REG_CTRL_MEAS, &ctrl_meas, 1);
  BMP280_ReadRegisters(BMP280_REG_CONFIG, &config, 1);

  printf("CTRL_MEAS = 0x%02X\r\n", ctrl_meas);
  printf("CONFIG    = 0x%02X\r\n", config);

  printf("\r\nStarting measurements...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  BMP280_ReadRaw(&raw_temperature, &raw_pressure);

	      temperature =
	          BMP280_CompensateTemperature(raw_temperature);

	      pressure =
	          BMP280_CompensatePressure(raw_pressure);


	      printf("RAW T = %ld | RAW P = %ld | ",
	             raw_temperature,
	             raw_pressure);

	      printf("Temperature = %.2f C | ",
	             temperature);

	      printf("Pressure = %.2f Pa | ",
	             pressure);

	      printf("%.2f hPa\r\n",
	             pressure / 100.0f);


	      HAL_Delay(1000);

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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_BMP_Pin */
  GPIO_InitStruct.Pin = CS_BMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_BMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
