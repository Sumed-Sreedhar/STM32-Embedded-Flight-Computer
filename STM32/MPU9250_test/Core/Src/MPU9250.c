/*
 * MPU9250.c
 *
 *  Created on: 01-Aug-2026
 *      Author: sumed
 */

#include "MPU9250.h"
#include "main.h"
#include "sensor_data.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi2;
extern int __io_putchar(int ch);

/* --------------------------------------------------------------------------
 * MPU9250 CS control
 * -------------------------------------------------------------------------- */

void MPU9250_CS_Low(void)
{
    HAL_GPIO_WritePin(CS_MPU_GPIO_Port, CS_MPU_Pin, GPIO_PIN_RESET);
}

void MPU9250_CS_High(void)
{
    HAL_GPIO_WritePin(CS_MPU_GPIO_Port, CS_MPU_Pin, GPIO_PIN_SET);
}

/* --------------------------------------------------------------------------
 * MPU9250 SPI register write
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU9250_WriteRegister(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;

    reg &= 0x7F;

    MPU9250_CS_Low();

    status = HAL_SPI_Transmit(&hspi2,
                              &reg,
                              1,
                              MPU9250_TIMEOUT);

    if(status != HAL_OK)
    {
        MPU9250_CS_High();
        return status;
    }

    status = HAL_SPI_Transmit(&hspi2,
                              &value,
                              1,
                              MPU9250_TIMEOUT);

    MPU9250_CS_High();

    return status;
}

/* --------------------------------------------------------------------------
 * MPU9250 SPI register read
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU9250_ReadRegister(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;

    reg |= 0x80;

    MPU9250_CS_Low();

    status = HAL_SPI_Transmit(&hspi2,
                              &reg,
                              1,
                              MPU9250_TIMEOUT);

    if(status != HAL_OK)
    {
        MPU9250_CS_High();
        return status;
    }

    status = HAL_SPI_Receive(&hspi2,
                             value,
                             1,
                             MPU9250_TIMEOUT);

    MPU9250_CS_High();

    return status;
}

/* --------------------------------------------------------------------------
 * MPU9250 WHO_AM_I
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU9250_Who_AM_I(uint8_t *chip_id)
{
    HAL_StatusTypeDef status;

    status = MPU9250_ReadRegister(MPU9250_WHO_AM_I, chip_id);

    if(status != HAL_OK)
    {
        return status;
    }

    if(*chip_id != MPU9250_CHIP_ID)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Read MPU external sensor data registers
 * -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU9250_ReadExtSensorData(uint8_t *data,
                                                    uint8_t length)
{
    HAL_StatusTypeDef status;

    uint8_t reg = MPU9250_EXT_SENS_DATA_00 | 0x80;

    MPU9250_CS_Low();

    status = HAL_SPI_Transmit(&hspi2,
                              &reg,
                              1,
                              MPU9250_TIMEOUT);

    if(status != HAL_OK)
    {
        MPU9250_CS_High();
        return status;
    }

    status = HAL_SPI_Receive(&hspi2,
                             data,
                             length,
                             MPU9250_TIMEOUT);

    MPU9250_CS_High();

    return status;
}

/* --------------------------------------------------------------------------
 * Write one AK8963 register through MPU9250 internal I2C master
 * -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU9250_MagWrite(uint8_t reg,
                                          uint8_t value)
{
    HAL_StatusTypeDef status;

    /* AK8963 address + WRITE */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_ADDR,
                                   AK8963_ADDRESS);

    if(status != HAL_OK)
        return status;

    /* AK8963 register */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_REG,
                                   reg);

    if(status != HAL_OK)
        return status;

    /* Data to write */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_DO,
                                   value);

    if(status != HAL_OK)
        return status;

    /* Enable SLV0, perform one-byte write */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_CTRL,
                                   0x81);

    if(status != HAL_OK)
        return status;

    HAL_Delay(10);

    /* Disable SLV0 */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_CTRL,
                                   0x00);

    return status;
}

/* --------------------------------------------------------------------------
 * Read one AK8963 register through MPU9250 internal I2C master
 * -------------------------------------------------------------------------- */

static HAL_StatusTypeDef MPU9250_MagReadRegister(uint8_t reg,
                                                 uint8_t *value)
{
    HAL_StatusTypeDef status;

    uint8_t data;

    /* AK8963 address + READ */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_ADDR,
                                   AK8963_ADDRESS | 0x80);

    if(status != HAL_OK)
        return status;

    /* AK8963 register */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_REG,
                                   reg);

    if(status != HAL_OK)
        return status;

    /* Enable SLV0 and read one byte */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_CTRL,
                                   0x81);

    if(status != HAL_OK)
        return status;

    HAL_Delay(10);

    /* Read result from EXT_SENS_DATA_00 */
    status = MPU9250_ReadExtSensorData(&data, 1);

    if(status != HAL_OK)
        return status;

    *value = data;

    /* Disable SLV0 */
    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_CTRL,
                                   0x00);

    return status;
}

/* --------------------------------------------------------------------------
 * Magnetometer diagnostic
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU9250_MagDiagnostic(uint8_t *who_am_i,
                                        uint8_t *cntl1,
                                        uint8_t *st1,
                                        uint8_t *st2,
                                        int16_t *mag_x,
                                        int16_t *mag_y,
                                        int16_t *mag_z)
{
    HAL_StatusTypeDef status;

    uint8_t data[8];
    uint8_t mst_status;

    /* --------------------------------------------------------------
     * Check MPU internal I2C master status
     * -------------------------------------------------------------- */

    status = MPU9250_ReadRegister(MPU9250_I2C_MST_STATUS,
                                  &mst_status);

    if(status != HAL_OK)
        return status;

    printf("MST_STATUS BEFORE: 0x%02X\r\n", mst_status);

    /* --------------------------------------------------------------
     * Read AK8963 WHO_AM_I
     * Expected = 0x48
     * -------------------------------------------------------------- */

    status = MPU9250_MagReadRegister(AK8963_WHO_AM_I,
                                     who_am_i);

    if(status != HAL_OK)
        return status;

    printf("AK8963 WHO_AM_I: 0x%02X\r\n",
           *who_am_i);

    /* --------------------------------------------------------------
     * Read AK8963 CNTL1
     * -------------------------------------------------------------- */

    status = MPU9250_MagReadRegister(AK8963_CNTL1,
                                     cntl1);

    if(status != HAL_OK)
        return status;

    printf("AK8963 CNTL1: 0x%02X\r\n",
           *cntl1);

    /* --------------------------------------------------------------
     * Put AK8963 back into continuous measurement mode
     * -------------------------------------------------------------- */

    status = MPU9250_MagWrite(AK8963_CNTL1,
                              0x16);

    if(status != HAL_OK)
        return status;

    HAL_Delay(20);

    /* --------------------------------------------------------------
     * Configure SLV0 for continuous read:
     *
     * ST1
     * HXL
     * HXH
     * HYL
     * HYH
     * HZL
     * HZH
     * ST2
     * -------------------------------------------------------------- */

    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_ADDR,
                                   AK8963_ADDRESS | 0x80);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_REG,
                                   AK8963_ST1);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_I2C_SLV0_CTRL,
                                   0x88);

    if(status != HAL_OK)
        return status;

    HAL_Delay(20);

    /* --------------------------------------------------------------
     * Read eight bytes from EXT_SENS_DATA_00
     * -------------------------------------------------------------- */

    status = MPU9250_ReadExtSensorData(data, 8);

    if(status != HAL_OK)
        return status;

    /* Print raw bytes */
    printf("MAG BYTES: ");

    for(int i = 0; i < 8; i++)
    {
        printf("%02X ", data[i]);
    }

    printf("\r\n");

    /* --------------------------------------------------------------
     * Decode data
     * -------------------------------------------------------------- */

    *st1 = data[0];

    *mag_x = (int16_t)((data[2] << 8) | data[1]);

    *mag_y = (int16_t)((data[4] << 8) | data[3]);

    *mag_z = (int16_t)((data[6] << 8) | data[5]);

    *st2 = data[7];

    /* --------------------------------------------------------------
     * Read I2C master status again
     * -------------------------------------------------------------- */

    status = MPU9250_ReadRegister(MPU9250_I2C_MST_STATUS,
                                  &mst_status);

    if(status != HAL_OK)
        return status;

    printf("MST_STATUS AFTER: 0x%02X\r\n",
           mst_status);

    if(mst_status & 0x80)
        printf("WARNING: SLV4 NACK\r\n");

    if(mst_status & 0x40)
        printf("WARNING: I2C LOST ARBITRATION\r\n");

    if(mst_status & 0x10)
        printf("WARNING: SLV1 NACK\r\n");

    if(mst_status & 0x08)
        printf("WARNING: SLV2 NACK\r\n");

    if(mst_status & 0x04)
        printf("WARNING: SLV3 NACK\r\n");

    if(mst_status & 0x02)
        printf("WARNING: SLV2 NACK\r\n");

    if(mst_status & 0x01)
        printf("I2C MASTER CLOCK ERROR\r\n");

    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * MPU9250 initialization
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU9250_Init(void)
{
    uint8_t PWR_MGMT_1  = 0x01;
    uint8_t PWR_MGMT_2  = 0x00;

    uint8_t CONFIG       = 0x03;
    uint8_t SMPLRT_DIV   = 0x09;

    uint8_t GYRO_CONFIG  = 0x00;
    uint8_t ACCEL_CONFIG = 0x00;
    uint8_t ACCEL_CONFIG2 = 0x03;

    uint8_t INT_ENABLE   = 0x00;

    /* Enable MPU9250 internal I2C master */
    uint8_t USER_CTRL    = 0x20;

    /* I2C master clock */
    uint8_t I2C_MST_CTRL = 0x0D;

    HAL_StatusTypeDef status;

    /* --------------------------------------------------------------
     * MPU9250 clock
     * -------------------------------------------------------------- */

    status = MPU9250_WriteRegister(MPU9250_PWR_MGMT_1,
                                   PWR_MGMT_1);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_PWR_MGMT_2,
                                   PWR_MGMT_2);

    if(status != HAL_OK)
        return status;

    HAL_Delay(10);

    /* --------------------------------------------------------------
     * Enable internal I2C master
     * -------------------------------------------------------------- */

    status = MPU9250_WriteRegister(MPU9250_USER_CTRL,
                                   USER_CTRL);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_I2C_MST_CTRL,
                                   I2C_MST_CTRL);

    if(status != HAL_OK)
        return status;

    HAL_Delay(10);

    /* --------------------------------------------------------------
     * AK8963 soft reset
     * -------------------------------------------------------------- */

    status = MPU9250_MagWrite(AK8963_CNTL2,
                              0x01);

    if(status != HAL_OK)
        return status;

    HAL_Delay(20);

    /* --------------------------------------------------------------
     * AK8963 power down
     * -------------------------------------------------------------- */

    status = MPU9250_MagWrite(AK8963_CNTL1,
                              0x00);

    if(status != HAL_OK)
        return status;

    HAL_Delay(10);

    /* --------------------------------------------------------------
     * AK8963 continuous measurement mode 2
     *
     * 0x16:
     * 0x10 = 16-bit output
     * 0x06 = continuous measurement mode 2
     * -------------------------------------------------------------- */

    status = MPU9250_MagWrite(AK8963_CNTL1,
                              0x16);

    if(status != HAL_OK)
        return status;

    HAL_Delay(20);

    /* --------------------------------------------------------------
     * MPU9250 configuration
     * -------------------------------------------------------------- */

    status = MPU9250_WriteRegister(MPU9250_CONFIG,
                                   CONFIG);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_SMPLRT_DIV,
                                   SMPLRT_DIV);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_GYRO_CONFIG,
                                   GYRO_CONFIG);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_ACCEL_CONFIG,
                                   ACCEL_CONFIG);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_ACCEL_CONFIG2,
                                   ACCEL_CONFIG2);

    if(status != HAL_OK)
        return status;

    status = MPU9250_WriteRegister(MPU9250_INT_ENABLE,
                                   INT_ENABLE);

    if(status != HAL_OK)
        return status;

    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Read accelerometer + gyro + temperature
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU9250_ReadRaw(int16_t *accel_raw_x,
                                  int16_t *accel_raw_y,
                                  int16_t *accel_raw_z,
                                  int16_t *gyro_raw_x,
                                  int16_t *gyro_raw_y,
                                  int16_t *gyro_raw_z,
                                  int16_t *temp_raw)
{
    uint8_t data[14];

    uint8_t reg = MPU9250_ACCEL_XOUT_H | 0x80;

    HAL_StatusTypeDef status;

    MPU9250_CS_Low();

    status = HAL_SPI_Transmit(&hspi2,
                              &reg,
                              1,
                              MPU9250_TIMEOUT);

    if(status != HAL_OK)
    {
        MPU9250_CS_High();
        return status;
    }

    status = HAL_SPI_Receive(&hspi2,
                             data,
                             14,
                             MPU9250_TIMEOUT);

    if(status != HAL_OK)
    {
        MPU9250_CS_High();
        return status;
    }

    *accel_raw_x = (int16_t)((data[0] << 8) | data[1]);

    *accel_raw_y = (int16_t)((data[2] << 8) | data[3]);

    *accel_raw_z = (int16_t)((data[4] << 8) | data[5]);

    *temp_raw = (int16_t)((data[6] << 8) | data[7]);

    *gyro_raw_x = (int16_t)((data[8] << 8) | data[9]);

    *gyro_raw_y = (int16_t)((data[10] << 8) | data[11]);

    *gyro_raw_z = (int16_t)((data[12] << 8) | data[13]);

    MPU9250_CS_High();

    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Read magnetometer raw data
 * -------------------------------------------------------------------------- */

HAL_StatusTypeDef MPU9250_ReadMagRaw(int16_t *mag_raw_x,
                                     int16_t *mag_raw_y,
                                     int16_t *mag_raw_z)
{
    uint8_t data[8];

    HAL_StatusTypeDef status;

    status = MPU9250_ReadExtSensorData(data, 8);

    if(status != HAL_OK)
        return status;

    *mag_raw_x = (int16_t)((data[2] << 8) | data[1]);

    *mag_raw_y = (int16_t)((data[4] << 8) | data[3]);

    *mag_raw_z = (int16_t)((data[6] << 8) | data[5]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_DirectWhoAmI(uint8_t *chip_id)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = MPU9250_WHO_AM_I | 0x80;
    tx[1] = 0x00;

    MPU9250_CS_Low();

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi2,
                                tx,
                                rx,
                                2,
                                MPU9250_TIMEOUT);

    MPU9250_CS_High();

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * rx[0] is the response during the register-address byte.
     * rx[1] is the actual WHO_AM_I response.
     */
    *chip_id = rx[1];

    printf("WHO_AM_I TX: %02X %02X\r\n", tx[0], tx[1]);
    printf("WHO_AM_I RX: %02X %02X\r\n", rx[0], rx[1]);

    return HAL_OK;
}
