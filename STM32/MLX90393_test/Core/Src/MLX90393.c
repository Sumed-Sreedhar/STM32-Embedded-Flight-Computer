/*
 * MLX90393.c
 *
 *  Created on: Aug 30, 2026
 *      Author: sumed
 */

#include "MLX90393.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;


/*----------------------------------------------------------*/
/* I2C Device Test                                          */
/*----------------------------------------------------------*/

HAL_StatusTypeDef MLX90393_I2C_Test(void)
{
    return HAL_I2C_IsDeviceReady(
        &hi2c1,
        MLX90393_I2C_ADDR,
        3,
        MLX90393_TIMEOUT
    );
}


/*----------------------------------------------------------*/
/* Read Register                                            */
/*----------------------------------------------------------*/

HAL_StatusTypeDef MLX90393_ReadRegister(
    uint8_t reg,
    uint16_t *value)
{
    uint8_t rx[3];

    uint16_t mem_addr =
        ((uint16_t)MLX90393_CMD_RR << 8) |
        (uint16_t)(reg << 2);

    /*
     * Sends:
     *
     * 0x50
     * reg << 2
     *
     * then reads:
     *
     * status
     * data MSB
     * data LSB
     */
    if (HAL_I2C_Mem_Read(
            &hi2c1,
            MLX90393_I2C_ADDR,
            mem_addr,
            I2C_MEMADD_SIZE_16BIT,
            rx,
            3,
            MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* ERROR flag = status bit 4 */
    if (rx[0] & 0x10)
    {
        return HAL_ERROR;
    }

    *value = ((uint16_t)rx[1] << 8) | rx[2];

    return HAL_OK;
}


/*----------------------------------------------------------*/
/* Write Register                                           */
/*----------------------------------------------------------*/

HAL_StatusTypeDef MLX90393_WriteRegister(
    uint8_t reg,
    uint16_t value)
{
    uint8_t tx[4];

    tx[0] = MLX90393_CMD_WR;
    tx[1] = (uint8_t)(value >> 8);
    tx[2] = (uint8_t)(value & 0xFF);
    tx[3] = (uint8_t)(reg << 2);

    if (HAL_I2C_Master_Transmit(
            &hi2c1,
            MLX90393_I2C_ADDR,
            tx,
            4,
            MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


/*----------------------------------------------------------*/
/* Reset                                                    */
/*----------------------------------------------------------*/

HAL_StatusTypeDef MLX90393_Reset(void)
{
    uint8_t cmd = MLX90393_CMD_RT;
    uint8_t status_byte = 0;

    if (HAL_I2C_Master_Transmit(
            &hi2c1,
            MLX90393_I2C_ADDR,
            &cmd,
            1,
            MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_I2C_Master_Receive(
        &hi2c1,
        MLX90393_I2C_ADDR,
        &status_byte,
        1,
        MLX90393_TIMEOUT
    );

    HAL_Delay(5);

    return HAL_OK;
}


/*----------------------------------------------------------*/
/* Configuration                                            */
/*----------------------------------------------------------*/

HAL_StatusTypeDef MLX90393_Configure(void)
{
    uint16_t conf0 = 0;
    uint16_t conf1 = 0;
    uint16_t conf2 = 0;

    if (MLX90393_ReadRegister(
            MLX90393_REG_CONF0,
            &conf0) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (MLX90393_ReadRegister(
            MLX90393_REG_CONF1,
            &conf1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (MLX90393_ReadRegister(
            MLX90393_REG_CONF2,
            &conf2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * For now, leave the existing sensor configuration alone.
     *
     * We are deliberately not modifying CONF0/CONF1/CONF2
     * while establishing the basic raw measurement path.
     */

    return HAL_OK;
}


/*----------------------------------------------------------*/
/* Start Measurement                                        */
/*----------------------------------------------------------*/

HAL_StatusTypeDef MLX90393_StartMeasurement(void)
{
    uint8_t cmd = MLX90393_SM_XYZ;
    uint8_t status_byte = 0;

    if (HAL_I2C_Master_Transmit(
            &hi2c1,
            MLX90393_I2C_ADDR,
            &cmd,
            1,
            MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Read the status response from the SM command.
     */
    HAL_I2C_Master_Receive(
        &hi2c1,
        MLX90393_I2C_ADDR,
        &status_byte,
        1,
        MLX90393_TIMEOUT
    );

    return HAL_OK;
}


/*----------------------------------------------------------*/
/* Read Measurement                                        */
/*----------------------------------------------------------*/

HAL_StatusTypeDef MLX90393_ReadMeasurement(
    int16_t *x,
    int16_t *y,
    int16_t *z)
{
    uint8_t rx[7];

    /*
     * Send RM_XYZ and receive:
     *
     * rx[0] = status
     * rx[1] = X MSB
     * rx[2] = X LSB
     * rx[3] = Y MSB
     * rx[4] = Y LSB
     * rx[5] = Z MSB
     * rx[6] = Z LSB
     */
    if (HAL_I2C_Mem_Read(
            &hi2c1,
            MLX90393_I2C_ADDR,
            MLX90393_RM_XYZ,
            I2C_MEMADD_SIZE_8BIT,
            rx,
            7,
            MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* ERROR flag = status bit 4 */
    if (rx[0] & 0x10)
    {
        return HAL_ERROR;
    }

    *x = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);

    *y = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);

    *z = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

    return HAL_OK;
}
