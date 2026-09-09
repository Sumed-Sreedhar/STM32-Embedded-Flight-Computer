/*
 * MLX90393.c
 *
 *  Created on: Aug 30, 2026
 *      Author: sumed
 */

#include "MLX90393.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;


HAL_StatusTypeDef MLX90393_I2C_Test(void)
{
    return HAL_I2C_IsDeviceReady(
        &hi2c1,
        MLX90393_I2C_ADDR,
        3,
        MLX90393_TIMEOUT
    );
}


HAL_StatusTypeDef MLX90393_ReadRegister(
    uint8_t reg,
    uint16_t *value)
{
    uint8_t rx[3];

    uint16_t mem_addr =
        ((uint16_t)MLX90393_CMD_RR << 8) |
        (uint16_t)(reg << 2);

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

    if (rx[0] & 0x10)
    {
        return HAL_ERROR;
    }

    *value = ((uint16_t)rx[1] << 8) | rx[2];

    return HAL_OK;
}


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

HAL_StatusTypeDef MLX90393_Reset(void)
{
    uint8_t cmd = MLX90393_CMD_RT;

    if (HAL_I2C_Master_Transmit(
            &hi2c1,
            MLX90393_I2C_ADDR,
            &cmd,
            1,
            MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(5); /* >= T_POR (max 1.5ms per datasheet); margin included */

    return HAL_OK;
}

HAL_StatusTypeDef MLX90393_Configure(void)
{
    uint16_t conf0_check = 0;
    uint16_t conf2_check = 0;

    if (MLX90393_WriteRegister(MLX90393_REG_CONF0, MLX90393_CONF0_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(2);

    if (MLX90393_WriteRegister(MLX90393_REG_CONF2, MLX90393_CONF2_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(2);

    if (MLX90393_ReadRegister(MLX90393_REG_CONF0, &conf0_check) != HAL_OK ||
        conf0_check != MLX90393_CONF0_VALUE)
    {
        return HAL_ERROR;
    }

    HAL_Delay(2);

    if (MLX90393_ReadRegister(MLX90393_REG_CONF2, &conf2_check) != HAL_OK ||
        conf2_check != MLX90393_CONF2_VALUE)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

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


    HAL_I2C_Master_Receive(
        &hi2c1,
        MLX90393_I2C_ADDR,
        &status_byte,
        1,
        MLX90393_TIMEOUT
    );

    return HAL_OK;
}


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

    if (rx[0] & 0x10)
    {
        return HAL_ERROR;
    }

    *x = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);

    *y = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);

    *z = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

    return HAL_OK;
}


void MLX90393_RawTo_uT(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z, float *X_uT, float *Y_uT, float *Z_uT)
{
	*X_uT = (*raw_x) * 0.150f;
	*Y_uT = (*raw_y) * 0.150f;
	*Z_uT = (*raw_z) * 0.242f;

}

