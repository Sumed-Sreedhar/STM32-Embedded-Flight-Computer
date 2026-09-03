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
    return HAL_I2C_IsDeviceReady(&hi2c1, MLX90393_I2C_ADDR, 3, MLX90393_TIMEOUT);
}

HAL_StatusTypeDef MLX90393_ReadRegister(uint8_t reg, uint16_t *value)
{
    uint8_t rx[3];
    uint16_t mem_addr = ((uint16_t)MLX90393_CMD_RR << 8) | (uint16_t)(reg << 2);

    /* Sends 0x50 + reg<<2, issues Repeated START, reads 3 bytes */
    if (HAL_I2C_Mem_Read(&hi2c1, MLX90393_I2C_ADDR, mem_addr,
                         I2C_MEMADD_SIZE_16BIT, rx, 3, MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Check status byte ERROR flag (Bit 4) */
    if (rx[0] & 0x10)
    {
        return HAL_ERROR;
    }

    *value = ((uint16_t)rx[1] << 8) | rx[2];
    return HAL_OK;
}

HAL_StatusTypeDef MLX90393_WriteRegister(uint8_t reg, uint16_t value)
{
    uint8_t tx[4];
    uint8_t status_byte = 0;

    tx[0] = MLX90393_CMD_WR;
    tx[1] = (uint8_t)(value >> 8);
    tx[2] = (uint8_t)(value & 0xFF);
    tx[3] = (uint8_t)(reg << 2);

    /* Transmit 4-byte WR packet */
    if (HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDR, tx, 4, MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Read back 1-byte status response */
    if (HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDR, &status_byte, 1, MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Check status byte ERROR flag (Bit 4) */
    if (status_byte & 0x10)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MLX90393_Reset(void)
{
    uint8_t cmd = MLX90393_CMD_RT;
    uint8_t status_byte = 0;

    if (HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDR, &cmd, 1, MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDR, &status_byte, 1, MLX90393_TIMEOUT);
    HAL_Delay(5); // Reset cycle delay
    return HAL_OK;
}

HAL_StatusTypeDef MLX90393_Configure(void)
{
    uint16_t conf0 = 0, conf1 = 0, conf2 = 0;

    if (MLX90393_ReadRegister(MLX90393_REG_CONF0, &conf0) != HAL_OK) return HAL_ERROR;
    if (MLX90393_ReadRegister(MLX90393_REG_CONF1, &conf1) != HAL_OK) return HAL_ERROR;
    if (MLX90393_ReadRegister(MLX90393_REG_CONF2, &conf2) != HAL_OK) return HAL_ERROR;

    /* CONF0 Setup */
    conf0 &= ~(MLX90393_CONF0_Z_SERIES_MASK | MLX90393_CONF0_GAIN_SEL_MASK | MLX90393_CONF0_HALLCONF_MASK);
    conf0 |= (MLX90393_CONF0_Z_SERIES | MLX90393_CONF0_GAIN_SEL | MLX90393_CONF0_HALLCONF);

    /* CONF1 Setup */
    conf1 &= ~(MLX90393_CONF1_TRIG_INT_SEL | MLX90393_CONF1_COMM_MODE_MASK |
               MLX90393_CONF1_WOC_DIFF | MLX90393_CONF1_EXT_TRIG | MLX90393_CONF1_TCMP_EN);
    conf1 |= MLX90393_CONF1_TRIG_INT_SEL;

    /* CONF2 Setup */
    conf2 &= ~(MLX90393_CONF2_OSR2_MASK | MLX90393_CONF2_DIG_FILT_MASK |
               MLX90393_CONF2_OSR_MASK | MLX90393_CONF2_RES_XYZ_MASK);
    conf2 |= (MLX90393_CONF2_OSR2 | MLX90393_CONF2_DIG_FILT | MLX90393_CONF2_OSR | MLX90393_CONF2_RES_XYZ);

    /* Write updated values */
    if (MLX90393_WriteRegister(MLX90393_REG_CONF0, conf0) != HAL_OK) return HAL_ERROR;
    if (MLX90393_WriteRegister(MLX90393_REG_CONF1, conf1) != HAL_OK) return HAL_ERROR;
    if (MLX90393_WriteRegister(MLX90393_REG_CONF2, conf2) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MLX90393_StartMeasurement(void)
{
    uint8_t cmd = MLX90393_SM_XYZ;
    uint8_t status_byte = 0;

    if (HAL_I2C_Master_Transmit(&hi2c1, MLX90393_I2C_ADDR, &cmd, 1, MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_I2C_Master_Receive(&hi2c1, MLX90393_I2C_ADDR, &status_byte, 1, MLX90393_TIMEOUT);
    return HAL_OK;
}

HAL_StatusTypeDef MLX90393_ReadMeasurement(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t rx[7];

    /* Sends 0x4E (RM_XYZ), issues Repeated START, reads 7 bytes */
    if (HAL_I2C_Mem_Read(&hi2c1, MLX90393_I2C_ADDR, MLX90393_RM_XYZ,
                         I2C_MEMADD_SIZE_8BIT, rx, 7, MLX90393_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Check status byte ERROR flag (Bit 4) */
    if (rx[0] & 0x10)
    {
        return HAL_ERROR;
    }

    *x = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);
    *y = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
    *z = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

    return HAL_OK;
}

void MLX90393_ConvertToMicroTesla(int16_t x_raw, int16_t y_raw, int16_t z_raw,
                                  MLX90393_Offsets_t *offsets,
                                  MLX90393_CalibratedData_t *output)
{
    int16_t x_corrected = x_raw;
    int16_t y_corrected = y_raw;
    int16_t z_corrected = z_raw;

    if (offsets != NULL)
    {
        x_corrected -= offsets->x_offset;
        y_corrected -= offsets->y_offset;
        z_corrected -= offsets->z_offset;
    }

    /* Apply sensitivity factors */
    output->x_uT = (float)x_corrected * MLX90393_SENS_XY_G5;
    output->y_uT = (float)y_corrected * MLX90393_SENS_XY_G5;
    output->z_uT = (float)z_corrected * MLX90393_SENS_Z_G5;
}
