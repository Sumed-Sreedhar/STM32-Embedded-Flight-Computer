/*
 * MLX90393.h
 *
 *  Created on: Aug 30, 2026
 *      Author: sumed
 */

#ifndef INC_MLX90393_H_
#define INC_MLX90393_H_

#include "main.h"
#include "sensor_data.h"

/* MLX90393 Command Definitions */
#define MLX90393_CMD_EX    0x80
#define MLX90393_CMD_RR    0x50
#define MLX90393_CMD_WR    0x60
#define MLX90393_CMD_SM    0x30
#define MLX90393_CMD_RM    0x40
#define MLX90393_CMD_RT    0xF0

/* Measurement commands: X + Y + Z */
#define MLX90393_SM_XYZ    (MLX90393_CMD_SM | 0x0E)
#define MLX90393_RM_XYZ    (MLX90393_CMD_RM | 0x0E)

/* Register Addresses */
#define MLX90393_REG_CONF0 0x00
#define MLX90393_REG_CONF1 0x01
#define MLX90393_REG_CONF2 0x02
#define MLX90393_REG_CONF3 0x03

/* Driver Definitions */
#define MLX90393_TIMEOUT    100
#define MLX90393_I2C_ADDR  (0x0C << 1)

/* Function Prototypes */

HAL_StatusTypeDef MLX90393_I2C_Test(void);

HAL_StatusTypeDef MLX90393_ReadRegister(
    uint8_t reg,
    uint16_t *value);

HAL_StatusTypeDef MLX90393_WriteRegister(
    uint8_t reg,
    uint16_t value);

HAL_StatusTypeDef MLX90393_Reset(void);

HAL_StatusTypeDef MLX90393_Configure(void);

HAL_StatusTypeDef MLX90393_StartMeasurement(void);

HAL_StatusTypeDef MLX90393_ReadMeasurement(
    int16_t *x,
    int16_t *y,
    int16_t *z);

#endif /* INC_MLX90393_H_ */
