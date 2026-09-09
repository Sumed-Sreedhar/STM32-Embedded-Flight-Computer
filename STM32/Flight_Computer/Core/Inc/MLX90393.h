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

/* Low-noise magnetometer config values */
#define MLX90393_CONF0_VALUE   0x007C   /* GAIN_SEL=7 (1x), HALLCONF=0xC */
#define MLX90393_CONF2_VALUE   0x000A   /* OSR=3, DIG_FILT=3, RES_XYZ=0, OSR2=0 (~16ms conv, low noise) */

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

void MLX90393_RawTo_uT(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z, float *X_uT, float *Y_uT, float *Z_uT);

#endif /* INC_MLX90393_H_ */
