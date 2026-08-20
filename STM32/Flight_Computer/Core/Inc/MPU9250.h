/*
 * MPU9250.h
 *
 *  Created on: 01-Aug-2026
 *      Author: sumed
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "main.h"
#include "stdint.h"

void MPU9250_CS_Low(void);
void MPU9250_CS_High(void);
HAL_StatusTypeDef MPU9250_WriteRegister(uint8_t  register,uint8_t value);
HAL_StatusTypeDef MPU9250_ReadRegister(uint8_t  reg,uint8_t *value);

HAL_StatusTypeDef MPU9250_Who_AM_I(uint8_t *chip_id);
HAL_StatusTypeDef MPU9250_Init(void);
HAL_StatusTypeDef MPU9250_ReadRaw(int16_t *accel_raw_x,int16_t *accel_raw_y,int16_t *accel_raw_z, int16_t *gyro_raw_x,int16_t *gyro_raw_y, int16_t *gyro_raw_z, int16_t *temp_raw);

#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_INT_ENABLE       0x38
#define MPU9250_PWR_MGMT_1       0x6B
#define MPU9250_PWR_MGMT_2       0x6C
#define MPU9250_WHO_AM_I         0x75
#define MPU9250_ACCEL_XOUT_H     0x3B
#define MPU9250_TIMEOUT 		 100
#define MPU9250_CHIP_ID 		 0x70


#endif /* INC_MPU9250_H_ */
