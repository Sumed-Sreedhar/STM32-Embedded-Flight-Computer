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
HAL_StatusTypeDef MPU9250_ReadMagRaw(int16_t *mag_raw_x, int16_t *mag_raw_y, int16_t *mag_raw_z);

HAL_StatusTypeDef MPU9250_MagDiagnostic(uint8_t *who_am_i,
                                        uint8_t *cntl1,
                                        uint8_t *st1,
                                        uint8_t *st2,
                                        int16_t *mag_x,
                                        int16_t *mag_y,
                                        int16_t *mag_z);

HAL_StatusTypeDef MPU9250_DirectWhoAmI(uint8_t *chip_id);

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
#define MPU9250_CHIP_ID 		 0x71

#define MPU9250_USER_CTRL          0x6A
#define MPU9250_I2C_MST_CTRL      0x24
#define MPU9250_I2C_MST_STATUS    0x36

#define MPU9250_I2C_SLV0_ADDR     0x25
#define MPU9250_I2C_SLV0_REG      0x26
#define MPU9250_I2C_SLV0_CTRL     0x27
#define MPU9250_I2C_SLV0_DO       0x63

#define MPU9250_EXT_SENS_DATA_00  0x49

#define AK8963_ADDRESS             0x0C

#define AK8963_WHO_AM_I           0x00
#define AK8963_ST1                0x02
#define AK8963_HXL                0x03
#define AK8963_HXH                0x04
#define AK8963_HYL                0x05
#define AK8963_HYH                0x06
#define AK8963_HZL                0x07
#define AK8963_HZH                0x08
#define AK8963_ST2                0x09
#define AK8963_CNTL1              0x0A
#define AK8963_CNTL2              0x0B
#define AK8963_WHO_AM_I_VALUE   0x48
#define AK8963_MODE_POWER_DOWN  0x00
#define AK8963_MODE_CONT_2      0x16
#define AK8963_ST1_DRDY         0x01
#define AK8963_ST2_HOFL         0x08

#endif /* INC_MPU9250_H_ */
