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

/* Measurement command bitmasks (X, Y, Z enabled) */
#define MLX90393_SM_XYZ    (MLX90393_CMD_SM | 0x0E)
#define MLX90393_RM_XYZ    (MLX90393_CMD_RM | 0x0E)

/* MLX90393 Register Addresses */
#define MLX90393_REG_CONF0 0x00
#define MLX90393_REG_CONF1 0x01
#define MLX90393_REG_CONF2 0x02
#define MLX90393_REG_CONF3 0x03

/* CONF0 Register Fields (0x00) */
#define MLX90393_CONF0_HALLCONF_MASK       0x000F // Bits 3:0
#define MLX90393_CONF0_GAIN_SEL_MASK       0x0070 // Bits 6:4
#define MLX90393_CONF0_Z_SERIES_MASK       0x0080 // Bit 7

#define MLX90393_CONF0_HALLCONF            0x000C // Default Hall setting
#define MLX90393_CONF0_GAIN_SEL            0x0050 // Gain = 5 (Bits 6:4 = 0x50)
#define MLX90393_CONF0_Z_SERIES            0x0000

/* CONF1 Register Fields (0x01) */
#define MLX90393_CONF1_BURST_DATA_RATE     0x003F // Bits 5:0
#define MLX90393_CONF1_BURST_SEL           0x03C0 // Bits 9:6
#define MLX90393_CONF1_TCMP_EN             0x0400 // Bit 10
#define MLX90393_CONF1_EXT_TRIG            0x1000 // Bit 12
#define MLX90393_CONF1_WOC_DIFF            0x2000 // Bit 13
#define MLX90393_CONF1_COMM_MODE_MASK      0x6000 // Bits 14:13
#define MLX90393_CONF1_TRIG_INT_SEL        0x8000 // Bit 15

/* CONF2 Register Fields (0x02) */
#define MLX90393_CONF2_OSR_MASK            0x0003 // Bits 1:0
#define MLX90393_CONF2_DIG_FILT_MASK       0x001C // Bits 4:2
#define MLX90393_CONF2_RES_XYZ_MASK        0x0FC0 // Bits 11:6
#define MLX90393_CONF2_OSR2_MASK           0x3000 // Bits 13:12

#define MLX90393_CONF2_OSR                 0x0000
#define MLX90393_CONF2_DIG_FILT            0x0000
#define MLX90393_CONF2_RES_XYZ             0x0000
#define MLX90393_CONF2_OSR2                0x0000

/* Driver Definitions */
#define MLX90393_TIMEOUT    100
#define MLX90393_I2C_ADDR  (0x0C << 1)

/* Function Prototypes */
HAL_StatusTypeDef MLX90393_I2C_Test(void);
HAL_StatusTypeDef MLX90393_ReadRegister(uint8_t reg, uint16_t *value);
HAL_StatusTypeDef MLX90393_WriteRegister(uint8_t reg, uint16_t value);
HAL_StatusTypeDef MLX90393_Reset(void);
HAL_StatusTypeDef MLX90393_Configure(void);
HAL_StatusTypeDef MLX90393_StartMeasurement(void);
HAL_StatusTypeDef MLX90393_ReadMeasurement(int16_t *x, int16_t *y, int16_t *z);

/* Sensitivity Factors for GAIN_SEL = 5 (0x50), RES = 0 */
#define MLX90393_SENS_XY_G5    0.268f // uT per LSB
#define MLX90393_SENS_Z_G5     0.489f // uT per LSB

/* Calibrated / Converted Data Structure */
typedef struct {
    float x_uT;
    float y_uT;
    float z_uT;
} MLX90393_CalibratedData_t;

/* Hard-Iron Offset Calibration Structure */
typedef struct {
    int16_t x_offset;
    int16_t y_offset;
    int16_t z_offset;
} MLX90393_Offsets_t;

/* Function Prototypes */
void MLX90393_ConvertToMicroTesla(int16_t x_raw, int16_t y_raw, int16_t z_raw,
                                  MLX90393_Offsets_t *offsets,
                                  MLX90393_CalibratedData_t *output);
#endif /* INC_MLX90393_H_ */

