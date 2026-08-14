/*
 * BMP280.h
 *
 *  Created on: 30-Jul-2026
 *      Author: sumed
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stdint.h"

void BMP280_CS_Low(void);
void BMP280_CS_High(void);
HAL_StatusTypeDef BMP280_ReadID(uint8_t *chip_id);
HAL_StatusTypeDef BMP280_Init(void);
HAL_StatusTypeDef BMP280_ReadRaw(int32_t *adc_T, int32_t *adc_P);
HAL_StatusTypeDef BMP280_LoadCalibration(void);
int32_t BMP280_CompensateTemp(int32_t adc_T);
uint32_t BMP280_CompensatePressure(int32_t adc_P);

#define BMP280_CHIP_ID 0x58
#define BMP280_REG_ID 0xD0
#define BMP280_TIMEOUT 100
#define CTRL_MEAS 0xF4
#define CONFIG 0xF5
#define CALIBRATION_DATA_START 0x88
#define PRESSURE_DATA_START 0xF7

#endif /* INC_BMP280_H_ */
