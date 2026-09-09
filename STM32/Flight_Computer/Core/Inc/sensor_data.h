/*
 * sensor_data.h
 *
 *  Created on: 30-Jul-2026
 *      Author: sumed
 */

#ifndef INC_SENSOR_DATA_H_
#define INC_SENSOR_DATA_H_

#include "stdint.h"

typedef struct
{
	/* BMP280 Measurements*/
    int32_t temperature_raw;
    int32_t pressure_raw;
    float temperature;
    float pressure;
    float altitude;

    /* MPU9250 Measurements*/
    int16_t accel_raw_x;
    int16_t accel_raw_y;
    int16_t accel_raw_z;
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
    int16_t temp_raw;

    /* MLX90393 Measurements */
    int16_t mag_raw_x;
    int16_t mag_raw_y;
    int16_t mag_raw_z;

    float mag_x_uT;
    float mag_y_uT;
    float mag_z_uT;

} sensor_data_t;

extern sensor_data_t sensor_data;


#endif /* INC_SENSOR_DATA_H_ */
